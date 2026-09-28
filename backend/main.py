import os
from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from typing import List, Dict, Any, Optional
import json

from .models import (
    DroneSpec, SensorSpec, MissionRequest, GEOSCAN_DRONE_CATALOG,
    SENSOR_CATALOG, WindConfig, LaunchPoint
)
from .data_loader import data_loader
from .photogrammetry import calculate_photogrammetry
from .path_planner import plan_multi_uav_mission
from .exporters import export_plan_to_geojson, export_plan_to_kml, export_plan_to_qgc_mission

app = FastAPI(
    title="Geoscan FleetCommander AI",
    description="Интеллектуальный сервис планирования и распределения беспилотных авиационных работ для группы БВС Геоскан",
    version="1.0.0"
)

# CORS для локальной разработки и веб-клиента
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.on_event("startup")
def startup_event():
    # Загружаем датасет хакатона при старте сервера
    data_loader.load_all()

@app.get("/api/health")
def health_check():
    return {
        "status": "healthy",
        "data_loaded": data_loader.is_loaded,
        "parcels_count": len(data_loader.survey_polygons),
        "zones_count": len(data_loader.airspace_zones),
        "obstacles_count": len(data_loader.obstacles)
    }

@app.get("/api/catalog")
def get_catalog():
    """Справочник доступных БВС, полезных нагрузок и критериев оптимизации"""
    return {
        "drones": list(GEOSCAN_DRONE_CATALOG.values()),
        "sensors": list(SENSOR_CATALOG.values()),
        "criteria": [
            {
                "id": "min_makespan",
                "name": "Минимизация времени выполнения (Параллельная работа)",
                "description": "Распределяет площадь между всеми БВС флота пропорционально их скорости для максимальной скорости закрытия миссии."
            },
            {
                "id": "min_flight_time",
                "name": "Минимизация суммарного налета (Экономия ресурса)",
                "description": "Минимизирует суммарный износ техники и расход аккумуляторов, отдавая приоритет самолету Геоскан 201."
            }
        ]
    }

@app.get("/api/parcels")
def list_parcels(limit: int = 50, offset: int = 0):
    """Список тестовых участков из 'Границы полетов.kml'"""
    items = []
    for p in data_loader.survey_polygons[offset:offset + limit]:
        items.append({
            "fid": p["fid"],
            "area_ha": round(p["area_m2"] / 10000.0, 2),
            "centroid": p["centroid"],
            "bounds": p["bounds"]
        })
    return {
        "total": len(data_loader.survey_polygons),
        "offset": offset,
        "limit": limit,
        "parcels": items
    }

@app.get("/api/parcels/{fid}")
def get_parcel(fid: int):
    """Получение конкретного полигона в формате GeoJSON по его FID"""
    for p in data_loader.survey_polygons:
        if p["fid"] == fid:
            coords = [list(c) for c in p["geometry"].exterior.coords]
            return {
                "type": "Feature",
                "properties": {
                    "fid": p["fid"],
                    "area_ha": round(p["area_m2"] / 10000.0, 2)
                },
                "geometry": {
                    "type": "Polygon",
                    "coordinates": [coords]
                }
            }
    raise HTTPException(status_code=404, detail=f"Parcel with FID {fid} not found")

@app.get("/api/airspace-zones")
def get_airspace_zones(minx: float = 38.0, miny: float = 54.0, maxx: float = 39.5, maxy: float = 55.5):
    """
    Зоны ограничений из 'Московская зона.kml' в разработанном стандарте 4D-Airspace
    с отфильтрованными высотами и типами.
    """
    zones = data_loader.get_zones_near((minx, miny, maxx, maxy), buffer_deg=0.08)
    features = []
    for z in zones[:80]: # Лимит для быстрой отрисовки в браузере
        geom = z["geometry"]
        if geom.geom_type == 'Polygon':
            poly_coords = [[list(c) for c in geom.exterior.coords]]
            geom_type = 'Polygon'
        elif geom.geom_type == 'MultiPolygon':
            poly_coords = [[[list(c) for c in poly.exterior.coords]] for poly in geom.geoms]
            geom_type = 'MultiPolygon'
        else:
            continue
            
        features.append({
            "type": "Feature",
            "geometry": {
                "type": geom_type,
                "coordinates": poly_coords[0] if geom_type == 'Polygon' else poly_coords
            },
            "properties": {
                "name": z["name"],
                "type": z["type"],
                "altitude_info": z["altitude_info"],
                "stroke": "#d90429",
                "fill": "#ef233c",
                "fill-opacity": 0.2
            }
        })
    return {
        "type": "FeatureCollection",
        "standard_version": "4D-Airspace-Draft-v1.0",
        "features": features
    }

@app.get("/api/obstacles")
def get_obstacles(minx: float = 38.0, miny: float = 54.0, maxx: float = 39.5, maxy: float = 55.5):
    """3D высотные препятствия из 'obstacles_Московская область.kml'"""
    obs = data_loader.get_obstacles_near((minx, miny, maxx, maxy), buffer_deg=0.05)
    features = []
    for o in obs[:150]:
        features.append({
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [o["centroid"][0], o["centroid"][1], o["height_m"]]
            },
            "properties": {
                "name": o["name"],
                "height_m": o["height_m"],
                "safety_radius_m": 70.0
            }
        })
    return {
        "type": "FeatureCollection",
        "features": features
    }

@app.post("/api/preview-photogrammetry")
def preview_photogrammetry(sensor_id: str = "sony_rx1r2", target_gsd_cm: float = 3.0):
    sensor = SENSOR_CATALOG.get(sensor_id)
    if not sensor:
        raise HTTPException(status_code=400, detail="Unknown sensor")
    return calculate_photogrammetry(sensor, target_gsd_cm)

@app.post("/api/plan-mission")
def plan_mission(req: MissionRequest):
    """
    Основной метод планирования полетного задания для группы БВС Геоскан.
    Поддерживает как одиночные сенсоры, так и комбинированную съемку (RGB + LiDAR / Геофизика).
    """
    drones = [GEOSCAN_DRONE_CATALOG[d_id] for d_id in req.available_drones if d_id in GEOSCAN_DRONE_CATALOG]
    if not drones:
        drones = list(GEOSCAN_DRONE_CATALOG.values())
        
    sensors_to_plan = []
    if req.sensor_ids and len(req.sensor_ids) > 0:
        for s_id in req.sensor_ids:
            if s_id in SENSOR_CATALOG:
                sensors_to_plan.append(SENSOR_CATALOG[s_id])
    if not sensors_to_plan:
        sensors_to_plan = [SENSOR_CATALOG.get(req.sensor_id, SENSOR_CATALOG["sony_rx1r2"])]

    geom_data = req.polygon_geojson.get("geometry", req.polygon_geojson)
    coords = geom_data.get("coordinates", [])
    nearby_obstacles = []
    if coords and len(coords) > 0:
        flat_coords = coords[0] if geom_data.get("type") == "Polygon" else coords[0][0]
        lons = [c[0] for c in flat_coords]
        lats = [c[1] for c in flat_coords]
        if lons and lats:
            minx, maxx = min(lons), max(lons)
            miny, maxy = min(lats), max(lats)
            nearby_obstacles = data_loader.get_obstacles_near((minx, miny, maxx, maxy), buffer_deg=0.03)

    all_drone_plans = []
    all_rejections = {}
    height_warnings = []
    all_obstacle_analysis = []
    feasibility_report = None
    total_area_ha = 0.0
    total_swaths = 0
    total_fleet_dist_km = 0.0
    total_fleet_time_min = 0.0
    max_makespan_min = 0.0
    photogrammetry_list = []
    wind_cond = None

    for sensor in sensors_to_plan:
        sub_res = plan_multi_uav_mission(
            polygon_geojson=req.polygon_geojson,
            sensor=sensor,
            drones=drones,
            wind=req.wind,
            criterion=req.optimization_criterion,
            target_gsd_cm=req.target_gsd_cm,
            launch_points=req.launch_points,
            obstacles=nearby_obstacles,
            sweep_angle_deg=req.sweep_angle_deg,
            max_allowed_time_min=req.max_allowed_time_min,
            max_available_drones=req.max_available_drones,
            battery_swap_penalty_min=req.battery_swap_penalty_min
        )
        if "error" in sub_res:
            continue
            
        all_drone_plans.extend(sub_res.get("drone_plans", []))
        if sub_res.get("fleet_rejections"):
            all_rejections.update(sub_res["fleet_rejections"])
        if sub_res.get("height_warning"):
            height_warnings.append(sub_res["height_warning"])
        if sub_res.get("obstacle_analysis"):
            all_obstacle_analysis.extend(sub_res["obstacle_analysis"])
        if sub_res.get("feasibility"):
            feasibility_report = sub_res["feasibility"]
            
        photogrammetry_list.append(sub_res.get("photogrammetry", {}))
        m = sub_res.get("metrics", {})
        total_area_ha = m.get("total_survey_area_ha", 0.0)
        total_swaths += m.get("total_swaths", 0)
        total_fleet_dist_km += m.get("total_fleet_distance_km", 0.0)
        total_fleet_time_min += m.get("total_fleet_time_min", 0.0)
        if m.get("makespan_min", 0.0) > max_makespan_min:
            max_makespan_min = m.get("makespan_min", 0.0)
        wind_cond = sub_res.get("wind_conditions")

    if not all_drone_plans:
        raise HTTPException(status_code=400, detail="Не удалось построить полетное задание для указанного участка")

    return {
        "status": "success",
        "optimization_criterion": req.optimization_criterion,
        "photogrammetry": photogrammetry_list[0] if len(photogrammetry_list) == 1 else photogrammetry_list,
        "height_warning": " | ".join(set(height_warnings)) if height_warnings else None,
        "fleet_rejections": all_rejections,
        "obstacle_analysis": all_obstacle_analysis,
        "feasibility": feasibility_report,
        "wind_conditions": wind_cond,
        "metrics": {
            "total_survey_area_ha": total_area_ha,
            "total_swaths": total_swaths,
            "makespan_min": round(max_makespan_min, 1),
            "total_fleet_distance_km": round(total_fleet_dist_km, 2),
            "total_fleet_time_min": round(total_fleet_time_min, 1),
            "active_drones_count": len(all_drone_plans)
        },
        "drone_plans": all_drone_plans
    }

@app.post("/api/export/kml")
def export_kml(drone_plan: Dict[str, Any]):
    kml_str = export_plan_to_kml(drone_plan)
    return Response(content=kml_str, media_type="application/vnd.google-earth.kml+xml")

@app.post("/api/export/geojson")
def export_geojson(drone_plan: Dict[str, Any]):
    gj = export_plan_to_geojson(drone_plan)
    return Response(content=json.dumps(gj, ensure_ascii=False, indent=2), media_type="application/geo+json")

@app.post("/api/export/qgc")
def export_qgc(drone_plan: Dict[str, Any]):
    qgc = export_plan_to_qgc_mission(drone_plan)
    return Response(content=json.dumps(qgc, ensure_ascii=False, indent=2), media_type="application/json")

# Монтирование статических файлов веб-приложения (frontend)
FRONTEND_DIST = os.path.join(os.path.dirname(__file__), "..", "frontend")
if os.path.exists(FRONTEND_DIST):
    app.mount("/", StaticFiles(directory=FRONTEND_DIST, html=True), name="frontend")
