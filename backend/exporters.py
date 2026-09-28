import json
from typing import Dict, Any, List
import xml.etree.ElementTree as ET
from xml.sax.saxutils import escape

def export_plan_to_geojson(drone_plan: Dict[str, Any]) -> Dict[str, Any]:
    """Экспорт индивидуального полетного задания БВС в стандартный GeoJSON"""
    features = []
    
    # 1. Линия маршрута (LineString)
    route_coords = drone_plan["geojson_linestring"]["coordinates"]
    features.append({
        "type": "Feature",
        "geometry": {
            "type": "LineString",
            "coordinates": route_coords
        },
        "properties": {
            "name": f"Траектория полета {drone_plan['drone_name']}",
            "drone_id": drone_plan["drone_id"],
            "drone_type": drone_plan["drone_type"],
            "distance_km": drone_plan["distance_km"],
            "flight_time_min": drone_plan["flight_time_min"],
            "flight_height_m": drone_plan["flight_height_m"],
            "cruise_speed_ms": drone_plan["cruise_speed_ms"],
            "battery_used_pct": drone_plan["battery_used_pct"],
            "stroke": "#00b4d8" if drone_plan["drone_type"] == "fixed_wing" else "#ffb703",
            "stroke-width": 3
        }
    })
    
    # 2. Ключевые точки (Waypoints)
    for idx, wp in enumerate(drone_plan["waypoints"]):
        features.append({
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [wp["lon"], wp["lat"], wp["alt_m"]]
            },
            "properties": {
                "index": idx + 1,
                "stage": wp["stage"],
                "action": wp["action"],
                "alt_m": wp["alt_m"],
                "speed_ms": wp["speed_ms"],
                "swath_idx": wp.get("swath_idx"),
                "trigger_dist_m": wp.get("trigger_dist_m")
            }
        })
        
    return {
        "type": "FeatureCollection",
        "metadata": {
            "drone_name": drone_plan["drone_name"],
            "total_distance_km": drone_plan["distance_km"],
            "total_time_min": drone_plan["flight_time_min"],
            "waypoints_count": len(drone_plan["waypoints"])
        },
        "features": features
    }

def export_plan_to_kml(drone_plan: Dict[str, Any]) -> str:
    """Экспорт индивидуального полетного задания БВС в валидный KML (Google Earth / Geoscan)"""
    drone_name = escape(str(drone_plan["drone_name"]))
    line_color = "7fff0000" if drone_plan["drone_type"] == "fixed_wing" else "7f00ffff" # ABGR format
    
    kml = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        f'    <name>Полетное задание: {drone_name}</name>',
        '    <description>Сформировано в сервисе Geoscan FleetCommander</description>',
        '    <Style id="routeStyle">',
        '      <LineStyle>',
        f'        <color>{line_color}</color>',
        '        <width>4</width>',
        '      </LineStyle>',
        '    </Style>',
        '    <Style id="takeoffStyle">',
        '      <IconStyle><color>ff00ff00</color><scale>1.2</scale></IconStyle>',
        '    </Style>',
        '    <Style id="landingStyle">',
        '      <IconStyle><color>ff0000ff</color><scale>1.2</scale></IconStyle>',
        '    </Style>',
        '    <Folder>',
        '      <name>Траектория</name>',
        '      <Placemark>',
        f'        <name>{drone_name} - Полный маршрут</name>',
        '        <styleUrl>#routeStyle</styleUrl>',
        '        <LineString>',
        '          <extrude>1</extrude>',
        '          <tessellate>1</tessellate>',
        '          <altitudeMode>relativeToGround</altitudeMode>',
        '          <coordinates>'
    ]
    
    # Координаты линии
    coord_tokens = []
    for c in drone_plan["geojson_linestring"]["coordinates"]:
        alt = c[2] if len(c) > 2 else drone_plan["flight_height_m"]
        coord_tokens.append(f"{c[0]},{c[1]},{alt}")
    kml.append("            " + " ".join(coord_tokens))
    
    kml.extend([
        '          </coordinates>',
        '        </LineString>',
        '      </Placemark>',
        '    </Folder>',
        '    <Folder>',
        '      <name>Ключевые точки (Waypoints)</name>'
    ])
    
    for idx, wp in enumerate(drone_plan["waypoints"]):
        stage = escape(str(wp["stage"]))
        action = escape(str(wp["action"]))
        kml.extend([
            '      <Placemark>',
            f'        <name>WP {idx+1}: {stage}</name>',
            f'        <description>Действие: {action}&#10;Высота: {wp["alt_m"]} м&#10;Скорость: {wp["speed_ms"]} м/с</description>',
            '        <Point>',
            '          <altitudeMode>relativeToGround</altitudeMode>',
            f'          <coordinates>{wp["lon"]},{wp["lat"]},{wp["alt_m"]}</coordinates>',
            '        </Point>',
            '      </Placemark>'
        ])
        
    kml.extend([
        '    </Folder>',
        '  </Document>',
        '</kml>'
    ])
    
    return "\n".join(kml)

def export_plan_to_qgc_mission(drone_plan: Dict[str, Any]) -> Dict[str, Any]:
    """Экспорт в формат QGroundControl / Mission Planner (.plan)"""
    items = []
    for idx, wp in enumerate(drone_plan["waypoints"]):
        command = 16 # MAV_CMD_NAV_WAYPOINT
        if wp["stage"] == "TAKEOFF":
            command = 22 # MAV_CMD_NAV_TAKEOFF
        elif wp["stage"] == "LANDING":
            command = 21 # MAV_CMD_NAV_LAND
            
        items.append({
            "autoContinue": True,
            "command": command,
            "doJumpId": idx + 1,
            "frame": 3, # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [0.0, 0.0, 0.0, 0.0, wp["lat"], wp["lon"], wp["alt_m"]],
            "type": "SimpleItem"
        })
        
    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl / Geoscan Planner",
        "mission": {
            "cruiseSpeed": drone_plan["cruise_speed_ms"],
            "hoverSpeed": drone_plan["cruise_speed_ms"],
            "items": items,
            "plannedHomePosition": [
                drone_plan["waypoints"][0]["lat"],
                drone_plan["waypoints"][0]["lon"],
                0
            ],
            "vehicleType": 1 if drone_plan["drone_type"] == "fixed_wing" else 2,
            "version": 2
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1
    }

def export_emergency_to_qgc_mission(drone_plan: Dict[str, Any]) -> Dict[str, Any]:
    """Экспорт аварийного плана схода на резервную ВПП в формат QGroundControl (.plan)"""
    emerg = drone_plan.get("emergency_diversion") or {}
    waypoints = emerg.get("waypoints") or []
    if not waypoints:
        return export_plan_to_qgc_mission(drone_plan)
        
    items = []
    for idx, wp in enumerate(waypoints):
        command = 16 # MAV_CMD_NAV_WAYPOINT
        if wp.get("stage") == "LANDING":
            command = 21 # MAV_CMD_NAV_LAND
        items.append({
            "autoContinue": True,
            "command": command,
            "doJumpId": idx + 1,
            "frame": 3,
            "params": [0.0, 0.0, 0.0, 0.0, wp["lat"], wp["lon"], wp["alt_m"]],
            "type": "SimpleItem"
        })
        
    return {
        "fileType": "Plan",
        "geoFence": {"circles": [], "polygons": [], "version": 2},
        "groundStation": "QGroundControl / Geoscan Emergency Contingency",
        "mission": {
            "cruiseSpeed": drone_plan.get("cruise_speed_ms", 15.0),
            "hoverSpeed": drone_plan.get("cruise_speed_ms", 15.0),
            "items": items,
            "plannedHomePosition": [
                emerg.get("lat", waypoints[0]["lat"]),
                emerg.get("lon", waypoints[0]["lon"]),
                0
            ],
            "vehicleType": 1 if drone_plan.get("drone_type") == "fixed_wing" else 2,
            "version": 2
        },
        "rallyPoints": {"points": [], "version": 2},
        "version": 1
    }

