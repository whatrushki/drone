import sys
sys.path.append(r'c:\Users\kriti\Desktop\drone')
from backend.data_loader import data_loader
from backend.models import GEOSCAN_DRONE_CATALOG, SENSOR_CATALOG, WindConfig
from backend.path_planner import plan_multi_uav_mission

data_loader.load_all()
p0 = data_loader.survey_polygons[0]
poly_geojson = {
    'type': 'Polygon',
    'coordinates': [list(p0['geometry'].exterior.coords)]
}
sensor = SENSOR_CATALOG['sony_rx1r2']
drones = [GEOSCAN_DRONE_CATALOG['geoscan_201'], GEOSCAN_DRONE_CATALOG['geoscan_gemini']]
wind = WindConfig(speed_ms=6.0, direction_deg=120.0)

print("=== TEST 1: MIN_MAKESPAN (Минимум времени) ===")
res1 = plan_multi_uav_mission(poly_geojson, sensor, drones, wind, criterion="min_makespan", target_gsd_cm=3.0)
print("Area:", res1["metrics"]["total_survey_area_ha"], "ha")
print("Total swaths:", res1["metrics"]["total_swaths"])
print("Makespan:", res1["metrics"]["makespan_min"], "min")
print("Total distance:", res1["metrics"]["total_fleet_distance_km"], "km")
for dp in res1["drone_plans"]:
    print(f"  -> {dp['drone_name']}: {dp['swaths_count']} swaths, {dp['distance_km']} km, {dp['flight_time_min']} min, bat left: {dp['battery_remaining_pct']}%, safe: {dp['is_energy_safe']}")

print("\n=== TEST 2: MIN_FLIGHT_TIME (Минимум налета/ресурса) ===")
res2 = plan_multi_uav_mission(poly_geojson, sensor, drones, wind, criterion="min_flight_time", target_gsd_cm=3.0)
print("Total swaths:", res2["metrics"]["total_swaths"])
print("Makespan:", res2["metrics"]["makespan_min"], "min")
print("Total distance:", res2["metrics"]["total_fleet_distance_km"], "km")
for dp in res2["drone_plans"]:
    print(f"  -> {dp['drone_name']}: {dp['swaths_count']} swaths, {dp['distance_km']} km, {dp['flight_time_min']} min, bat left: {dp['battery_remaining_pct']}%")
