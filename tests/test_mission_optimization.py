import unittest
import math
import io
import json
import zipfile
import xml.etree.ElementTree as ET
from itertools import permutations, product

from fastapi.testclient import TestClient
from shapely.geometry import LineString, shape

from backend.main import app
from backend.path_planner import (
    get_utm_transformer,
    optimize_swath_tour_for_base,
    score_full_tour,
)
from backend.wind_math import solve_wind_triangle


class MissionOptimizationTests(unittest.TestCase):
    def test_small_swath_tour_matches_exhaustive_search(self):
        swaths = [
            [(0.0, 0.0), (100.0, 0.0)],
            [(20.0, 50.0), (140.0, 50.0)],
            [(10.0, 120.0), (110.0, 120.0)],
            [(80.0, 190.0), (200.0, 190.0)],
        ]
        base = (-40.0, -30.0)
        for drone_type in ("fixed_wing", "multirotor"):
            actual = optimize_swath_tour_for_base(swaths, base, drone_type, 85.0)
            actual_cost = score_full_tour(actual, base, drone_type, 85.0)
            brute_cost = min(
                score_full_tour([
                    (swaths[i][0], swaths[i][-1]) if direction == 0
                    else (swaths[i][-1], swaths[i][0])
                    for i, direction in zip(order, directions)
                ], base, drone_type, 85.0)
                for order in permutations(range(len(swaths)))
                for directions in product((0, 1), repeat=len(swaths))
            )
            self.assertAlmostEqual(actual_cost, brute_cost, places=7)

    @classmethod
    def setUpClass(cls):
        cls.client = TestClient(app, raise_server_exceptions=False)
        cls.client.__enter__()

    @classmethod
    def tearDownClass(cls):
        cls.client.__exit__(None, None, None)

    def plan(self, fid, criterion="min_makespan"):
        parcel = self.client.get(f"/api/parcels/{fid}")
        self.assertEqual(parcel.status_code, 200)
        return self.client.post("/api/plan-mission", json={
            "polygon_geojson": parcel.json(),
            "available_drones": ["geoscan_201", "geoscan_gemini"],
            "optimization_criterion": criterion,
            "target_gsd_cm": 3.0,
        })

    def test_objectives_compare_complete_routes(self):
        for fid in (2190, 1843):
            fastest = self.plan(fid).json()
            least_flight = self.plan(fid, "min_flight_time").json()
            self.assertLessEqual(
                fastest["metrics"]["makespan_min"],
                least_flight["metrics"]["makespan_min"] + 0.11,
            )
            self.assertLessEqual(
                least_flight["metrics"]["total_fleet_time_min"],
                fastest["metrics"]["total_fleet_time_min"] + 0.11,
            )
            self.assertGreaterEqual(fastest["metrics"]["coverage_pct"], 99.0)
            self.assertFalse(fastest["search"]["optimality_proven"])

    def test_small_parcel_has_no_server_crash(self):
        self.assertNotEqual(self.plan(278).status_code, 500)

    def test_unsafe_obstacle_is_rejected(self):
        response = self.plan(2248)
        self.assertEqual(response.status_code, 422)
        self.assertIn("70", response.json()["detail"])

    def test_reported_distance_matches_exported_route(self):
        result = self.plan(1843, "min_flight_time").json()
        for plan in result["drone_plans"]:
            coords = plan["geojson_linestring"]["coordinates"]
            to_utm, _ = get_utm_transformer(coords[0][0], coords[0][1])
            distance_km = LineString([to_utm(x, y) for x, y, _ in coords]).length / 1000.0
            self.assertAlmostEqual(plan["distance_km"], distance_km, delta=0.03)
            self.assertEqual(len(plan["waypoint_times_s"]), len(coords))
            self.assertEqual(plan["waypoint_times_s"][0], 0.0)
            self.assertTrue(all(a <= b for a, b in zip(
                plan["waypoint_times_s"], plan["waypoint_times_s"][1:]
            )))
            self.assertAlmostEqual(plan["waypoint_times_s"][-1], plan["flight_time_s"], delta=0.1)

    def test_parcel_index_returns_matching_feature_and_area(self):
        listed = self.client.get("/api/parcels?limit=1000").json()
        self.assertEqual(listed["total"], len(listed["parcels"]))
        sample = listed["parcels"][100]
        feature = self.client.get(f"/api/parcels/by-index/{sample['index']}").json()
        self.assertEqual(feature["properties"]["fid"], sample["fid"])
        self.assertEqual(feature["properties"]["area_m2"], sample["area_m2"])
        self.assertEqual(self.client.get("/api/parcels/by-index/99999").status_code, 404)

    def test_airspace_endpoint_returns_valid_geojson_geometry(self):
        collection = self.client.get("/api/airspace-zones").json()
        self.assertEqual(collection["type"], "FeatureCollection")
        self.assertTrue(collection["features"])
        for feature in collection["features"]:
            self.assertTrue(shape(feature["geometry"]).is_valid)

    def test_allowed_area_rejects_route_outside_boundary(self):
        parcel = self.client.get("/api/parcels/1843").json()
        outside = {"type": "Polygon", "coordinates": [[
            [37.0, 55.0], [37.001, 55.0], [37.001, 55.001],
            [37.0, 55.001], [37.0, 55.0],
        ]]}
        response = self.client.post("/api/plan-mission", json={
            "polygon_geojson": parcel,
            "available_drones": ["geoscan_201", "geoscan_gemini"],
            "allowed_airspace_geojson": outside,
        })
        self.assertEqual(response.status_code, 422)
        self.assertIn("разрешенной области", response.json()["detail"])

    def test_headwind_can_make_a_leg_infeasible(self):
        solution = solve_wind_triangle(0, 10, 15, 0)
        self.assertFalse(solution["feasible"])
        self.assertEqual(solution["ground_speed_ms"], 0.0)

    def test_group_export_contains_every_drone_plan(self):
        result = self.plan(1843).json()
        plans = result["drone_plans"]
        response = self.client.post("/api/export/kml", json=plans)
        self.assertEqual(response.status_code, 200)
        with zipfile.ZipFile(io.BytesIO(response.content)) as archive:
            self.assertEqual(len(archive.namelist()), len(plans))
            for name in archive.namelist():
                ET.fromstring(archive.read(name))

    def test_single_plan_exports_parse(self):
        plan = self.plan(2190, "min_flight_time").json()["drone_plans"][0]
        kml = self.client.post("/api/export/kml", json=plan)
        self.assertEqual(kml.status_code, 200)
        ET.fromstring(kml.content)
        geojson = self.client.post("/api/export/geojson", json=plan)
        self.assertEqual(geojson.status_code, 200)
        self.assertTrue(json.loads(geojson.content)["features"])
        qgc = self.client.post("/api/export/qgc", json=plan)
        self.assertEqual(qgc.status_code, 200)
        self.assertTrue(json.loads(qgc.content)["mission"]["items"])

    def test_search_compares_available_bases(self):
        polygon = {"type": "Polygon", "coordinates": [[
            [37.0, 55.0], [37.006, 55.0], [37.006, 55.006],
            [37.0, 55.006], [37.0, 55.0],
        ]]}
        response = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "available_drones": ["geoscan_201"],
            "launch_points": [
                {"id": "far", "name": "far", "lat": 55.0, "lon": 37.1},
                {"id": "near", "name": "near", "lat": 55.0, "lon": 37.0},
            ],
            "wind": {"speed_ms": 0, "direction_deg": 0},
            "optimization_criterion": "min_flight_time",
            "avoid_nfz": False,
        })
        self.assertEqual(response.status_code, 200)
        start = response.json()["drone_plans"][0]["waypoints"][0]
        self.assertAlmostEqual(start["lon"], 37.0)

    def test_time_limit_is_hard_constraint(self):
        parcel = self.client.get("/api/parcels/1843").json()
        response = self.client.post("/api/plan-mission", json={
            "polygon_geojson": parcel,
            "available_drones": ["geoscan_201", "geoscan_gemini"],
            "max_allowed_time_min": 1.0,
        })
        self.assertEqual(response.status_code, 422)

    def test_same_drone_sensor_jobs_are_sequential(self):
        polygon = {"type": "Polygon", "coordinates": [[
            [37.0, 55.0], [37.006, 55.0], [37.006, 55.006],
            [37.0, 55.006], [37.0, 55.0],
        ]]}
        response = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "sensor_ids": ["sony_rx1r2", "multispectral"],
            "available_drones": ["geoscan_gemini"],
            "avoid_nfz": False,
        })
        self.assertEqual(response.status_code, 200)
        result = response.json()
        plans = result["drone_plans"]
        self.assertEqual(result["metrics"]["active_drones_count"], 1)
        self.assertGreaterEqual(plans[1]["start_time_s"], plans[0]["flight_time_s"])
        self.assertGreater(result["metrics"]["makespan_min"], plans[1]["flight_time_min"])
        limited = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "sensor_ids": ["sony_rx1r2", "multispectral"],
            "available_drones": ["geoscan_gemini"],
            "avoid_nfz": False,
            "max_allowed_time_min": result["metrics"]["makespan_min"] - 1.0,
        })
        self.assertEqual(limited.status_code, 422)

    def test_battery_level_and_emergency_diversion(self):
        polygon = {"type": "Polygon", "coordinates": [[
            [37.0, 55.0], [37.006, 55.0], [37.006, 55.006],
            [37.0, 55.006], [37.0, 55.0],
        ]]}
        response = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "available_drones": ["geoscan_201"],
            "drone_battery_levels": {"geoscan_201": 70.0},
            "launch_points": [
                {"id": "base_main", "name": "ВПП Основная", "lat": 55.0, "lon": 37.0, "type": "base"},
                {"id": "pad_emerg", "name": "Площадка Резерв", "lat": 55.005, "lon": 37.007, "type": "emergency_pad"},
            ],
            "avoid_nfz": False,
        })
        self.assertEqual(response.status_code, 200)
        data = response.json()
        plan = data["drone_plans"][0]
        # Проверяем, что начальный заряд отразился в остатке батареи
        self.assertLessEqual(plan["battery_remaining_pct"], 70.0)
        # Проверяем наличие расчета аварийного схода на резервную площадку
        emerg = plan.get("emergency_diversion")
        self.assertIsNotNone(emerg)
        self.assertEqual(emerg["pad_id"], "pad_emerg")
        self.assertGreater(emerg["distance_km"], 0.0)
        self.assertIn("landing_procedure", emerg)
        self.assertIn("psr_info", emerg)
        self.assertGreaterEqual(len(emerg["waypoints"]), 3)

    def test_low_battery_drone_utilization_in_makespan(self):
        """Проверяем, что при 40% батареи дрон не отбраковывается, а задействуется с меньшей долей галсов."""
        polygon = {
            "type": "Polygon",
            "coordinates": [[[38.65, 54.85], [38.67, 54.85], [38.67, 54.86], [38.65, 54.86], [38.65, 54.85]]]
        }
        res = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "sensor_id": "sony_rx1r2",
            "available_drones": ["geoscan_201", "geoscan_gemini"],
            "drone_battery_levels": {"geoscan_gemini": 40.0, "geoscan_201": 100.0},
            "optimization_criterion": "min_makespan",
            "avoid_nfz": False,
            "avoid_obstacles": False
        })
        self.assertEqual(res.status_code, 200)
        data = res.json()
        # Оба дрона должны быть подняты в воздух (параллельный флот)
        self.assertEqual(data["metrics"]["active_drones_count"], 2)
        plans_by_drone = {p["drone_id"]: p for p in data["drone_plans"]}
        self.assertIn("geoscan_gemini", plans_by_drone)
        self.assertIn("geoscan_201", plans_by_drone)
        # Gemini с 40% получает меньше галсов, чем Geoscan 201 с 100%
        self.assertLess(plans_by_drone["geoscan_gemini"]["swaths_count"], plans_by_drone["geoscan_201"]["swaths_count"])

    def test_qgc_export_params_has_no_null(self):
        plan = self.plan(2190, "min_flight_time").json()["drone_plans"][0]
        qgc_res = self.client.post("/api/export/qgc", json=plan)
        self.assertEqual(qgc_res.status_code, 200)
        qgc_json = json.loads(qgc_res.content)
        for item in qgc_json["mission"]["items"]:
            for p in item["params"]:
                self.assertIsNotNone(p, "QGC params array must not contain None/null values")

    def test_fixed_wing_turns_and_emergency_path_smoothness(self):
        """Проверяем отсутствие резких изломов в виражах разворота самолета и прямолинейность аварийного схода."""
        parcel = self.client.get("/api/parcels/1843").json()
        res = self.client.post("/api/plan-mission", json={
            "polygon_geojson": parcel,
            "available_drones": ["geoscan_201"],
            "optimization_criterion": "min_makespan",
            "target_gsd_cm": 3.0,
            "wind_speed_ms": 6.0,
            "wind_direction_deg": 135.0
        })
        self.assertEqual(res.status_code, 200)
        data = res.json()
        dp = data["drone_plans"][0]
        
        # 1. Проверка углов в точках поворота Дубинса
        wps = dp["waypoints"]
        coords = [(w["lon"], w["lat"]) for w in wps]
        for i in range(1, len(coords) - 1):
            if wps[i].get("stage") == "TURN_DUBINS":
                p0, p1, p2 = coords[i-1], coords[i], coords[i+1]
                v1 = (p1[0] - p0[0], p1[1] - p0[1])
                v2 = (p2[0] - p1[0], p2[1] - p1[1])
                m1, m2 = math.hypot(*v1), math.hypot(*v2)
                if m1 > 1e-6 and m2 > 1e-6:
                    dot = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (m1 * m2)))
                    angle = math.degrees(math.acos(dot))
                    self.assertLessEqual(angle, 45.0, f"Вираж самолета имеет резкий излом {angle:.1f}° в WP {i}")
                    
        # 2. Проверка аварийного схода: строгий створ и отсутствие зигзагов
        em = dp.get("emergency_diversion")
        self.assertIsNotNone(em, "План должен содержать резервный сход")
        ecoords = [(w["lon"], w["lat"]) for w in em["waypoints"]]
        for i in range(1, len(ecoords) - 1):
            p0, p1, p2 = ecoords[i-1], ecoords[i], ecoords[i+1]
            v1 = (p1[0] - p0[0], p1[1] - p0[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            m1, m2 = math.hypot(*v1), math.hypot(*v2)
            if m1 > 1e-6 and m2 > 1e-6:
                dot = max(-1.0, min(1.0, (v1[0]*v2[0] + v1[1]*v2[1]) / (m1 * m2)))
                angle = math.degrees(math.acos(dot))
                self.assertLessEqual(angle, 1.0, f"Аварийный сход должен быть прямым коридором, но угол в WP {i} = {angle:.1f}°")


    def test_high_altitude_zone_not_blocking_low_drone(self):
        """Проверяем, что зоны на высоте от 4.5 км до 30 км (UUR215) не блокируют полет БВС на 100м."""
        # Полигон на юге МО (где действует UUR215)
        polygon = {
            "type": "Polygon",
            "coordinates": [[[38.50, 54.75], [38.52, 54.75], [38.52, 54.76], [38.50, 54.76], [38.50, 54.75]]]
        }
        res = self.client.post("/api/plan-mission", json={
            "polygon_geojson": polygon,
            "available_drones": ["geoscan_gemini"],
            "target_gsd_cm": 3.0,
            "avoid_nfz": True
        })
        self.assertEqual(res.status_code, 200, f"Планирование не должно падать из-за высотных зон: {res.text}")
        data = res.json()
        self.assertIn("drone_plans", data)
        self.assertGreater(len(data["drone_plans"]), 0)


if __name__ == "__main__":
    unittest.main()


