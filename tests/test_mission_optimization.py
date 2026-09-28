import unittest
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


if __name__ == "__main__":
    unittest.main()
