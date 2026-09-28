import sys
sys.path.append(r'c:\Users\kriti\Desktop\drone')
from backend.data_loader import data_loader
from backend.models import GEOSCAN_DRONE_CATALOG, SENSOR_CATALOG, WindConfig, LaunchPoint
from backend.path_planner import plan_multi_uav_mission, get_utm_transformer
import numpy as np
import math

data_loader.load_all()
p0 = data_loader.survey_polygons[0]
poly_geojson = {'type': 'Polygon', 'coordinates': [list(p0['geometry'].exterior.coords)]}
sensor = SENSOR_CATALOG['sony_rx1r2']
drones = [GEOSCAN_DRONE_CATALOG['geoscan_201']]
wind = WindConfig(speed_ms=6.0, direction_deg=120.0)

for ang in [62.2, 0.0, 90.0, 140.0]:
    print(f"\n================ TESTING ANGLE {ang} ================")
    res = plan_multi_uav_mission(poly_geojson, sensor, drones, wind, criterion='min_flight_time', sweep_angle_deg=ang)
    plan = res['drone_plans'][0]
    wps = plan['waypoints']
    to_utm, to_wgs = get_utm_transformer(wps[0]['lon'], wps[0]['lat'])
    pts = [to_utm(w['lon'], w['lat']) for w in wps]

    found = 0
    for i in range(len(pts) - 1):
        pA = np.array(pts[i])
        pB = np.array(pts[i+1])
        dAB = pB - pA
        lAB = np.linalg.norm(dAB)
        if lAB < 1.0: continue
        uAB = dAB / lAB
        
        for j in range(i + 1, min(len(pts) - 1, i + 35)):
            pC = np.array(pts[j])
            pD = np.array(pts[j+1])
            dCD = pD - pC
            lCD = np.linalg.norm(dCD)
            if lCD < 1.0: continue
            uCD = dCD / lCD
            dot = np.dot(uAB, uCD)
            if dot < -0.85:
                proj = np.dot(pC - pA, uAB)
                if -10.0 <= proj <= lAB + 10.0:
                    dist_line = np.linalg.norm((pC - pA) - proj * uAB)
                    if dist_line < 15.0:
                        found += 1
                        print(f"BACKTRACK #{found} at angle {ang}:")
                        print(f"   Seg {i}->{i+1}: {wps[i].get('stage')} ({wps[i].get('action')}) -> {wps[i+1].get('stage')} ({wps[i+1].get('action')}), len={lAB:.1f}m")
                        print(f"   Seg {j}->{j+1}: {wps[j].get('stage')} ({wps[j].get('action')}) -> {wps[j+1].get('stage')} ({wps[j+1].get('action')}), len={lCD:.1f}m")
                        print(f"   Distance between lines: {dist_line:.1f}m, dot={dot:.3f}")
                        break
            if found >= 5:
                break
    if found == 0:
        print("No backtracks found!")
