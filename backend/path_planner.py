import math
import time
from itertools import combinations
import numpy as np
from typing import List, Dict, Any, Tuple, Optional
from shapely.geometry import Polygon, MultiPolygon, LineString, Point, box, shape
from shapely.affinity import rotate, translate
from shapely.ops import transform, unary_union
import pyproj
from .models import DroneSpec, SensorSpec, LaunchPoint, WindConfig, GEOSCAN_DRONE_CATALOG, SENSOR_CATALOG
from .photogrammetry import calculate_photogrammetry
from .wind_math import solve_wind_triangle, calculate_flight_leg, calculate_flight_leg_utm

def get_utm_transformer(lon: float, lat: float):
    """Возвращает проекции WGS84 <-> UTM для точных метрических расчетов"""
    utm_zone = int((lon + 180) / 6) + 1
    epsg_code = 32600 + utm_zone if lat >= 0 else 32700 + utm_zone
    to_utm = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:{epsg_code}", always_xy=True).transform
    to_wgs = pyproj.Transformer.from_crs(f"EPSG:{epsg_code}", "EPSG:4326", always_xy=True).transform
    return to_utm, to_wgs

def append_unique_pt(pts_list: list, new_pt: Tuple[float, float], min_dist: float = 2.0):
    """Добавляет точку в список, исключая дубликаты и микро-скачки (< min_dist метров)"""
    if not pts_list:
        pts_list.append((float(new_pt[0]), float(new_pt[1])))
    else:
        last = pts_list[-1]
        if math.hypot(new_pt[0] - last[0], new_pt[1] - last[1]) >= min_dist:
            pts_list.append((float(new_pt[0]), float(new_pt[1])))

def solve_csc(
    c0: np.ndarray, s0: int,
    c1: np.ndarray, s1: int,
    R: float
) -> Optional[Tuple[np.ndarray, np.ndarray, float, np.ndarray]]:
    """
    Аналитическое решение касательной между двумя окружностями радиуса R (Дубинс CSC).
    s0, s1: +1 (левый вираж / CCW), -1 (правый вираж / CW).
    Гарантирует движение строго вперед (L_seg >= 0, никаких обратных векторов и движений вспять).
    """
    V = c1 - c0
    D = float(np.linalg.norm(V))
    delta_s = s0 - s1
    val = D**2 - (R * delta_s)**2
    if val < 0.0:
        return None
    L_seg = math.sqrt(val)
    theta = math.atan2(V[1], V[0])
    alpha = math.atan2(R * delta_s, L_seg)
    psi = theta + alpha
    U = np.array([math.cos(psi), math.sin(psi)])
    norm_vec = np.array([U[1], -U[0]])
    T0 = c0 + R * s0 * norm_vec
    T1 = c1 + R * s1 * norm_vec
    return T0, T1, L_seg, U

def arc_len_dubins(c: np.ndarray, p_from: np.ndarray, p_to: np.ndarray, s: int, R: float) -> float:
    """Длина дуги на окружности радиуса R от p_from до p_to в направлении s (+1=CCW, -1=CW)"""
    v1 = p_from - c
    v2 = p_to - c
    a1 = math.atan2(v1[1], v1[0])
    a2 = math.atan2(v2[1], v2[0])
    diff = (a2 - a1) if s > 0 else (a1 - a2)
    return (diff % (2.0 * math.pi)) * R

def generate_dubins_turn(
    p_exit: Tuple[float, float],     # Точка выхода с предыдущего галса (UTM x, y)
    heading_out: float,              # Направление выхода (градусы)
    p_enter: Tuple[float, float],    # Точка входа на следующий галс (UTM x, y)
    heading_in: float,               # Направление входа (градусы)
    turn_radius_m: float = 85.0,     # Минимальный радиус виража (Геоскан 201)
    lead_out_m: float = 25.0
) -> List[Tuple[float, float]]:
    """
    Генерирует строгую непрерывную (G1) кинематически выполнимую траекторию разворота
    для БВС самолетного типа (Геоскан 201). На всех дугах радиус кривизны строго >= R_min (85м).
    
    1. Самолет сходит с галса по касательной через стабилизирующий выбег (lead_out).
    2. Если следующий галс параллелен и идет строго навстречу (dot < -0.9, D < 2R):
       выполняется классический авиационный разворот 'рыбий хвост' (Bulb turn)
       с точным сопряжением двух дуг радиуса R и выходом на створ следующего галса.
    3. Для всех остальных конфигураций (D >= 2R, галсы под углом или сонаправленные):
       рассчитывается кратчайшая прямая траектория Дубинса (LSL, RSR, LSR, RSL) с виражами радиуса R.
    4. Заход на следующий галс выполняется строго по оси галса (рассогласование 0.00°).
       Полностью исключены движения вспять, изломы и петли 'вперед-назад'.
    """
    R = float(turn_radius_m)
    rad_out = math.radians(heading_out)
    d_out = np.array([math.sin(rad_out), math.cos(rad_out)])
    rad_in = math.radians(heading_in)
    d_in = np.array([math.sin(rad_in), math.cos(rad_in)])
    
    # P0: выбег с предыдущего галса (стабилизация после окончания триггера)
    P0 = np.array(p_exit, dtype=float) + d_out * lead_out_m
    # P1: точка створа перед входом на следующий галс
    P1 = np.array(p_enter, dtype=float) - d_in * max(20.0, lead_out_m)
    
    dot_dir = float(np.dot(d_out, d_in))
    n_out = np.array([d_out[1], -d_out[0]])
    
    pts = []
    append_unique_pt(pts, P0)
    
    # 1. Если следующий галс параллелен и идет навстречу (dot < -0.85)
    if dot_dir < -0.85:
        dP = P1 - P0
        along = float(np.dot(dP, d_out))
        lat_proj = float(np.dot(dP, n_out))
        n_side = n_out if lat_proj >= 0.0 else -n_out
        D = abs(lat_proj)
        
        # Выравниваем уровень виража по наиболее удаленной точке вперед по d_out,
        # чтобы разворот происходил строго снаружи полигона перед створом входа
        if along > 0.0:
            P0_turn = P0 + d_out * along
            append_unique_pt(pts, (float(P0_turn[0]), float(P0_turn[1])))
        else:
            P0_turn = P0
            
        delta_y = math.sqrt(max(0.0, 4.0 * R**2 - D**2)) if D < 2.0 * R else 0.0
        
        if D < 2.0 * R:
            # Классический авиационный вираж "Рыбий хвост" (Bulb Turn / Teardrop):
            # Дуга 1 наружу радиуса R на C1 + сопряженная дуга 2 радиуса R на C2
            C1 = P0_turn - n_side * R
            C2 = P0_turn + n_side * (D - R) + d_out * delta_y
            M = (C1 + C2) / 2.0
            
            # Дуга 1: по C1 против часовой стрелки (в базисе n_side, d_out)
            vec_M1 = M - C1
            proj_n1 = float(np.dot(vec_M1, n_side))
            proj_d1 = float(np.dot(vec_M1, d_out))
            a1 = math.atan2(proj_d1, proj_n1)
            num_pts1 = max(4, int(math.ceil(R * a1 / 18.0)))
            for a in np.linspace(0.0, a1, num_pts1)[1:]:
                pt = C1 + n_side * (R * math.cos(a)) + d_out * (R * math.sin(a))
                append_unique_pt(pts, (float(pt[0]), float(pt[1])))
                
            # Дуга 2: по C2 по часовой стрелке (в базисе n_side, d_out)
            vec_M2 = M - C2
            proj_n2 = float(np.dot(vec_M2, n_side))
            proj_d2 = float(np.dot(vec_M2, d_out))
            a_start2 = math.atan2(proj_d2, proj_n2)
            sweep_angle2 = (2.0 * math.pi + a_start2) if a_start2 < 0.0 else a_start2
            num_pts2 = max(6, int(math.ceil(R * sweep_angle2 / 18.0)))
            for a in np.linspace(a_start2, a_start2 - sweep_angle2, num_pts2)[1:]:
                pt = C2 + n_side * (R * math.cos(a)) + d_out * (R * math.sin(a))
                append_unique_pt(pts, (float(pt[0]), float(pt[1])))
                
            # Точка выхода на створ следующего галса
            p_entry_aligned = P0_turn + n_side * D + d_out * delta_y
            append_unique_pt(pts, (float(p_entry_aligned[0]), float(p_entry_aligned[1])))
            append_unique_pt(pts, P1)
            append_unique_pt(pts, p_enter)
            return pts
        else:
            # Широкий U-Turn (D >= 2R): поворот на 90°, прямая вставка (D - 2R), поворот на 90°
            C1 = P0_turn + n_side * R
            num_pts1 = max(4, int(math.ceil(R * (math.pi / 2.0) / 18.0)))
            # Дуга 1: от P0_turn до p_mid_start (поворот на 90 градусов)
            for a in np.linspace(0.0, math.pi / 2.0, num_pts1)[1:]:
                pt = C1 - n_side * (R * math.cos(a)) + d_out * (R * math.sin(a))
                append_unique_pt(pts, (float(pt[0]), float(pt[1])))
                
            # Прямой отрезок перехода к следующему створу
            p_mid_start = C1 + d_out * R
            p_mid_end = p_mid_start + n_side * (D - 2.0 * R)
            if (D - 2.0 * R) > 1.0:
                append_unique_pt(pts, (float(p_mid_end[0]), float(p_mid_end[1])))
            
            # Дуга 2: поворот еще на 90 градусов от p_mid_end до створа входа (P0_turn + n_side * D)
            C2 = P0_turn + n_side * (D - R)
            for psi in np.linspace(0.0, math.pi / 2.0, num_pts1)[1:]:
                pt = C2 + d_out * (R * math.cos(psi)) + n_side * (R * math.sin(psi))
                append_unique_pt(pts, (float(pt[0]), float(pt[1])))
                
            append_unique_pt(pts, P1)
            append_unique_pt(pts, p_enter)
            return pts

    # 2. Универсальный строгий аналитический расчет путей Дубинса (CSC) для произвольных углов
    th0 = math.radians((90.0 - heading_out) % 360.0)
    th1 = math.radians((90.0 - heading_in) % 360.0)
    n0 = np.array([-math.sin(th0), math.cos(th0)])
    n1 = np.array([-math.sin(th1), math.cos(th1)])
    
    C0_L = P0 + R * n0
    C0_R = P0 - R * n0
    C1_L = P1 + R * n1
    C1_R = P1 - R * n1
    
    candidates = [
        ('LSL', C0_L, 1, C1_L, 1),
        ('RSR', C0_R, -1, C1_R, -1),
        ('LSR', C0_L, 1, C1_R, -1),
        ('RSL', C0_R, -1, C1_L, 1),
    ]
    
    best = None
    min_len = 1e9
    for name, c0, s0, c1, s1 in candidates:
        res = solve_csc(c0, s0, c1, s1, R)
        if res:
            T0, T1, L_seg, U = res
            l1 = arc_len_dubins(c0, P0, T0, s0, R)
            l2 = arc_len_dubins(c1, T1, P1, s1, R)
            tot = l1 + L_seg + l2
            if tot < min_len:
                min_len = tot
                best = (name, c0, s0, c1, s1, T0, T1, L_seg, l1, l2)
                
    if best:
        name, c0, s0, c1, s1, T0, T1, L_seg, l1, l2 = best
        
        # Дуга 1: C0, от P0 до T0 с направлением вращения s0
        v_s = P0 - c0
        a_s = math.atan2(v_s[1], v_s[0])
        diff1 = (math.atan2(T0[1] - c0[1], T0[0] - c0[0]) - a_s) if s0 > 0 else (a_s - math.atan2(T0[1] - c0[1], T0[0] - c0[0]))
        diff1 = diff1 % (2.0 * math.pi)
        num1 = max(3, int(math.ceil(l1 / 18.0)))
        for t in np.linspace(0.0, 1.0, num1)[1:]:
            a = a_s + s0 * diff1 * t
            append_unique_pt(pts, c0 + R * np.array([math.cos(a), math.sin(a)]))
            
        # Прямолинейный отрезок по касательной: от T0 до T1
        append_unique_pt(pts, T1)
        
        # Дуга 2: C1, от T1 до P1 с направлением вращения s1
        v_s2 = T1 - c1
        a_s2 = math.atan2(v_s2[1], v_s2[0])
        diff2 = (math.atan2(P1[1] - c1[1], P1[0] - c1[0]) - a_s2) if s1 > 0 else (a_s2 - math.atan2(P1[1] - c1[1], P1[0] - c1[0]))
        diff2 = diff2 % (2.0 * math.pi)
        num2 = max(3, int(math.ceil(l2 / 18.0)))
        for t in np.linspace(0.0, 1.0, num2)[1:]:
            a = a_s2 + s1 * diff2 * t
            append_unique_pt(pts, c1 + R * np.array([math.cos(a), math.sin(a)]))
    else:
        append_unique_pt(pts, P1)
        
    append_unique_pt(pts, P1)
    append_unique_pt(pts, p_enter)
    return pts

def generate_multirotor_turn(
    p_exit: Tuple[float, float],
    p_enter: Tuple[float, float],
    radius_m: float = 6.0
) -> List[Tuple[float, float]]:
    """Плавный маневренный переход между галсами для мультироторов (Геоскан 801, Gemini)"""
    p_ex = np.array(p_exit, dtype=float)
    p_en = np.array(p_enter, dtype=float)
    mid = (p_ex + p_en) / 2.0
    vec = p_en - p_ex
    dist = np.linalg.norm(vec)
    if dist < 1.0:
        return []
    n = np.array([-vec[1], vec[0]]) / dist
    apex = mid + n * min(radius_m, dist * 0.5)
    return [(float(apex[0]), float(apex[1]))]

def plan_entry_approach(
    p_base: Tuple[float, float],
    p_start: Tuple[float, float],
    p_end: Tuple[float, float],
    R: float = 85.0,
    lead_in: float = 80.0
) -> List[Tuple[float, float]]:
    """
    Генерирует кинематически плавный (C1) выход самолета с точки взлета на створ 1-го галса.
    Самолет влетает в p_start СТРОГО ПО КАСАТЕЛЬНОЙ к направлению галса (угол 0 градусов).
    Полностью исключены острые углы и резкие виражи на входе в галс.
    """
    p_b = np.array(p_base, dtype=float)
    p_s = np.array(p_start, dtype=float)
    p_e = np.array(p_end, dtype=float)
    
    d = p_e - p_s
    norm_d = np.linalg.norm(d)
    if norm_d < 1e-3:
        return [(float(p_b[0]), float(p_b[1])), (float(p_s[0]), float(p_s[1]))]
    d = d / norm_d
    
    best_pts = None
    best_len = 1e9
    
    for is_ccw in [True, False]:
        a_target = math.atan2(-d[0], d[1]) if is_ccw else math.atan2(d[0], -d[1])
        r_vec = np.array([math.cos(a_target), math.sin(a_target)])
        
        for s_tan in np.linspace(-lead_in, -lead_in - 500.0, 25):
            p_tan = p_s + d * s_tan
            C = p_tan - R * r_vec
            delta = p_b - C
            L = np.linalg.norm(delta)
            if L <= R + 15.0:
                continue
                
            phi = math.atan2(delta[1], delta[0])
            beta = math.acos(min(1.0, max(-1.0, R / L)))
            sign = +1 if is_ccw else -1
            a_T = phi + sign * beta
            
            diff = (a_target - a_T) % (2 * math.pi) if is_ccw else (a_T - a_target) % (2 * math.pi)
            num_pts = max(4, int(math.ceil(R * diff / 20.0)))
            angles = [a_T + diff * t for t in np.linspace(0, 1, num_pts)] if is_ccw else [a_T - diff * t for t in np.linspace(0, 1, num_pts)]
            
            pts = []
            append_unique_pt(pts, p_b)
            for a in angles:
                append_unique_pt(pts, C + R * np.array([math.cos(a), math.sin(a)]))
            append_unique_pt(pts, p_tan)
            append_unique_pt(pts, p_s)
            
            total_len = math.sqrt(max(0.0, L**2 - R**2)) + R * diff + abs(s_tan)
            if total_len < best_len:
                best_len = total_len
                best_pts = pts
            
    if best_pts is not None:
        return best_pts
        
    dx = p_s[0] - p_b[0]
    dy = p_s[1] - p_b[1]
    h_base = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
    h_swath = (math.degrees(math.atan2(d[0], d[1])) + 360.0) % 360.0
    dubins_pts = generate_dubins_turn(p_base, h_base, p_start, h_swath, R)
    pts = [(float(p_b[0]), float(p_b[1]))]
    for pt in dubins_pts:
        append_unique_pt(pts, pt)
    append_unique_pt(pts, p_start)
    return pts

def plan_exit_approach(
    p_end: Tuple[float, float],
    p_prev: Tuple[float, float],
    p_base: Tuple[float, float],
    R: float = 85.0,
    lead_out: float = 40.0
) -> List[Tuple[float, float]]:
    """
    Генерирует плавный сход с последнего галса и пологий заход на посадку к ВПП.
    Радиус виража строго >= R. Никаких разворотов на месте и прыжков траектории.
    """
    p_e = np.array(p_end, dtype=float)
    p_pr = np.array(p_prev, dtype=float)
    p_b = np.array(p_base, dtype=float)
    
    d = p_e - p_pr
    norm_d = np.linalg.norm(d)
    if norm_d < 1e-3:
        return [(float(p_e[0]), float(p_e[1])), (float(p_b[0]), float(p_b[1]))]
    d = d / norm_d
    
    best_pts = None
    best_len = 1e9
    
    for is_ccw in [True, False]:
        a_start = math.atan2(-d[0], d[1]) if is_ccw else math.atan2(d[0], -d[1])
        r_vec = np.array([math.cos(a_start), math.sin(a_start)])
        
        for lo in np.linspace(lead_out, lead_out + 400.0, 20):
            p_lo = p_e + d * lo
            C = p_lo - R * r_vec
            
            delta = p_b - C
            L = np.linalg.norm(delta)
            if L <= R + 15.0:
                continue
                
            phi = math.atan2(delta[1], delta[0])
            beta = math.acos(min(1.0, max(-1.0, R / L)))
            sign = -1 if is_ccw else +1
            a_leave = phi + sign * beta
            
            diff = (a_leave - a_start) % (2 * math.pi) if is_ccw else (a_start - a_leave) % (2 * math.pi)
            num_pts = max(4, int(math.ceil(R * diff / 20.0)))
            angles = [a_start + diff * t for t in np.linspace(0, 1, num_pts)] if is_ccw else [a_start - diff * t for t in np.linspace(0, 1, num_pts)]
            
            pts = []
            append_unique_pt(pts, p_e)
            append_unique_pt(pts, p_lo)
            for a in angles:
                append_unique_pt(pts, C + R * np.array([math.cos(a), math.sin(a)]))
            append_unique_pt(pts, p_b)
            
            total_len = lo + R * diff + math.sqrt(max(0.0, L**2 - R**2))
            if total_len < best_len:
                best_len = total_len
                best_pts = pts
            
    if best_pts is not None:
        return best_pts
        
    dx = p_b[0] - p_e[0]
    dy = p_b[1] - p_e[1]
    h_to_base = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
    h_swath = (math.degrees(math.atan2(d[0], d[1])) + 360.0) % 360.0
    dubins_pts = generate_dubins_turn(p_end, h_swath, p_base, h_to_base, R)
    pts = [(float(p_e[0]), float(p_e[1]))]
    for pt in dubins_pts:
        append_unique_pt(pts, pt)
    append_unique_pt(pts, p_base)
    return pts

def calc_transition_cost(
    p_out: Tuple[float, float],
    dir_out: Tuple[float, float],
    p_in: Tuple[float, float],
    dir_in: Tuple[float, float],
    drone_type: str = "fixed_wing",
    turn_radius_m: float = 85.0
) -> float:
    """Расчет кинематической стоимости перехода между галсами с учетом типа БАС"""
    dx = p_in[0] - p_out[0]
    dy = p_in[1] - p_out[1]
    dist = math.hypot(dx, dy)
    if drone_type != "fixed_wing":
        return dist * 1.05
    R = float(turn_radius_m)
    dot = dir_out[0] * dir_in[0] + dir_out[1] * dir_in[1]
    if dist < 2.0 * R:
        turn_len = 2.0 * math.pi * R * 0.75 + dist
    else:
        turn_len = math.pi * (dist / 2.0)
    if dot > 0.3:
        turn_len += 2.0 * R
    # Жесткий штраф за перелеты через всё поле, исключающий паразитные диагонали
    if dist > 3.0 * R:
        turn_len += (dist - 3.0 * R) * 2.0
    return turn_len

def score_full_tour(
    tour: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    base_utm: Tuple[float, float],
    drone_type: str = "fixed_wing",
    turn_radius_m: float = 85.0
) -> float:
    """Оценка полной длины миссии: подлет с базы + переходы между галсами + возврат на базу"""
    if not tour:
        return 1e12
    p_start_first = tour[0][0]
    p_end_last = tour[-1][1]
    d_in = math.hypot(base_utm[0] - p_start_first[0], base_utm[1] - p_start_first[1])
    d_out = math.hypot(base_utm[0] - p_end_last[0], base_utm[1] - p_end_last[1])
    total_cost = d_in + d_out
    for k in range(len(tour) - 1):
        curr_s, curr_e = tour[k]
        next_s, next_e = tour[k + 1]
        c_dx, c_dy = curr_e[0] - curr_s[0], curr_e[1] - curr_s[1]
        n_dx, n_dy = next_e[0] - next_s[0], next_e[1] - next_s[1]
        c_len = max(1e-3, math.hypot(c_dx, c_dy))
        n_len = max(1e-3, math.hypot(n_dx, n_dy))
        c_dir = (c_dx / c_len, c_dy / c_len)
        n_dir = (n_dx / n_len, n_dy / n_len)
        total_cost += calc_transition_cost(curr_e, c_dir, next_s, n_dir, drone_type, turn_radius_m)
    return total_cost

def two_opt_tour(
    tour: List[Tuple[Tuple[float, float], Tuple[float, float]]],
    base_utm: Tuple[float, float],
    drone_type: str = "fixed_wing",
    turn_radius_m: float = 85.0,
    max_iter: int = 60
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """2-opt локальная оптимизация для устранения самопересечений и холостых петель"""
    best_tour = list(tour)
    best_score = score_full_tour(best_tour, base_utm, drone_type, turn_radius_m)
    N = len(best_tour)
    if N <= 2 or N > 60:
        return best_tour
    improved = True
    iterations = 0

    def transition(left, right):
        ls, le = left
        rs, re = right
        ld = (le[0] - ls[0], le[1] - ls[1])
        rd = (re[0] - rs[0], re[1] - rs[1])
        ll = max(1e-3, math.hypot(*ld))
        rl = max(1e-3, math.hypot(*rd))
        return calc_transition_cost(
            le, (ld[0] / ll, ld[1] / ll),
            rs, (rd[0] / rl, rd[1] / rl),
            drone_type, turn_radius_m,
        )

    def base_distance(point):
        return math.hypot(base_utm[0] - point[0], base_utm[1] - point[1])

    while improved and iterations < max_iter:
        improved = False
        iterations += 1
        for i in range(N - 1):
            for j in range(i + 1, N):
                reversed_first = (best_tour[j][1], best_tour[j][0])
                reversed_last = (best_tour[i][1], best_tour[i][0])
                old_boundary = (
                    base_distance(best_tour[0][0]) if i == 0
                    else transition(best_tour[i - 1], best_tour[i])
                ) + (
                    base_distance(best_tour[-1][1]) if j == N - 1
                    else transition(best_tour[j], best_tour[j + 1])
                )
                new_boundary = (
                    base_distance(reversed_first[0]) if i == 0
                    else transition(best_tour[i - 1], reversed_first)
                ) + (
                    base_distance(reversed_last[1]) if j == N - 1
                    else transition(reversed_last, best_tour[j + 1])
                )
                if best_score - old_boundary + new_boundary >= best_score - 1.0:
                    continue
                rev_segment = [(pe, ps) for (ps, pe) in reversed(best_tour[i:j + 1])]
                new_tour = best_tour[:i] + rev_segment + best_tour[j + 1:]
                new_score = score_full_tour(new_tour, base_utm, drone_type, turn_radius_m)
                if new_score < best_score - 1.0:
                    best_score = new_score
                    best_tour = new_tour
                    improved = True
                    break
            if improved:
                break
    return best_tour


def exact_swath_tour_for_base(
    swaths: List[List[Tuple[float, float]]],
    base_utm: Tuple[float, float],
    drone_type: str,
    turn_radius_m: float,
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Minimum approximate transit cost for all orders and directions (N <= 9)."""
    oriented = [[(swath[0], swath[-1]), (swath[-1], swath[0])]
                for swath in swaths]
    n = len(swaths)
    dp = {}
    parent = {}

    def transition(left, right):
        start_a, end_a = left
        start_b, end_b = right
        da = (end_a[0] - start_a[0], end_a[1] - start_a[1])
        db = (end_b[0] - start_b[0], end_b[1] - start_b[1])
        la = max(1e-3, math.hypot(*da))
        lb = max(1e-3, math.hypot(*db))
        return calc_transition_cost(
            end_a, (da[0] / la, da[1] / la),
            start_b, (db[0] / lb, db[1] / lb),
            drone_type, turn_radius_m,
        )

    for idx in range(n):
        for direction in (0, 1):
            point = oriented[idx][direction][0]
            dp[(1 << idx, idx, direction)] = math.hypot(
                point[0] - base_utm[0], point[1] - base_utm[1]
            )

    full_mask = (1 << n) - 1
    for mask in range(1, full_mask + 1):
        for last in range(n):
            if not mask & (1 << last):
                continue
            for direction in (0, 1):
                state = (mask, last, direction)
                cost = dp.get(state)
                if cost is None:
                    continue
                for nxt in range(n):
                    if mask & (1 << nxt):
                        continue
                    next_mask = mask | (1 << nxt)
                    for next_direction in (0, 1):
                        next_state = (next_mask, nxt, next_direction)
                        new_cost = cost + transition(oriented[last][direction], oriented[nxt][next_direction])
                        if new_cost < dp.get(next_state, math.inf):
                            dp[next_state] = new_cost
                            parent[next_state] = state

    best_state = min(
        ((full_mask, last, direction) for last in range(n) for direction in (0, 1)),
        key=lambda state: dp[state] + math.hypot(
            oriented[state[1]][state[2]][1][0] - base_utm[0],
            oriented[state[1]][state[2]][1][1] - base_utm[1],
        ),
    )
    tour = []
    state = best_state
    while True:
        _, last, direction = state
        tour.append(oriented[last][direction])
        if state not in parent:
            break
        state = parent[state]
    tour.reverse()
    return tour


def optimize_swath_tour_for_base(
    swaths: List[List[Tuple[float, float]]],
    base_utm: Tuple[float, float],
    drone_type: str = "fixed_wing",
    turn_radius_m: float = 85.0
) -> List[Tuple[Tuple[float, float], Tuple[float, float]]]:
    """Minimize estimated transit cost over swath orders and directions.

    The search is exact for at most nine swaths and heuristic for larger fields.
    Final mission selection uses measured route time after path construction.
    """
    if not swaths:
        return []
    if len(swaths) <= 9:
        return exact_swath_tour_for_base(swaths, base_utm, drone_type, turn_radius_m)
    N = len(swaths)
    start_candidates = []
    for idx, s in enumerate(swaths):
        d0 = math.hypot(base_utm[0] - s[0][0], base_utm[1] - s[0][1])
        d1 = math.hypot(base_utm[0] - s[-1][0], base_utm[1] - s[-1][1])
        start_candidates.append((d0, idx, 0))
        start_candidates.append((d1, idx, 1))
    start_candidates.sort(key=lambda x: x[0])

    best_overall_tour = None
    best_overall_cost = 1e12

    start_limit = 2 if N > 60 else 6
    for init_dist, first_idx, first_dir in start_candidates[:min(start_limit, len(start_candidates))]:
        unvisited = set(range(N))
        tour = []
        s = swaths[first_idx]
        tour.append((s[0], s[-1]) if first_dir == 0 else (s[-1], s[0]))
        unvisited.remove(first_idx)

        while unvisited:
            curr_s, curr_e = tour[-1]
            c_dx, c_dy = curr_e[0] - curr_s[0], curr_e[1] - curr_s[1]
            c_len = max(1e-3, math.hypot(c_dx, c_dy))
            c_dir = (c_dx / c_len, c_dy / c_len)

            best_cand, best_dir, best_cost = None, 0, 1e12
            for idx in unvisited:
                cand_s = swaths[idx]
                s_dx = cand_s[-1][0] - cand_s[0][0]
                s_dy = cand_s[-1][1] - cand_s[0][1]
                s_len = max(1e-3, math.hypot(s_dx, s_dy))
                fwd_dir = (s_dx / s_len, s_dy / s_len)
                rev_dir = (-fwd_dir[0], -fwd_dir[1])

                cost0 = calc_transition_cost(curr_e, c_dir, cand_s[0], fwd_dir, drone_type, turn_radius_m)
                cost1 = calc_transition_cost(curr_e, c_dir, cand_s[-1], rev_dir, drone_type, turn_radius_m)
                if cost0 < best_cost:
                    best_cost, best_cand, best_dir = cost0, idx, 0
                if cost1 < best_cost:
                    best_cost, best_cand, best_dir = cost1, idx, 1

            s = swaths[best_cand]
            tour.append((s[0], s[-1]) if best_dir == 0 else (s[-1], s[0]))
            unvisited.remove(best_cand)

        tour_opt = two_opt_tour(tour, base_utm, drone_type, turn_radius_m)
        cost_opt = score_full_tour(tour_opt, base_utm, drone_type, turn_radius_m)
        if cost_opt < best_overall_cost:
            best_overall_cost = cost_opt
            best_overall_tour = tour_opt

    return best_overall_tour


def compute_optimal_sweep_angle(
    poly_utm: Polygon,
    wind: WindConfig,
    base_utm: Optional[Tuple[float, float]] = None,
    sweep_angle_override: Optional[float] = None,
    line_spacing_m: float = 80.0,
    drone_speed_ms: float = 21.0,
    drone_turn_radius_m: float = 85.0
) -> float:
    """
    Эвристический выбор угла прокладки галсов из конечного набора кандидатов:
    1. Если пользователь явно задал угол (override >= 0), используется он.
    2. Если задан -1.0 ('axis'), строго берется главная продольная ось полигона.
    3. По умолчанию (режим 'auto'):
       - Анализируется форма полигона (главная ось MRR и ребра Convex Hull).
       - Оценивается набор углов-кандидатов с полным расчетом физической стоимости миссии:
         * T_swaths: время пролета галсов по ветровому треугольнику (учет попутно-встречного ветра).
         * T_turns: время и энергозатраты на развороты (N_swaths - 1) * (pi*R + lead_out) / V.
           Это доминирующий фактор: при вытянутом полигоне летать в ширину приводит к 50-70% лишних виражей!
         * Crosswind Penalty: при слабом и умеренном ветре (W <= 5 м/с) снос камеры легко парируется
           автопилотом и подвесом (штраф = 0). Лишь при сильном ветре (> 5.5 м/с) штраф квадратично
           возрастает, балансируя ориентацию ближе к ветру.
         * Base Transit: штраф за расстояние до базы.
       - Выбирается угол с минимальной оценкой стоимости среди проверенных углов.
    """
    if sweep_angle_override is not None and sweep_angle_override >= 0.0:
        return float(sweep_angle_override) % 180.0
        
    mrr = poly_utm.minimum_rotated_rectangle
    coords = list(mrr.exterior.coords)[:4]
    edge1 = (coords[1][0] - coords[0][0], coords[1][1] - coords[0][1])
    edge2 = (coords[2][0] - coords[1][0], coords[2][1] - coords[1][1])
    len1 = math.hypot(edge1[0], edge1[1])
    len2 = math.hypot(edge2[0], edge2[1])
    
    if len1 >= len2:
        long_axis_angle = math.degrees(math.atan2(edge1[1], edge1[0])) % 180.0
    else:
        long_axis_angle = math.degrees(math.atan2(edge2[1], edge2[0])) % 180.0

    if sweep_angle_override == -1.0:
        return round(long_axis_angle, 1)

    # Кандидаты на оптимальный угол:
    candidates = set([
        round(long_axis_angle, 1),
        round((long_axis_angle + 90.0) % 180.0, 1),
        round(float(wind.direction_deg) % 180.0, 1),
        round((float(wind.direction_deg) + 90.0) % 180.0, 1)
    ])
    
    # Добавляем углы выраженных прямых ребер выпуклой оболочки
    try:
        hull_pts = list(poly_utm.convex_hull.exterior.coords)[:-1]
        for i in range(len(hull_pts)):
            pA = hull_pts[i]
            pB = hull_pts[(i + 1) % len(hull_pts)]
            edx, edy = pB[0] - pA[0], pB[1] - pA[1]
            if math.hypot(edx, edy) > 150.0:
                candidates.add(round(math.degrees(math.atan2(edy, edx)) % 180.0, 1))
    except Exception:
        pass
        
    # Сетка углов с шагом 15 градусов
    for a in range(0, 180, 15):
        candidates.add(float(a))
        
    best_ang = long_axis_angle
    min_cost = 1e12
    
    rad_wind = math.radians(wind.direction_deg)
    turn_dist_m = math.pi * drone_turn_radius_m + 50.0
    
    for ang in candidates:
        swaths = plan_coverage_swaths(poly_utm, line_spacing_m, ang)
        if not swaths:
            continue
            
        n = len(swaths)
        tot_len = sum(math.hypot(s[-1][0] - s[0][0], s[-1][1] - s[0][1]) for s in swaths)
        
        # 1. Время пролета галсов по ветровому треугольнику
        w_fwd = solve_wind_triangle(ang, drone_speed_ms, wind.speed_ms, wind.direction_deg)
        w_bwd = solve_wind_triangle((ang + 180.0) % 360.0, drone_speed_ms, wind.speed_ms, wind.direction_deg)
        v_fwd = max(4.0, w_fwd['ground_speed_ms'])
        v_bwd = max(4.0, w_bwd['ground_speed_ms'])
        t_swaths = (tot_len * 0.5 / v_fwd) + (tot_len * 0.5 / v_bwd)
        
        # 2. Стоимость разворотов (время и расход батареи на вираж)
        t_turns = max(0, n - 1) * (turn_dist_m / drone_speed_ms)
        
        # 3. Штраф за боковой ветер (боковой снос камеры)
        rad_ang = math.radians(ang)
        crosswind = wind.speed_ms * abs(math.sin(rad_ang - rad_wind))
        pen_cross = 0.0
        # При ветре <= 5.0 м/с боковой снос полностью парируется подвесом и WCA автопилота
        if crosswind > 5.0:
            pen_cross = ((crosswind - 5.0) ** 2) * 15.0
            
        # 4. Согласование с точкой взлета/посадки
        pen_base = 0.0
        if base_utm is not None and swaths:
            d_b = min(
                math.hypot(base_utm[0] - swaths[0][0][0], base_utm[1] - swaths[0][0][1]),
                math.hypot(base_utm[0] - swaths[-1][0][0], base_utm[1] - swaths[-1][0][1])
            )
            pen_base = (d_b / drone_speed_ms) * 0.5
            
        total_cost = t_swaths + t_turns + pen_cross + pen_base
        if total_cost < min_cost:
            min_cost = total_cost
            best_ang = ang
            
    return round(best_ang, 1)


def plan_coverage_swaths(
    polygon_utm: Polygon,
    line_spacing_m: float,
    sweep_angle_deg: float,
    phase: float = 0.5,
) -> List[List[Tuple[float, float]]]:
    """
    Нарезает непрерывные параллельные галсы вдоль полигона.
    Исключает разрывы, внутренние перелеты и диагональные скачки.
    """
    poly_rot = rotate(polygon_utm, -sweep_angle_deg, origin='centroid')
    minx, miny, maxx, maxy = poly_rot.bounds
    
    y = miny + line_spacing_m * phase
    swaths = []
    
    while y <= maxy:
        sweep_line = LineString([(minx - 200.0, y), (maxx + 200.0, y)])
        inter = poly_rot.intersection(sweep_line)
        
        if not inter.is_empty:
            if inter.geom_type == 'LineString':
                coords = list(inter.coords)
                if len(coords) >= 2:
                    swath_len = math.hypot(coords[-1][0] - coords[0][0], coords[-1][1] - coords[0][1])
                    if swath_len >= 35.0:
                        ls = LineString([coords[0], coords[-1]])
                        ls_orig = rotate(ls, sweep_angle_deg, origin=poly_rot.centroid)
                        c_orig = list(ls_orig.coords)
                        if len(c_orig) >= 2:
                            swaths.append(c_orig)
            elif inter.geom_type == 'MultiLineString':
                # Для полигонов с вырезами (буферные зоны препятствий) сохраняем безопасные сегменты
                for g in inter.geoms:
                    coords = list(g.coords)
                    if len(coords) >= 2:
                        swath_len = math.hypot(coords[-1][0] - coords[0][0], coords[-1][1] - coords[0][1])
                        if swath_len >= 35.0:
                            ls = LineString([coords[0], coords[-1]])
                            ls_orig = rotate(ls, sweep_angle_deg, origin=poly_rot.centroid)
                            c_orig = list(ls_orig.coords)
                            if len(c_orig) >= 2:
                                swaths.append(c_orig)
                        
        y += line_spacing_m

    return swaths


def coverage_ratio_for_swaths(
    polygon_utm: Polygon,
    swaths: List[List[Tuple[float, float]]],
    footprint_width_m: float,
) -> float:
    if not swaths or polygon_utm.area <= 0:
        return 0.0
    footprints = [LineString(swath).buffer(footprint_width_m / 2.0, cap_style=2)
                  for swath in swaths]
    return polygon_utm.intersection(unary_union(footprints)).area / polygon_utm.area

def generate_feasibility_assessment(
    compatible_drones: List[DroneSpec],
    drone_plans: List[Dict[str, Any]],
    makespan_min: float,
    total_fleet_time_min: float,
    total_area_ha: float,
    max_allowed_time_min: Optional[float] = None,
    max_available_drones: Optional[int] = None,
    battery_swap_penalty_min: float = 15.0
) -> Dict[str, Any]:
    """
    Интеллектуальная система поддержки принятия решений (СППР) для оператора БВС.
    Реализует прямые требования заказчика (Геоскан) из экспертного чата:
    - Проверка выполнимости при лимите времени (конец светового дня / режим полета)
    - Оценка требуемого размера флота
    - Анализ холостых перелетов vs параллельного флота
    - Расход ресурса до ТО по официальным регламентам Геоскана
    """
    sorties_by_drone = {}
    for plan in drone_plans:
        sorties_by_drone[plan["drone_id"]] = sorties_by_drone.get(plan["drone_id"], 0) + 1
    active_drones_count = len(sorties_by_drone)
    
    # 1. Проверка лимитов пользователя
    time_feasible = True
    time_delta = 0.0
    if max_allowed_time_min is not None and max_allowed_time_min > 0:
        if makespan_min > max_allowed_time_min:
            time_feasible = False
            time_delta = round(makespan_min - max_allowed_time_min, 1)
            
    drones_feasible = True
    drones_delta = 0
    if max_available_drones is not None and max_available_drones > 0:
        if active_drones_count > max_available_drones:
            drones_feasible = False
            drones_delta = active_drones_count - max_available_drones
            
    is_overall_feasible = time_feasible and drones_feasible and all(
        plan["is_energy_safe"] for plan in drone_plans
    )
    
    # 2. Проверка ресурса АКБ и вылетов на замену
    sorties_needed = len(drone_plans)
    battery_swaps = sum(max(0, count - 1) for count in sorties_by_drone.values())
            
    # 3. Расход ресурса до ТО по нормативам каталога:
    # 201, 401 - каждые 80 полетов
    # 501, 801 - каждые 160 часов
    # 701 - каждые 100 моточасов
    to_impact = []
    for p in drone_plans:
        d_id = p["drone_id"]
        d_spec = GEOSCAN_DRONE_CATALOG.get(d_id)
        if not d_spec: continue
        h = round(p["flight_time_min"] / 60.0, 2)
        if d_spec.type == "fixed_wing":
            to_impact.append({
                "drone_name": d_spec.name,
                "type": d_spec.type,
                "normative": "Каждые 80 полетов",
                "usage_this_mission": "1 полет",
                "wear_pct": 1.25,
                "remaining_cycles_estimate": 79
            })
        elif "801" in d_id or "501" in d_id:
            to_impact.append({
                "drone_name": d_spec.name,
                "type": d_spec.type,
                "normative": "Каждые 160 часов налета",
                "usage_this_mission": f"{h} ч",
                "wear_pct": round((h / 160.0) * 100, 2),
                "remaining_cycles_estimate": round(160.0 - h, 1)
            })
        else:
            to_impact.append({
                "drone_name": d_spec.name,
                "type": d_spec.type,
                "normative": "Каждые 80 полетов",
                "usage_this_mission": f"1 полет ({h} ч)",
                "wear_pct": 1.25,
                "remaining_cycles_estimate": 79
            })

    # 5. Текстовое экспертное заключение
    advice_lines = []
    if is_overall_feasible:
        advice_lines.append(f"Задание полностью выполнимо: длительность {makespan_min:.1f} мин укладывается в заданный лимит времени.")
    else:
        if not time_feasible:
            advice_lines.append(f"Лимит времени превышен на {time_delta:.1f} мин (расчет: {makespan_min:.1f} мин, лимит: {max_allowed_time_min:.1f} мин).")
        if not drones_feasible:
            advice_lines.append(f"Превышено число бортов: задействовано {active_drones_count} БВС при лимите {max_available_drones}.")
            
    if battery_swaps > 0:
        advice_lines.append(f"Потребуется {battery_swaps} промежуточных посадок на замену АКБ (+{battery_swaps * battery_swap_penalty_min:.0f} мин на земле).")
    else:
        advice_lines.append("Все борта выполнят миссию на одном заряде АКБ (без посадок на подзарядку).")

    if active_drones_count == 1:
        advice_lines.append("План задействует один БВС.")
    else:
        advice_lines.append(f"План задействует {active_drones_count} БВС; длительность {makespan_min:.1f} мин.")

    return {
        "is_feasible": is_overall_feasible,
        "status_label": "ВЫПОЛНИМО" if is_overall_feasible else "ТРЕБУЕТСЯ КОРРЕКТИРОВКА",
        "time_feasible": time_feasible,
        "drones_feasible": drones_feasible,
        "makespan_min": makespan_min,
        "max_allowed_time_min": max_allowed_time_min,
        "max_available_drones": max_available_drones,
        "active_drones_count": active_drones_count,
        "battery_sorties_total": sorties_needed,
        "battery_swaps_needed": battery_swaps,
        "advisor_summary": " ".join(advice_lines),
        "fleet_scenarios": None,
        "maintenance_impact": to_impact
    }

def _plan_mission_candidate(
    polygon_geojson: dict,
    sensor: SensorSpec,
    drones: List[DroneSpec],
    wind: WindConfig,
    criterion: str = "min_makespan",
    target_gsd_cm: float = 3.0,
    launch_points: Optional[List[LaunchPoint]] = None,
    obstacles: Optional[List[Dict[str, Any]]] = None,
    airspace_zones: Optional[List[Dict[str, Any]]] = None,
    sweep_angle_deg: Optional[float] = None,
    max_allowed_time_min: Optional[float] = None,
    max_available_drones: Optional[int] = None,
    battery_swap_penalty_min: float = 15.0,
    assignment_split: Optional[int] = None,
    overlap_forward: Optional[float] = None,
    overlap_side: Optional[float] = None,
) -> Dict[str, Any]:
    """
    Главный конвейер планирования полетных заданий для группы БВС Геоскан
    """
    # 1. Фотограмметрический расчет
    photo_params = calculate_photogrammetry(sensor, target_gsd_cm, overlap_forward, overlap_side)
    flight_height_m = photo_params["flight_height_m"]
    line_spacing_m = photo_params["line_spacing_m"]
    trigger_dist_m = photo_params["trigger_dist_m"]
    
    # Сохраняем все компоненты и внутренние вырезы GeoJSON.
    geom = polygon_geojson.get("geometry", polygon_geojson)
    if geom.get("type") not in ("Polygon", "MultiPolygon"):
        return {"error": "Нужен полигон или мультиполигон GeoJSON"}
    try:
        poly_wgs = shape(geom)
    except (TypeError, ValueError, IndexError):
        return {"error": "Некорректная геометрия области съемки"}
    if not poly_wgs.is_valid or poly_wgs.is_empty or poly_wgs.area <= 0:
        return {"error": "Область съемки пуста или имеет некорректную геометрию"}
        
    c_lon, c_lat = poly_wgs.centroid.x, poly_wgs.centroid.y
    to_utm, to_wgs = get_utm_transformer(c_lon, c_lat)
    
    # Полигон в метрах UTM
    poly_utm = transform(to_utm, poly_wgs)
    
    # Анализ 3D препятствий и расчет безопасного обхода
    obstacle_analysis = []
    poly_survey = poly_utm
    if obstacles:
        for obs in obstacles:
            obs_h = obs.get("height_m", 50.0)
            obs_pt_wgs = obs.get("centroid")
            if not obs_pt_wgs: continue
            obs_pt_utm = Point(to_utm(obs_pt_wgs[0], obs_pt_wgs[1]))
            dist_to_poly = obs_pt_utm.distance(poly_utm)
            if dist_to_poly <= 1000.0:
                clearance = flight_height_m - obs_h
                if clearance >= 25.0:
                    obstacle_analysis.append({
                        "name": obs.get("name", "Вышка"),
                        "height_m": obs_h,
                        "flight_height_m": round(flight_height_m, 1),
                        "distance_to_parcel_m": round(dist_to_poly, 1),
                        "clearance_m": round(clearance, 1),
                        "status": "SAFE_OVERFLIGHT",
                        "description": f"Безопасный перелёт над вышкой: запас высоты {clearance:.1f} м (норма >= 25 м)"
                    })
                else:
                    # По физике фотограмметрии подниматься выше нельзя из-за GSD.
                    # Вырезаем зону безопасности 70 м вокруг препятствия (согласно экспертам в чате)
                    safe_buf = obs_pt_utm.buffer(70.0)
                    is_cut = False
                    if poly_survey.intersects(safe_buf):
                        poly_survey = poly_survey.difference(safe_buf)
                        is_cut = True
                    obstacle_analysis.append({
                        "name": obs.get("name", "Вышка"),
                        "height_m": obs_h,
                        "flight_height_m": round(flight_height_m, 1),
                        "distance_to_parcel_m": round(dist_to_poly, 1),
                        "clearance_m": round(clearance, 1),
                        "status": "HORIZONTAL_BYPASS",
                        "safety_buffer_radius_m": 70.0,
                        "is_cut_from_swaths": is_cut,
                        "description": f"Горизонтальный обход: высота съемки {flight_height_m:.1f}м ниже вышки {obs_h:.1f}м + буфер. Нарезка галсов скорректирована с радиусом 70 м."
                    })

    # 2. Базовая точка старта/посадки (определяется до нарезки галсов для сквозной оптимизации)
    if not launch_points:
        # Автоматическое создание ВПП на безопасном удалении 250 м от границы полигона с наветренной стороны
        rad_wind = math.radians(wind.direction_deg)
        wind_dir_vec = np.array([math.sin(rad_wind), math.cos(rad_wind)])
        base_polygon = max(poly_utm.geoms, key=lambda part: part.area) if isinstance(poly_utm, MultiPolygon) else poly_utm
        poly_pts = np.array(base_polygon.exterior.coords)
        projs = np.dot(poly_pts, -wind_dir_vec)
        best_pt_idx = int(np.argmax(projs))
        border_pt = poly_pts[best_pt_idx]
        base_x = float(border_pt[0] - wind_dir_vec[0] * 250.0)
        base_y = float(border_pt[1] - wind_dir_vec[1] * 250.0)
        base_lon, base_lat = to_wgs(base_x, base_y)
        launch_points = [
            LaunchPoint(id="base_1", name="ВПП-1 (Основная)", lon=base_lon, lat=base_lat, type="base"),
            LaunchPoint(id="pad_emerg", name="Резервная площадка Восток", lon=base_lon + 0.008, lat=base_lat + 0.003, type="emergency_pad")
        ]
        
    base = next((point for point in launch_points if point.type == "base"), None)
    if base is None:
        return {"error": "Нужна хотя бы одна точка старта типа base"}
    base_utm = to_utm(base.lon, base.lat)

    # 3. Выбор профессионального угла галсов (с учетом базы, формы полигона, разворотов и ветра)
    active_drones = drones if drones else list(GEOSCAN_DRONE_CATALOG.values())
    primary_drone = active_drones[0]
    optimal_angle = compute_optimal_sweep_angle(
        poly_survey, wind, base_utm, sweep_angle_deg,
        line_spacing_m=line_spacing_m,
        drone_speed_ms=primary_drone.cruise_speed,
        drone_turn_radius_m=primary_drone.turn_radius
    )
    
    # Нарезка галсов съемки с оптимальным центрированием
    all_swaths = plan_coverage_swaths(poly_survey, line_spacing_m, optimal_angle, phase=0.5)
    sweep_phase = 0.5
    if not all_swaths:
        all_swaths = plan_coverage_swaths(poly_survey, line_spacing_m, optimal_angle, phase=0.2)
        sweep_phase = 0.2
    if not all_swaths:
        return {"error": "Полигон слишком мал для выбранного шага галсов"}
        
    num_swaths = len(all_swaths)

    # 4. Распределение задач между БВС (Smart Base-Aware Task Allocation)
    active_drones = drones if drones else list(GEOSCAN_DRONE_CATALOG.values())
    compatible_drones = []
    fleet_rejections = {}
    
    # Строгая проверка совместимости БВС и сенсора
    for d in active_drones:
        reasons = []
        if sensor.id not in d.sensors_supported:
            reasons.append(f"не поддерживает сенсор {sensor.name}")
        elif flight_height_m < d.min_operational_height_m:
            reasons.append(f"высота полета {flight_height_m:.1f}м ниже мин. безопасной {d.min_operational_height_m:.1f}м")
        if wind.speed_ms > d.max_wind_resistance:
            reasons.append(f"ветер {wind.speed_ms:.1f} м/с выше допустимых {d.max_wind_resistance:.1f} м/с")
            
        if not reasons:
            compatible_drones.append(d)
        else:
            fleet_rejections[d.id] = reasons
            
    if not compatible_drones:
        rejection_details = []
        for did, rs in fleet_rejections.items():
            d_spec = GEOSCAN_DRONE_CATALOG.get(did)
            d_name = d_spec.name if d_spec else did
            rejection_details.append(f"{d_name}: {', '.join(rs)}")
        return {
            "error": f"Среди выбранных БВС нет совместимых (сенсор: {sensor.name}, высота: {flight_height_m:.1f} м). Причины: " + "; ".join(rejection_details)
        }

    # Учет ограничения пользователя по максимальному числу активных бортов
    if max_available_drones is not None and max_available_drones > 0:
        compatible_drones = compatible_drones[:max_available_drones]
    
    drone_assignments = []
    
    def quick_estimate_drone_mission_time(
        swaths_subset: List[List[Tuple[float, float]]],
        drone_spec: DroneSpec,
        b_utm: Tuple[float, float],
        w_cfg: WindConfig
    ) -> float:
        if not swaths_subset:
            return 0.0
        p_first = swaths_subset[0][0]
        p_last = swaths_subset[-1][-1]
        d_in = math.hypot(b_utm[0] - p_first[0], b_utm[1] - p_first[1])
        d_out = math.hypot(b_utm[0] - p_last[0], b_utm[1] - p_last[1])
        total_swath_len = sum(math.hypot(s[-1][0] - s[0][0], s[-1][1] - s[0][1]) for s in swaths_subset)
        turn_penalty_m = (2.0 * math.pi * drone_spec.turn_radius * 0.75) if drone_spec.type == "fixed_wing" else 15.0
        turns_len = max(0, len(swaths_subset) - 1) * turn_penalty_m
        eff_speed = max(3.0, drone_spec.cruise_speed - w_cfg.speed_ms * 0.25)
        return (d_in + total_swath_len + turns_len + d_out) / eff_speed

    def quick_estimate_drone_total_makespan(
        swaths_subset: List[List[Tuple[float, float]]],
        drone_spec: DroneSpec,
        b_utm: Tuple[float, float],
        w_cfg: WindConfig,
        swap_penalty_s: float = 900.0
    ) -> float:
        if not swaths_subset:
            return 0.0
        safe_seconds = 60.0 * drone_spec.max_flight_time_min * max(
            0.0, drone_spec.battery_level - drone_spec.reserve_battery_pct
        ) / 100.0
        budget = max(60.0, safe_seconds * 0.75)
        
        sorties = 1
        batch = []
        total_time_s = 0.0
        for s in swaths_subset:
            proposed = batch + [s]
            t = quick_estimate_drone_mission_time(proposed, drone_spec, b_utm, w_cfg)
            if batch and t > budget:
                sorties += 1
                total_time_s += quick_estimate_drone_mission_time(batch, drone_spec, b_utm, w_cfg)
                batch = [s]
            else:
                batch = proposed
        if batch:
            total_time_s += quick_estimate_drone_mission_time(batch, drone_spec, b_utm, w_cfg)
        return total_time_s + (sorties - 1) * swap_penalty_s

    if criterion == "min_flight_time" and len(compatible_drones) > 1:
        # Минимизация суммарного налета: отдаем приоритет самому энергоэффективному аппарату (крыло 201)
        fixed_wing = next((d for d in compatible_drones if d.type == "fixed_wing"), None)
        best_drone = fixed_wing if fixed_wing else max(compatible_drones, key=lambda d: d.cruise_speed)
        drone_assignments = [(best_drone, all_swaths)]
    elif len(compatible_drones) == 1:
        drone_assignments = [(compatible_drones[0], all_swaths)]
    else:
        # Минимизация Makespan: балансировка времени выполнения миссии с непрерывным пространственным разделением
        # Связно упорядочиваем галсы в порядке удаления от базы (без разрыва параллельности)
        dist_to_first = min(math.hypot(base_utm[0] - p[0], base_utm[1] - p[1]) for p in all_swaths[0])
        dist_to_last = min(math.hypot(base_utm[0] - p[0], base_utm[1] - p[1]) for p in all_swaths[-1])
        if dist_to_first <= dist_to_last:
            ordered_swaths = list(all_swaths)
        else:
            ordered_swaths = list(reversed(all_swaths))

        # 2. Сортируем дроны: мультироторы (Gemini) берут ближнюю зону, самолеты (201) берут дальнюю зону
        sorted_drones = sorted(
            compatible_drones,
            key=lambda d: (0 if d.type == "multirotor" else 1, d.cruise_speed * d.max_flight_time_min)
        )
        
        num_d = len(sorted_drones)
        num_s = len(ordered_swaths)
        
        if num_s < num_d:
            sorted_drones = sorted(sorted_drones, key=lambda d: -d.cruise_speed)[:num_s]
            num_d = len(sorted_drones)

        if num_d == 1:
            drone_assignments = [(sorted_drones[0], ordered_swaths)]
        elif num_d == 2:
            d1, d2 = sorted_drones[0], sorted_drones[1]
            swap_s = battery_swap_penalty_min * 60.0
            best_split = max(1, int(num_s * (d1.cruise_speed / (d1.cruise_speed + d2.cruise_speed))))
            best_makespan = 1e9
            
            for split in ([assignment_split] if assignment_split is not None else range(1, num_s)):
                if split is None or not 1 <= split < num_s:
                    continue
                s1 = ordered_swaths[:split]
                s2 = ordered_swaths[split:]
                t1 = quick_estimate_drone_total_makespan(s1, d1, base_utm, wind, swap_s)
                t2 = quick_estimate_drone_total_makespan(s2, d2, base_utm, wind, swap_s)
                mspan = max(t1, t2)
                if mspan < best_makespan:
                    best_makespan = mspan
                    best_split = split
                    
            drone_assignments = [
                (d1, ordered_swaths[:best_split]),
                (d2, ordered_swaths[best_split:])
            ]
        else:
            # Взвешивание с учетом располагаемой энергии каждого борта
            weights = []
            for d in sorted_drones:
                safe_s = 60.0 * d.max_flight_time_min * max(0.0, d.battery_level - d.reserve_battery_pct) / 100.0
                weights.append(max(1.0, d.cruise_speed * math.sqrt(max(60.0, safe_s))))
            total_weight = sum(weights)
            cur = 0
            for i, d in enumerate(sorted_drones):
                if i == len(sorted_drones) - 1:
                    chunk = ordered_swaths[cur:]
                else:
                    share = max(1, int(round(num_s * (weights[i] / total_weight))))
                    end_idx = min(num_s, cur + share)
                    chunk = ordered_swaths[cur:end_idx]
                    cur = end_idx
                if chunk:
                    drone_assignments.append((d, chunk))

    # Делим длинную работу на реальные вылеты с возвращением на базу.
    expanded_assignments = []
    for drone, swaths in drone_assignments:
        if not swaths:
            continue
        safe_seconds = 60.0 * drone.max_flight_time_min * max(
            0.0, drone.battery_level - drone.reserve_battery_pct
        ) / 100.0
        estimate_budget = safe_seconds * 0.7
        batch = []
        for swath in swaths:
            proposed = batch + [swath]
            if batch and quick_estimate_drone_mission_time(proposed, drone, base_utm, wind) > estimate_budget:
                expanded_assignments.append((drone, batch))
                batch = [swath]
            else:
                batch = proposed
        if batch:
            expanded_assignments.append((drone, batch))

    # 5. Формирование траекторий, ключевых точек (Waypoints) и экспорта
    drone_plans = []
    total_fleet_flight_time_s = 0.0
    total_fleet_distance_m = 0.0
    makespan_s = 0.0
    
    sortie_counts = {}
    for drone, swaths in expanded_assignments:
        sortie_counts[drone.id] = sortie_counts.get(drone.id, 0) + 1
        waypoints = []
        path_utm_pts = []
        flight_distance_m = 0.0
        flight_time_s = 0.0
        battery_spent_pct = 0.0
        
        # Интеллектуальная оптимизация тура данного борта с привязкой к базе
        directed_swaths = optimize_swath_tour_for_base(
            swaths, base_utm, drone_type=drone.type, turn_radius_m=drone.turn_radius
        )
        if not directed_swaths:
            continue
        first_p_start, first_p_end = directed_swaths[0]
        last_p_start, last_p_end = directed_swaths[-1]
        
        # 1. Взлет
        waypoints.append({
            "stage": "TAKEOFF",
            "lon": base.lon,
            "lat": base.lat,
            "alt_m": flight_height_m,
            "speed_ms": drone.cruise_speed,
            "action": "LAUNCH_CATAPULT" if drone.takeoff_type == "catapult" else "VTOL_TAKEOFF"
        })
        path_utm_pts.append(base_utm)
        
        # 2. Плавный выход на первый галс (Approach Lead-In)
        if drone.type == "fixed_wing":
            entry_pts = plan_entry_approach(
                base_utm, first_p_start, first_p_end,
                R=drone.turn_radius, lead_in=60.0
            )
            for ep in entry_pts[1:-1]:
                ep_lon, ep_lat = to_wgs(ep[0], ep[1])
                waypoints.append({
                    "stage": "APPROACH_ENTRY",
                    "lon": round(ep_lon, 6),
                    "lat": round(ep_lat, 6),
                    "alt_m": flight_height_m,
                    "speed_ms": drone.cruise_speed,
                    "action": "ALIGN_SWATH_AXIS"
                })
                path_utm_pts.append(ep)
                
                leg_entry = calculate_flight_leg_utm(
                    path_utm_pts[-2], ep,
                    drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                    drone.type, drone.max_flight_time_min
                )
                flight_distance_m += leg_entry["distance_m"]
                flight_time_s += leg_entry["time_s"]
                battery_spent_pct += leg_entry["battery_used_pct"]
        else:
            # Для мультироторов прямой транзит к створу
            leg_transit = calculate_flight_leg_utm(
                base_utm, first_p_start,
                drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                drone.type, drone.max_flight_time_min
            )
            flight_distance_m += leg_transit["distance_m"]
            flight_time_s += leg_transit["time_s"]
            battery_spent_pct += leg_transit["battery_used_pct"]
        
        # 3. Проход оптимизированных галсов
        for s_idx, (p_start, p_end) in enumerate(directed_swaths):
            # Вход на галс (начало фотосъемки / сканирования)
            w_start_lon, w_start_lat = to_wgs(p_start[0], p_start[1])
            waypoints.append({
                "stage": "SURVEY_LINE",
                "swath_idx": s_idx + 1,
                "lon": round(w_start_lon, 6),
                "lat": round(w_start_lat, 6),
                "alt_m": flight_height_m,
                "speed_ms": drone.cruise_speed,
                "action": "START_CAMERA_TRIGGER",
                "trigger_dist_m": trigger_dist_m
            })
            path_utm_pts.append(p_start)
            if len(path_utm_pts) >= 2 and (s_idx > 0 or drone.type == "fixed_wing"):
                leg_align = calculate_flight_leg_utm(
                    path_utm_pts[-2], p_start,
                    drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                    drone.type, drone.max_flight_time_min
                )
                flight_distance_m += leg_align["distance_m"]
                flight_time_s += leg_align["time_s"]
                battery_spent_pct += leg_align["battery_used_pct"]
            
            # Рабочий пролет по галсу
            leg_swath = calculate_flight_leg_utm(
                p_start, p_end,
                drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                drone.type, drone.max_flight_time_min
            )
            flight_distance_m += leg_swath["distance_m"]
            flight_time_s += leg_swath["time_s"]
            battery_spent_pct += leg_swath["battery_used_pct"]
            
            w_end_lon, w_end_lat = to_wgs(p_end[0], p_end[1])
            waypoints.append({
                "stage": "SURVEY_LINE_END",
                "swath_idx": s_idx + 1,
                "lon": round(w_end_lon, 6),
                "lat": round(w_end_lat, 6),
                "alt_m": flight_height_m,
                "speed_ms": drone.cruise_speed,
                "action": "PAUSE_CAMERA_TRIGGER"
            })
            path_utm_pts.append(p_end)
            
            # Разворот на следующий галс
            if s_idx < len(directed_swaths) - 1:
                next_p_start, next_p_end = directed_swaths[s_idx + 1]
                
                if drone.type == "fixed_wing":
                    swath_heading = leg_swath["heading_deg"]
                    # Вычисляем путевой угол следующего галса
                    next_dx = next_p_end[0] - next_p_start[0]
                    next_dy = next_p_end[1] - next_p_start[1]
                    next_heading = (math.degrees(math.atan2(next_dx, next_dy)) + 360.0) % 360.0
                    
                    turn_pts = generate_dubins_turn(
                        p_end, swath_heading,
                        next_p_start, next_heading,
                        drone.turn_radius
                    )
                    for tp in turn_pts:
                        tp_lon, tp_lat = to_wgs(tp[0], tp[1])
                        waypoints.append({
                            "stage": "TURN_DUBINS",
                            "lon": round(tp_lon, 6),
                            "lat": round(tp_lat, 6),
                            "alt_m": flight_height_m,
                            "speed_ms": drone.cruise_speed,
                            "action": "BANKED_TURN"
                        })
                        path_utm_pts.append(tp)
                        
                        leg_turn = calculate_flight_leg_utm(
                            path_utm_pts[-2], tp,
                            drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                            drone.type, drone.max_flight_time_min
                        )
                        flight_distance_m += leg_turn["distance_m"]
                        flight_time_s += leg_turn["time_s"]
                        battery_spent_pct += leg_turn["battery_used_pct"]
                else:
                    turn_pts = generate_multirotor_turn(p_end, next_p_start, radius_m=4.0)
                    for tp in turn_pts:
                        tp_lon, tp_lat = to_wgs(tp[0], tp[1])
                        waypoints.append({
                            "stage": "TURN_VTOL",
                            "lon": round(tp_lon, 6),
                            "lat": round(tp_lat, 6),
                            "alt_m": flight_height_m,
                            "speed_ms": drone.cruise_speed * 0.85,
                            "action": "COORDINATED_TURN"
                        })
                        path_utm_pts.append(tp)
                        
                        leg_turn = calculate_flight_leg_utm(
                            path_utm_pts[-2], tp,
                            drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                            drone.type, drone.max_flight_time_min
                        )
                        flight_distance_m += leg_turn["distance_m"]
                        flight_time_s += leg_turn["time_s"]
                        battery_spent_pct += leg_turn["battery_used_pct"]

        # 4. Возврат и посадка (к ближайшей точке ВПП)
        if drone.type == "fixed_wing":
            exit_pts = plan_exit_approach(
                last_p_end, last_p_start, base_utm,
                R=drone.turn_radius, lead_out=30.0
            )
            for ep in exit_pts[1:-1]:
                ep_lon, ep_lat = to_wgs(ep[0], ep[1])
                waypoints.append({
                    "stage": "APPROACH_LANDING",
                    "lon": round(ep_lon, 6),
                    "lat": round(ep_lat, 6),
                    "alt_m": flight_height_m,
                    "speed_ms": drone.cruise_speed,
                    "action": "GLIDE_TO_BASE"
                })
                path_utm_pts.append(ep)
                
                leg_exit = calculate_flight_leg_utm(
                    path_utm_pts[-2], ep,
                    drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                    drone.type, drone.max_flight_time_min
                )
                flight_distance_m += leg_exit["distance_m"]
                flight_time_s += leg_exit["time_s"]
                battery_spent_pct += leg_exit["battery_used_pct"]

            waypoints.append({
                "stage": "LANDING",
                "lon": base.lon,
                "lat": base.lat,
                "alt_m": 0.0,
                "speed_ms": 0.0,
                "action": "PARACHUTE_DEPLOY"
            })
            path_utm_pts.append(base_utm)
        else:
            leg_return = calculate_flight_leg_utm(
                last_p_end, base_utm,
                drone.cruise_speed, wind.speed_ms, wind.direction_deg,
                drone.type, drone.max_flight_time_min
            )
            flight_distance_m += leg_return["distance_m"]
            flight_time_s += leg_return["time_s"]
            battery_spent_pct += leg_return["battery_used_pct"]
            
            waypoints.append({
                "stage": "LANDING",
                "lon": base.lon,
                "lat": base.lat,
                "alt_m": 0.0,
                "speed_ms": 0.0,
                "action": "VTOL_LAND"
            })
            path_utm_pts.append(base_utm)

        
        # Метрики считаются по той же полилинии, которая попадет в экспорт.
        # Это включает последний участок возвращения к базе.
        flight_distance_m = 0.0
        flight_time_s = 0.0
        battery_spent_pct = 0.0
        waypoint_times_s = [0.0]
        for idx in range(1, len(path_utm_pts)):
            leg_speed = waypoints[idx]["speed_ms"] or drone.cruise_speed
            leg = calculate_flight_leg_utm(
                path_utm_pts[idx - 1], path_utm_pts[idx], leg_speed,
                wind.speed_ms, wind.direction_deg, drone.type,
                drone.max_flight_time_min
            )
            if not math.isfinite(leg["time_s"]):
                return {"error": f"Ветер делает маршрут БВС {drone.name} невыполнимым"}
            flight_distance_m += leg["distance_m"]
            flight_time_s += leg["time_s"]
            battery_spent_pct += leg["battery_used_pct"]
            waypoint_times_s.append(round(flight_time_s, 2))

        # GeoJSON LineString маршрута
        route_coords = [list(to_wgs(pt[0], pt[1])) + [flight_height_m] for pt in path_utm_pts]
        
        # Расчет безопасного резерва батареи (Point of Safe Return)
        rem_battery = max(0.0, round(drone.battery_level - battery_spent_pct, 1))
        is_safe = rem_battery >= drone.reserve_battery_pct
        
        # Регламент ТО из чата экспертов
        maintenance_info = {
            "flight_limit": drone.maintenance_flight_limit,
            "limit_unit": "полетов" if drone.type == "fixed_wing" else "часов",
            "usage_this_mission": 1 if drone.type == "fixed_wing" else round(flight_time_s / 3600.0, 2),
            "status": "Норма"
        }
        
        # Расчет полноценной аварийной посадки на резервную площадку (Non-fictional Emergency Diversion)
        emergency_diversion = None
        emergency_pads = [point for point in (launch_points or []) if point.type == "emergency_pad"]
        if emergency_pads:
            nearest_pad = min(
                emergency_pads,
                key=lambda pad: math.hypot(to_utm(pad.lon, pad.lat)[0] - last_p_end[0],
                                           to_utm(pad.lon, pad.lat)[1] - last_p_end[1])
            )
            pad_utm = to_utm(nearest_pad.lon, nearest_pad.lat)
            leg_emerg = calculate_flight_leg_utm(
                last_p_end, pad_utm, drone.cruise_speed,
                wind.speed_ms, wind.direction_deg, drone.type, drone.max_flight_time_min
            )
            
            # 1. Расчет критической точки возврата (PSR - Point of Safe Return)
            psr_idx = len(waypoints) - 1
            psr_coord = [waypoints[-1]["lon"], waypoints[-1]["lat"]]
            for wp_i, wp_dict in enumerate(waypoints):
                t_wp = waypoint_times_s[wp_i] if wp_i < len(waypoint_times_s) else flight_time_s
                used_bat = (t_wp / (drone.max_flight_time_min * 60.0)) * 100.0
                cur_bat = max(0.0, drone.battery_level - used_bat)
                wp_u = to_utm(wp_dict["lon"], wp_dict["lat"])
                ret_cost = calculate_flight_leg_utm(
                    wp_u, base_utm, drone.cruise_speed,
                    wind.speed_ms, wind.direction_deg, drone.type, drone.max_flight_time_min
                )
                if cur_bat < ret_cost["battery_used_pct"] + drone.reserve_battery_pct:
                    psr_idx = max(0, wp_i - 1)
                    psr_coord = [waypoints[psr_idx]["lon"], waypoints[psr_idx]["lat"]]
                    break

            # 2. Построение эшелонированного прямого захода на аварийную ВПП / площадку
            breakoff_wgs = list(to_wgs(last_p_end[0], last_p_end[1]))
            dx = pad_utm[0] - last_p_end[0]
            dy = pad_utm[1] - last_p_end[1]
            dist_pad = math.hypot(dx, dy)
            u_dir = (dx / dist_pad, dy / dist_pad) if dist_pad > 1.0 else (0.0, 1.0)
            
            if drone.type == "fixed_wing":
                # Самолет: прямой посадочный коридор к резервной ВПП со снижением и выбросом парашюта
                drift_time_s = 9.0  # снижение на парашюте с H=40м со скоростью ~4.5 м/с
                drift_offset_m = wind.speed_ms * drift_time_s
                
                # Точка начала снижения и торможения перед ВПП (IAF)
                d_iaf = min(160.0, max(40.0, dist_pad * 0.35))
                iaf_utm = (pad_utm[0] - u_dir[0] * d_iaf, pad_utm[1] - u_dir[1] * d_iaf)
                
                # Точка глушения тяги и выброса парашюта на глиссаде
                d_deploy = min(45.0, max(15.0, dist_pad * 0.12))
                deploy_utm = (pad_utm[0] - u_dir[0] * d_deploy, pad_utm[1] - u_dir[1] * d_deploy)
                
                iaf_wgs = list(to_wgs(iaf_utm[0], iaf_utm[1]))
                deploy_wgs = list(to_wgs(deploy_utm[0], deploy_utm[1]))
                
                landing_desc = (
                    f"Прямой аварийный сход на резервную ВПП. Снижение до 50м на рубеже захода ({round(d_iaf)}м до ВПП), "
                    f"глушение двигателя и раскрытие парашюта на H=40м (ожидаемый ветровой снос {drift_offset_m:.1f}м)."
                )
                emerg_wps = [
                    {"lon": round(breakoff_wgs[0], 6), "lat": round(breakoff_wgs[1], 6), "alt_m": round(flight_height_m, 1), "speed_ms": drone.cruise_speed, "stage": "EMERGENCY_DIVERSION", "action": "BREAKOFF_TRANSIT"},
                    {"lon": round(iaf_wgs[0], 6), "lat": round(iaf_wgs[1], 6), "alt_m": 50.0, "speed_ms": drone.min_speed, "stage": "APPROACH_IAF", "action": "DECELERATE_ALIGN"},
                    {"lon": round(deploy_wgs[0], 6), "lat": round(deploy_wgs[1], 6), "alt_m": 40.0, "speed_ms": drone.min_speed, "stage": "PARACHUTE_DEPLOY", "action": "CUT_ENGINE_DEPLOY"},
                    {"lon": round(nearest_pad.lon, 6), "lat": round(nearest_pad.lat, 6), "alt_m": 0.0, "speed_ms": 0.0, "stage": "LANDING", "action": "TOUCHDOWN_PARACHUTE"}
                ]
            else:
                # Коптер (VTOL): пологая глиссада снижения и вертикальное касание
                d_iaf = min(70.0, max(25.0, dist_pad * 0.3))
                d_hover = min(18.0, max(8.0, dist_pad * 0.08))
                
                iaf_utm = (pad_utm[0] - u_dir[0] * d_iaf, pad_utm[1] - u_dir[1] * d_iaf)
                hover_utm = (pad_utm[0] - u_dir[0] * d_hover, pad_utm[1] - u_dir[1] * d_hover)
                
                iaf_wgs = list(to_wgs(iaf_utm[0], iaf_utm[1]))
                hover_wgs = list(to_wgs(hover_utm[0], hover_utm[1]))
                
                landing_desc = f"Пологая глиссада с высоты {flight_height_m:.1f}м до 12м над площадкой, зависание и вертикальная посадка VTOL."
                emerg_wps = [
                    {"lon": round(breakoff_wgs[0], 6), "lat": round(breakoff_wgs[1], 6), "alt_m": round(flight_height_m, 1), "speed_ms": drone.cruise_speed, "stage": "EMERGENCY_DIVERSION", "action": "BREAKOFF_TRANSIT"},
                    {"lon": round(iaf_wgs[0], 6), "lat": round(iaf_wgs[1], 6), "alt_m": 25.0, "speed_ms": 6.0, "stage": "APPROACH_IAF", "action": "DECELERATE_GLIDE"},
                    {"lon": round(hover_wgs[0], 6), "lat": round(hover_wgs[1], 6), "alt_m": 12.0, "speed_ms": 3.0, "stage": "FINAL_HOVER", "action": "VTOL_DESCENT"},
                    {"lon": round(nearest_pad.lon, 6), "lat": round(nearest_pad.lat, 6), "alt_m": 0.0, "speed_ms": 0.0, "stage": "LANDING", "action": "TOUCHDOWN_VTOL"}
                ]

            emerg_coords = [[wp["lon"], wp["lat"], wp["alt_m"]] for wp in emerg_wps]

            emergency_diversion = {
                "pad_id": nearest_pad.id,
                "pad_name": nearest_pad.name,
                "lon": nearest_pad.lon,
                "lat": nearest_pad.lat,
                "distance_km": round(leg_emerg["distance_m"] / 1000.0, 2),
                "flight_time_min": round(leg_emerg["time_s"] / 60.0, 1),
                "battery_needed_pct": round(leg_emerg["battery_used_pct"], 1),
                "landing_procedure": landing_desc,
                "psr_info": {
                    "waypoint_index": psr_idx,
                    "lon": round(psr_coord[0], 6),
                    "lat": round(psr_coord[1], 6),
                    "description": f"Точка безопасного возврата (PSR, WP {psr_idx + 1})"
                },
                "waypoints": emerg_wps,
                "geojson_linestring": {
                    "type": "LineString",
                    "coordinates": emerg_coords
                }
            }

        plan_item = {
            "drone_id": drone.id,
            "drone_name": drone.name,
            "sortie_index": sortie_counts[drone.id],
            "drone_type": drone.type,
            "sensor_id": sensor.id,
            "sensor_name": sensor.name,
            "flight_height_m": flight_height_m,
            "cruise_speed_ms": drone.cruise_speed,
            "swaths_count": len(swaths),
            "distance_km": round(flight_distance_m / 1000.0, 2),
            "flight_time_min": round(flight_time_s / 60.0, 1),
            "flight_time_s": round(flight_time_s, 1),
            "battery_used_pct": round(battery_spent_pct, 1),
            "battery_remaining_pct": rem_battery,
            "is_energy_safe": is_safe,
            "maintenance_info": maintenance_info,
            "emergency_diversion": emergency_diversion,
            "waypoints": waypoints,
            "waypoint_times_s": waypoint_times_s,
            "geojson_linestring": {
                "type": "LineString",
                "coordinates": route_coords
            }
        }
        drone_plans.append(plan_item)
        
        total_fleet_flight_time_s += flight_time_s
        total_fleet_distance_m += flight_distance_m
        if flight_time_s > makespan_s:
            makespan_s = flight_time_s

    # Один БВС выполняет собственные вылеты последовательно.
    elapsed_by_drone = {}
    for plan in drone_plans:
        elapsed_by_drone[plan["drone_id"]] = elapsed_by_drone.get(plan["drone_id"], 0.0) + plan["flight_time_s"]
    for drone_id in elapsed_by_drone:
        sorties = sortie_counts[drone_id]
        elapsed_by_drone[drone_id] += max(0, sorties - 1) * battery_swap_penalty_min * 60.0
    makespan_s = max(elapsed_by_drone.values(), default=0.0)

    footprints = []
    half_footprint = photo_params["footprint_w_m"] / 2.0
    for plan in drone_plans:
        waypoints = plan["waypoints"]
        for idx, waypoint in enumerate(waypoints[:-1]):
            if waypoint["stage"] == "SURVEY_LINE" and waypoints[idx + 1]["stage"] == "SURVEY_LINE_END":
                end = waypoints[idx + 1]
                line = LineString([
                    to_utm(waypoint["lon"], waypoint["lat"]),
                    to_utm(end["lon"], end["lat"]),
                ])
                footprints.append(line.buffer(half_footprint, cap_style=2))
    covered_area = poly_survey.intersection(unary_union(footprints)).area if footprints else 0.0
    coverage_pct = 100.0 * covered_area / poly_survey.area if poly_survey.area else 0.0

    # Порог для отдельной проверки оператором; не является юридическим заключением.
    height_warning = None
    if flight_height_m > 150.0:
        height_warning = (
            f"Высота полета H = {flight_height_m:.1f} м превышает порог 150 м. "
            "Проверьте применимые ограничения воздушного пространства и необходимые разрешения до вылета."
        )

    # Интеллектуальный ТЭО-анализ (СППР для оператора)
    feasibility = generate_feasibility_assessment(
        compatible_drones=compatible_drones,
        drone_plans=drone_plans,
        makespan_min=round(makespan_s / 60.0, 1),
        total_fleet_time_min=round(total_fleet_flight_time_s / 60.0, 1),
        total_area_ha=round(poly_utm.area / 10000.0, 2),
        max_allowed_time_min=max_allowed_time_min,
        max_available_drones=max_available_drones,
        battery_swap_penalty_min=battery_swap_penalty_min
    )

    return {
        "status": "success",
        "optimization_criterion": criterion,
        "photogrammetry": photo_params,
        "height_warning": height_warning,
        "fleet_rejections": fleet_rejections,
        "obstacle_analysis": obstacle_analysis,
        "feasibility": feasibility,
        "wind_conditions": {
            "speed_ms": wind.speed_ms,
            "direction_deg": wind.direction_deg,
            "optimal_sweep_angle_deg": round(optimal_angle, 1),
            "sweep_phase": sweep_phase,
        },
        "metrics": {
            "total_survey_area_ha": round(poly_utm.area / 10000.0, 2),
            "total_swaths": num_swaths,
            "coverage_pct": round(min(100.0, coverage_pct), 2),
            "makespan_min": round(makespan_s / 60.0, 1),
            "total_fleet_distance_km": round(total_fleet_distance_m / 1000.0, 2),
            "total_fleet_time_min": round(total_fleet_flight_time_s / 60.0, 1),
            "active_drones_count": len(elapsed_by_drone),
            "optimal_sweep_angle_deg": round(optimal_angle, 1)
        },
        "drone_plans": drone_plans
    }


def _candidate_violation(
    result: Dict[str, Any],
    obstacles: Optional[List[Dict[str, Any]]],
    airspace_zones: Optional[List[Dict[str, Any]]],
    avoid_nfz: bool,
    allowed_airspace_geojson: Optional[dict] = None,
) -> Optional[str]:
    """Reject a complete route that violates a hard flight constraint."""
    if result["metrics"]["coverage_pct"] < 99.0:
        return f"Съемка покрывает только {result['metrics']['coverage_pct']:.2f}% допустимой области"
    for plan in result["drone_plans"]:
        if not plan["is_energy_safe"]:
            return f"Недостаточный запас батареи у {plan['drone_name']}"
        coords = plan["geojson_linestring"]["coordinates"]
        if allowed_airspace_geojson:
            allowed = shape(allowed_airspace_geojson.get("geometry", allowed_airspace_geojson))
            if not allowed.buffer(1e-7).covers(LineString([(c[0], c[1]) for c in coords])):
                return "Маршрут выходит за пределы указанной разрешенной области"
        to_utm, _ = get_utm_transformer(coords[0][0], coords[0][1])
        route = LineString([to_utm(c[0], c[1]) for c in coords])
        height = plan["flight_height_m"]

        for obstacle in obstacles or []:
            center = obstacle.get("centroid")
            if center and height - obstacle.get("height_m", 50.0) < 25.0:
                if route.distance(Point(to_utm(center[0], center[1]))) < 70.0:
                    return f"Маршрут проходит ближе 70 м к препятствию {obstacle.get('name', '')}"

        if avoid_nfz:
            for zone in airspace_zones or []:
                if not zone["geometry"].intersects(
                    LineString([(c[0], c[1]) for c in coords])
                ):
                    continue
                altitude = zone.get("altitude_info", {})
                min_alt = float(altitude.get("min_alt_m", 0.0))
                max_alt = float(altitude.get("max_alt_m", math.inf))
                ref = str(altitude.get("reference", "AMSL")).upper()
                raw = str(altitude.get("raw_text", "")).lower()

                # Учет высоты рельефа в Московской области (до 300м AMSL) и буфера безопасности 30м
                SAFETY_BUFFER_M = 30.0
                TERRAIN_MAX_AMSL = 300.0

                is_agl = ref == "AGL" or "от земли" in raw or "gnd" in raw
                if is_agl:
                    # Зона от уровня земли (AGL): безопасен пролет строго ниже зоны или строго выше
                    vertically_clear = (height + SAFETY_BUFFER_M < min_alt) or (height - SAFETY_BUFFER_M > max_alt)
                else:
                    # Зона от уровня моря (AMSL / эшелоны FL)
                    # Высота дрона над уровнем моря с учетом максимальной высоты рельефа
                    drone_amsl_max = height + TERRAIN_MAX_AMSL + SAFETY_BUFFER_M
                    drone_amsl_min = max(0.0, height)
                    # Если нижняя граница зоны (например, FL150 = 4572м) существенно выше высоты дрона с рельефом
                    vertically_clear = (drone_amsl_max < min_alt) or (drone_amsl_min > max_alt + SAFETY_BUFFER_M)

                if not vertically_clear:
                    return f"Маршрут пересекает зону {zone.get('name', '')} (H={min_alt:.0f}..{max_alt:.0f}м); эшелон полета {height:.0f}м не обеспечивает безопасный интервал"
    return None


def plan_multi_uav_mission(
    polygon_geojson: dict,
    sensor: SensorSpec,
    drones: List[DroneSpec],
    wind: WindConfig,
    criterion: str = "min_makespan",
    target_gsd_cm: float = 3.0,
    launch_points: Optional[List[LaunchPoint]] = None,
    allowed_airspace_geojson: Optional[dict] = None,
    obstacles: Optional[List[Dict[str, Any]]] = None,
    airspace_zones: Optional[List[Dict[str, Any]]] = None,
    sweep_angle_deg: Optional[float] = None,
    max_allowed_time_min: Optional[float] = None,
    max_available_drones: Optional[int] = None,
    battery_swap_penalty_min: float = 15.0,
    avoid_nfz: bool = True,
    overlap_forward: Optional[float] = None,
    overlap_side: Optional[float] = None,
) -> Dict[str, Any]:
    """Compare complete, feasible missions for the requested objective.

    This is a bounded search over sweep angles, fleet subsets and two-drone
    splits. It reports the best evaluated route, not a global optimum proof.
    """
    if criterion not in ("min_makespan", "min_flight_time"):
        return {"error": "Неизвестный критерий оптимизации"}
    if not drones:
        return {"error": "Не выбран ни один БВС"}

    limit = min(len(drones), max_available_drones) if max_available_drones else len(drones)
    subsets = [list(group) for size in range(1, limit + 1)
               for group in combinations(drones, size)]
    subsets.sort(key=lambda group: -len(group) if criterion == "min_makespan" else len(group))
    angles = [sweep_angle_deg] if sweep_angle_deg is not None else [None, -1.0, wind.direction_deg % 180.0]
    if launch_points:
        emergency_pads = [point for point in launch_points if point.type == "emergency_pad"]
        base_options = [[point, *emergency_pads] for point in launch_points if point.type == "base"]
        if not base_options:
            return {"error": "Нужна хотя бы одна точка старта типа base"}
    else:
        base_options = [None]
    configurations = [(base_option, index, angle)
                      for index, angle in enumerate(angles)
                      for base_option in base_options]
    best = None
    best_key = None
    evaluated = 0
    rejected = 0
    errors = []
    scenario_best = {}
    auto_angle_by_drone = {}
    started = time.monotonic()
    deadline_s = 12.0

    for base_option, angle_index, angle in configurations:
        for subset in subsets:
            if evaluated and time.monotonic() - started >= deadline_s:
                break
            splits = [None]
            for split in splits:
                if evaluated and time.monotonic() - started >= deadline_s:
                    break
                angle_key = (base_option[0].id if base_option else None, subset[0].id)
                candidate_angle = auto_angle_by_drone.get(angle_key, angle) if angle is None else angle
                result = _plan_mission_candidate(
                    polygon_geojson, sensor, subset, wind,
                    criterion="min_makespan" if len(subset) > 1 else "min_flight_time",
                    target_gsd_cm=target_gsd_cm, launch_points=base_option,
                    obstacles=obstacles, airspace_zones=airspace_zones,
                    sweep_angle_deg=candidate_angle, max_allowed_time_min=max_allowed_time_min,
                    max_available_drones=max_available_drones,
                    battery_swap_penalty_min=battery_swap_penalty_min,
                    assignment_split=split,
                    overlap_forward=overlap_forward,
                    overlap_side=overlap_side,
                )
                evaluated += 1
                if "error" in result:
                    errors.append(result["error"])
                    continue
                if angle is None:
                    auto_angle_by_drone[angle_key] = result["metrics"]["optimal_sweep_angle_deg"]
                # Оптимальный split для двух дронов уже аналитически вычислен внутри _plan_mission_candidate
                violation = _candidate_violation(
                    result, obstacles, airspace_zones, avoid_nfz, allowed_airspace_geojson
                )
                if max_allowed_time_min is not None and result["metrics"]["makespan_min"] > max_allowed_time_min:
                    violation = f"Маршрут не укладывается в лимит {max_allowed_time_min:.1f} мин"
                if violation:
                    rejected += 1
                    errors.append(violation)
                    continue
                plans = result["drone_plans"]
                elapsed = {}
                sorties = {}
                for plan in plans:
                    drone_id = plan["drone_id"]
                    elapsed[drone_id] = elapsed.get(drone_id, 0.0) + plan["flight_time_s"]
                    sorties[drone_id] = sorties.get(drone_id, 0) + 1
                elapsed_with_swaps = [
                    seconds + (sorties[drone_id] - 1) * battery_swap_penalty_min * 60.0
                    for drone_id, seconds in elapsed.items()
                ]
                total_flight_s = sum(elapsed.values())
                objective = max(elapsed_with_swaps) if criterion == "min_makespan" else total_flight_s
                secondary = total_flight_s if criterion == "min_makespan" else max(elapsed_with_swaps)
                key = (objective, secondary, len(elapsed))
                scenario_kind = "single_drone" if len(elapsed) == 1 else "parallel_fleet"
                scenario_makespan = max(elapsed_with_swaps)
                if scenario_kind not in scenario_best or scenario_makespan < scenario_best[scenario_kind][0]:
                    scenario_best[scenario_kind] = (scenario_makespan, result)
                if best_key is None or key < best_key:
                    best = result
                    best_key = key
        if evaluated and time.monotonic() - started >= deadline_s:
            break
        # A large area needs a bounded answer more than more sweep-angle trials.
        if best and best["metrics"]["total_swaths"] > 60 and angle_index > 0:
            break

    if best is None:
        return {"error": errors[0] if errors else "Не найден допустимый маршрут"}
    best["optimization_criterion"] = criterion
    if "single_drone" in scenario_best and "parallel_fleet" in scenario_best:
        scenarios = {}
        for kind, (duration_s, result) in scenario_best.items():
            count = result["metrics"]["active_drones_count"]
            sorties = len(result["drone_plans"])
            scenarios[kind] = {
                "name": "1 борт" if count == 1 else f"Параллельный флот ({count} БВС)",
                "drones_count": count,
                "makespan_min": round(duration_s / 60.0, 1),
                "battery_swaps": sorties - count,
                "to_wear_score": f"{sorties} вылетов",
            }
        best["feasibility"]["fleet_scenarios"] = scenarios
    best["search"] = {
        "method": "bounded_full_route_comparison",
        "evaluated_candidates": evaluated,
        "rejected_candidates": rejected,
        "optimality_proven": False,
        "objective_seconds": round(best_key[0], 1),
    }
    return best
