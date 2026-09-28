import math
from typing import Tuple, Dict

def solve_wind_triangle(
    heading_deg: float,      # Желаемый путевой угол движения (0=Север, 90=Восток)
    airspeed_ms: float,      # Воздушная скорость БВС (м/с)
    wind_speed_ms: float,    # Скорость ветра (м/с)
    wind_from_deg: float     # Откуда дует ветер (метеорологический азимут)
) -> Dict[str, float]:
    """
    Решает навигационный треугольник скоростей (Triangle of Velocities):
    - Вычисляет угол упреждения сноса (WCA - Wind Correction Angle / Crab angle)
    - Вычисляет истинный курс БВС (True Heading)
    - Вычисляет путевую скорость относительно земли (Ground Speed)
    """
    if airspeed_ms <= 0:
        return {"ground_speed_ms": 0.0, "wca_deg": 0.0, "true_heading_deg": heading_deg, "feasible": False}

    # Направление, КУДА дует ветер (wind_to_deg)
    wind_to_deg = (wind_from_deg + 180.0) % 360.0
    
    # Угол между путевым углом и направлением ветра
    track_rad = math.radians(heading_deg)
    wind_rad = math.radians(wind_to_deg)
    
    # Вектор ветра
    wx = wind_speed_ms * math.sin(wind_rad)
    wy = wind_speed_ms * math.cos(wind_rad)
    
    # Разложение ветра на путевую линию:
    # w_cross: боковая составляющая (снос), w_head: продольная составляющая
    angle_diff = wind_rad - track_rad
    w_cross = wind_speed_ms * math.sin(angle_diff)
    w_along = wind_speed_ms * math.cos(angle_diff)
    
    # Проверка на сдувание (если боковой ветер больше воздушной скорости)
    if abs(w_cross) >= airspeed_ms:
        # Невозможно компенсировать снос
        return {
            "ground_speed_ms": 0.1,
            "wca_deg": 90.0 if w_cross > 0 else -90.0,
            "true_heading_deg": heading_deg,
            "feasible": False
        }
        
    # Угол упреждения сноса: sin(WCA) = -w_cross / airspeed
    wca_rad = math.asin(-w_cross / airspeed_ms)
    wca_deg = math.degrees(wca_rad)
    
    # Путевая скорость: проекция воздушной скорости + продольный ветер
    ground_speed_ms = airspeed_ms * math.cos(wca_rad) + w_along
    ground_speed_ms = max(0.5, ground_speed_ms)
    
    true_heading_deg = (heading_deg + wca_deg) % 360.0
    
    return {
        "ground_speed_ms": round(ground_speed_ms, 2),
        "wca_deg": round(wca_deg, 2),
        "true_heading_deg": round(true_heading_deg, 1),
        "feasible": True
    }

def calculate_flight_leg_utm(
    p1_utm: Tuple[float, float],
    p2_utm: Tuple[float, float],
    airspeed_ms: float,
    wind_speed_ms: float,
    wind_from_deg: float,
    drone_type: str = "fixed_wing",
    max_flight_time_min: float = 180.0
) -> Dict[str, float]:
    """
    Рассчитывает параметры перелета по отрезку p1 -> p2 в метрических координатах UTM с учетом ветра.
    Исключает сферические искажения и погрешности приближенного пересчета.
    """
    dx = p2_utm[0] - p1_utm[0]
    dy = p2_utm[1] - p1_utm[1]
    distance_m = math.hypot(dx, dy)
    if distance_m < 0.1:
        return {
            "distance_m": 0.0,
            "heading_deg": 0.0,
            "ground_speed_ms": airspeed_ms,
            "wca_deg": 0.0,
            "time_s": 0.0,
            "battery_used_pct": 0.0
        }
        
    heading_deg = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
    wt = solve_wind_triangle(heading_deg, airspeed_ms, wind_speed_ms, wind_from_deg)
    v_ground = wt["ground_speed_ms"]
    time_s = distance_m / v_ground
    
    # Физически корректный расход батареи:
    # Время time_s уже отражает путевую скорость v_ground (увеличивается против ветра, уменьшается по ветру).
    burn_rate_pct_per_min = 100.0 / max(1.0, max_flight_time_min)
    
    # Небольшая аэродинамическая поправка на угол упреждения сноса WCA и балансировку
    aero_drag_factor = 1.0 + (abs(wt["wca_deg"]) / 90.0) * 0.08
    if drone_type == "multirotor":
        # У мультиротора сопротивление ветру требует дополнительного наклона
        aero_drag_factor += min(0.2, (wind_speed_ms / 20.0) * 0.15)
        
    battery_used_pct = (time_s / 60.0) * burn_rate_pct_per_min * aero_drag_factor
    
    return {
        "distance_m": round(distance_m, 1),
        "heading_deg": round(heading_deg, 1),
        "ground_speed_ms": v_ground,
        "wca_deg": wt["wca_deg"],
        "time_s": round(time_s, 1),
        "battery_used_pct": round(battery_used_pct, 2)
    }

def calculate_flight_leg(
    p1: Tuple[float, float], # (lon, lat)
    p2: Tuple[float, float], # (lon, lat)
    airspeed_ms: float,
    wind_speed_ms: float,
    wind_from_deg: float,
    drone_type: str = "fixed_wing",
    max_flight_time_min: float = 180.0
) -> Dict[str, float]:
    """
    Рассчитывает параметры перелета по отрезку p1 -> p2 (WGS84 градусы) с учетом ветра
    """
    mid_lat = math.radians((p1[1] + p2[1]) / 2.0)
    m_per_deg_lat = 111132.954 - 559.822 * math.cos(2 * mid_lat)
    m_per_deg_lon = 111412.84 * math.cos(mid_lat)
    
    dx = (p2[0] - p1[0]) * m_per_deg_lon
    dy = (p2[1] - p1[1]) * m_per_deg_lat
    
    return calculate_flight_leg_utm((0.0, 0.0), (dx, dy), airspeed_ms, wind_speed_ms, wind_from_deg, drone_type, max_flight_time_min)

