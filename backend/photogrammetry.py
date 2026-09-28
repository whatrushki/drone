import math
from typing import Dict, Any
from .models import SensorSpec

def calculate_photogrammetry(
    sensor: SensorSpec,
    target_gsd_cm: float,
    overlap_forward: float = None,
    overlap_side: float = None,
    max_shutter_speed_s: float = 1.0 / 1000.0 # типовая выдержка 1/1000 с
) -> Dict[str, Any]:
    """
    Рассчитывает фотограмметрические параметры полета БВС под оптику/сенсор:
    - Рабочую высоту полета H (AGL)
    - Ширину захвата на местности (Footprint width)
    - Длину кадра на местности (Footprint length)
    - Межгалсовое расстояние (Line spacing / Step)
    - Базис съемки (расстояние между срабатываниями затвора)
    - Максимально допустимую путевую скорость во избежание смаза (Motion Blur)
    """
    p_forward = overlap_forward if overlap_forward is not None else sensor.default_overlap_forward
    p_side = overlap_side if overlap_side is not None else sensor.default_overlap_side
    
    # Размер одного физического пикселя на матрице (в мм)
    pixel_size_mm = sensor.sensor_width_mm / sensor.resolution_w_px
    
    # Целевой GSD в метрах (например, 3.0 см -> 0.03 м)
    gsd_m = target_gsd_cm / 100.0
    
    # Высота полета над поверхностью: H = (F_mm * GSD_m) / pixel_size_mm
    flight_height_m = (sensor.focal_length_mm * gsd_m) / (pixel_size_mm / 1000.0) / 1000.0
    
    # Ограничения по минимальной и максимальной рабочей высоте для БАС (обычно от 50 до 500 м)
    flight_height_m = max(30.0, min(1000.0, flight_height_m))
    
    # Фактический размер кадра на местности (Footprint) в метрах
    footprint_w_m = (sensor.sensor_width_mm * flight_height_m) / sensor.focal_length_mm
    footprint_h_m = (sensor.sensor_height_mm * flight_height_m) / sensor.focal_length_mm
    
    # Межгалсовый шаг (расстояние между соседними параллельными линиями)
    line_spacing_m = footprint_w_m * (1.0 - p_side)
    
    # Базис фотографирования (дистанция между центрами смежных снимков вдоль галса)
    trigger_dist_m = footprint_h_m * (1.0 - p_forward)
    
    # Максимальная скорость против смаза: смещение за время экспозиции не более 0.5 GSD
    max_speed_anti_blur_ms = (gsd_m * 0.5) / max_shutter_speed_s
    
    return {
        "sensor_id": sensor.id,
        "sensor_name": sensor.name,
        "flight_height_m": round(flight_height_m, 1),
        "target_gsd_cm": round(target_gsd_cm, 2),
        "footprint_w_m": round(footprint_w_m, 1),
        "footprint_h_m": round(footprint_h_m, 1),
        "line_spacing_m": round(line_spacing_m, 1),
        "trigger_dist_m": round(trigger_dist_m, 1),
        "overlap_forward_pct": round(p_forward * 100, 1),
        "overlap_side_pct": round(p_side * 100, 1),
        "max_speed_anti_blur_ms": round(max_speed_anti_blur_ms, 1)
    }
