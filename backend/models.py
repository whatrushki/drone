from typing import List, Dict, Optional, Literal, Tuple
from pydantic import BaseModel, Field

class DroneSpec(BaseModel):
    id: str
    name: str
    type: Literal["fixed_wing", "multirotor"]
    cruise_speed: float
    max_speed: float
    min_speed: float
    max_flight_time_min: float
    battery_level: float = 100.0
    reserve_battery_pct: float = 20.0
    turn_radius: float
    max_wind_resistance: float
    takeoff_type: Literal["catapult", "vtol"]
    landing_type: Literal["parachute", "vtol"]
    min_operational_height_m: float = 80.0 # Минимальная безопасная высота полета (для крыла 90м, для коптеров 25м)
    min_swath_spacing_m: float = 55.0      # Минимальный допустимый шаг галсов
    maintenance_flight_limit: int = 80     # Норматив ТО (Геоскан 201: каждые 80 полетов)
    sensors_supported: List[str]

GEOSCAN_DRONE_CATALOG: Dict[str, DroneSpec] = {
    "geoscan_201": DroneSpec(
        id="geoscan_201",
        name="Геоскан 201 (Самолетного типа)",
        type="fixed_wing",
        cruise_speed=21.0,
        max_speed=28.0,
        min_speed=16.0,
        max_flight_time_min=180.0,
        battery_level=100.0,
        reserve_battery_pct=20.0,
        turn_radius=85.0,
        max_wind_resistance=15.0,
        takeoff_type="catapult",
        landing_type="parachute",
        min_operational_height_m=90.0, # Самолет не может безопасно крутить виражи ниже 90м над лесом/застройкой
        min_swath_spacing_m=55.0,
        maintenance_flight_limit=80,
        sensors_supported=["sony_rx1r2", "sony_a6000", "multispectral", "thermal"]
    ),
    "geoscan_801": DroneSpec(
        id="geoscan_801",
        name="Геоскан 801 (Тяжелый мультиротор)",
        type="multirotor",
        cruise_speed=12.0,
        max_speed=16.0,
        min_speed=0.0,
        max_flight_time_min=40.0,
        battery_level=100.0,
        reserve_battery_pct=25.0,
        turn_radius=3.0,
        max_wind_resistance=12.0,
        takeoff_type="vtol",
        landing_type="vtol",
        min_operational_height_m=25.0, # Специализация: бреющий полет для LiDAR и магнитометрии
        min_swath_spacing_m=15.0,
        maintenance_flight_limit=160,
        sensors_supported=["lidar_agm_ms3", "geophysics_mag", "sony_a6000", "thermal"]
    ),
    "geoscan_gemini": DroneSpec(
        id="geoscan_gemini",
        name="Геоскан Gemini (Геодезический квадрокоптер)",
        type="multirotor",
        cruise_speed=14.0,
        max_speed=18.0,
        min_speed=0.0,
        max_flight_time_min=45.0,
        battery_level=100.0,
        reserve_battery_pct=20.0,
        turn_radius=2.0,
        max_wind_resistance=12.0,
        takeoff_type="vtol",
        landing_type="vtol",
        min_operational_height_m=30.0, # Детальная АФС на малых и средних высотах
        min_swath_spacing_m=20.0,
        maintenance_flight_limit=120,
        sensors_supported=["sony_rx1r2", "sony_a6000", "multispectral"]
    )
}

class SensorSpec(BaseModel):
    id: str
    name: str
    type: Literal["RGB", "мультиспектральная", "ИК", "LiDAR", "геофизическая"]
    sensor_width_mm: float   # ширина матрицы (мм)
    sensor_height_mm: float  # высота матрицы (мм)
    focal_length_mm: float   # фокусное расстояние объектива (мм)
    resolution_w_px: int     # разрешение матрицы по ширине
    resolution_h_px: int     # разрешение матрицы по высоте
    min_trigger_interval_s: float = 1.0 # интервал между кадрами (с)
    default_overlap_forward: float = 0.75 # продольное перекрытие 75%
    default_overlap_side: float = 0.65    # поперечное перекрытие 65%

SENSOR_CATALOG: Dict[str, SensorSpec] = {
    "sony_rx1r2": SensorSpec(
        id="sony_rx1r2",
        name="Sony RX1R II (42.4 Мп полнокадровая)",
        type="RGB",
        sensor_width_mm=35.9,
        sensor_height_mm=24.0,
        focal_length_mm=35.0,
        resolution_w_px=7952,
        resolution_h_px=5304,
        min_trigger_interval_s=1.2,
        default_overlap_forward=0.75,
        default_overlap_side=0.65
    ),
    "sony_a6000": SensorSpec(
        id="sony_a6000",
        name="Sony A6000 (24.3 Мп APS-C)",
        type="RGB",
        sensor_width_mm=23.5,
        sensor_height_mm=15.6,
        focal_length_mm=20.0,
        resolution_w_px=6000,
        resolution_h_px=4000,
        min_trigger_interval_s=1.0,
        default_overlap_forward=0.75,
        default_overlap_side=0.65
    ),
    "lidar_agm_ms3": SensorSpec(
        id="lidar_agm_ms3",
        name="LiDAR АГМ-МС3 (Лазерный сканер)",
        type="LiDAR",
        sensor_width_mm=30.0,
        sensor_height_mm=30.0,
        focal_length_mm=25.0,
        resolution_w_px=5000,
        resolution_h_px=5000,
        min_trigger_interval_s=0.1,
        default_overlap_forward=0.50,
        default_overlap_side=0.30
    ),
    "multispectral": SensorSpec(
        id="multispectral",
        name="MicaSense RedEdge-P (Мультиспектр 5-канальный)",
        type="мультиспектральная",
        sensor_width_mm=5.8,
        sensor_height_mm=4.4,
        focal_length_mm=5.4,
        resolution_w_px=2048,
        resolution_h_px=1536,
        min_trigger_interval_s=1.0,
        default_overlap_forward=0.80,
        default_overlap_side=0.70
    ),
    "thermal": SensorSpec(
        id="thermal",
        name="FLIR Vue Pro R (Тепловизор 640x512)",
        type="ИК",
        sensor_width_mm=10.8,
        sensor_height_mm=8.7,
        focal_length_mm=13.0,
        resolution_w_px=640,
        resolution_h_px=512,
        min_trigger_interval_s=0.5,
        default_overlap_forward=0.80,
        default_overlap_side=0.70
    ),
    "geophysics_mag": SensorSpec(
        id="geophysics_mag",
        name="Квантовый магнитометр (Геофизика)",
        type="геофизическая",
        sensor_width_mm=20.0,
        sensor_height_mm=20.0,
        focal_length_mm=20.0,
        resolution_w_px=1000,
        resolution_h_px=1000,
        min_trigger_interval_s=0.2,
        default_overlap_forward=0.50,
        default_overlap_side=0.40
    )
}

class WindConfig(BaseModel):
    speed_ms: float = 5.0
    direction_deg: float = 90.0

class LaunchPoint(BaseModel):
    id: str
    name: str
    lat: float
    lon: float
    alt_m: float = 0.0
    type: Literal["base", "emergency_pad"] = "base"

class MissionRequest(BaseModel):
    polygon_geojson: dict
    selected_fid: Optional[int] = None
    sensor_id: str = "sony_rx1r2"
    sensor_ids: Optional[List[str]] = None # Поддержка нескольких сенсоров для комбинированной съемки (RGB + LiDAR)
    target_gsd_cm: float = 3.0
    overlap_forward: Optional[float] = None
    overlap_side: Optional[float] = None
    available_drones: List[str] = ["geoscan_201", "geoscan_gemini"]
    launch_points: Optional[List[LaunchPoint]] = None
    wind: WindConfig = WindConfig()
    sweep_angle_deg: Optional[float] = None # Ручной или авто выбор угла галсов (по умолчанию по длинной оси)
    optimization_criterion: Literal["min_makespan", "min_flight_time"] = "min_makespan"
    avoid_nfz: bool = True
    avoid_obstacles: bool = True
    flight_time_utc: Optional[str] = "2026-05-15T10:00:00Z"
    # Операционные ограничения и ТЭО (ответ на требования экспертов Геоскана)
    max_allowed_time_min: Optional[float] = None # Лимит времени на миссию (мин)
    max_available_drones: Optional[int] = None # Максимально допустимое кол-во активных бортов
    battery_swap_penalty_min: float = 15.0 # Время на наземное обслуживание и смену АКБ (мин)
