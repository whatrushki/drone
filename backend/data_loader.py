import os
import re
import xml.etree.ElementTree as ET
from typing import List, Dict, Any, Optional, Tuple
from shapely.geometry import Polygon, MultiPolygon, Point, shape
from shapely.strtree import STRtree
from shapely.ops import transform
from pyproj import Transformer
import json

DATA_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "data")

def parse_altitude_text(text: str) -> Dict[str, Any]:
    """
    Разработанный нами стандартный парсер неструктурированных высотных данных
    в соответствии со стандартом 4D-Airspace (ответ на прямой запрос Геоскана).
    """
    if not text:
        return {"min_alt_m": 0.0, "max_alt_m": 10000.0, "type": "UNSPECIFIED", "raw": text}
    
    text_clean = text.replace('\xa0', ' ').strip()
    
    # 1. Поиск эшелонов полета FLxxx (Flight Level)
    # Пример: "От FL280 до FL400" или "до FL090"
    fl_matches = re.findall(r'FL\s*(\d+)', text_clean, re.IGNORECASE)
    
    # 2. Поиск высот в метрах
    # Пример: "От земли до 700 м (2300 фут) AMSL" или "От 800 м"
    m_matches = re.findall(r'(\d+)\s*(?:м|m)', text_clean, re.IGNORECASE)
    
    min_alt = 0.0
    max_alt = 12000.0
    ref_type = "AMSL"
    
    if "от земли" in text_clean.lower() or "gnd" in text_clean.lower():
        min_alt = 0.0
        ref_type = "AGL"
    
    if fl_matches:
        fl_values = [float(fl) * 100 * 0.3048 for fl in fl_matches]
        if len(fl_values) == 1:
            if "до" in text_clean.lower():
                max_alt = fl_values[0]
            else:
                min_alt = fl_values[0]
        elif len(fl_values) >= 2:
            min_alt = min(fl_values)
            max_alt = max(fl_values)
    elif m_matches:
        m_values = [float(m) for m in m_matches]
        if len(m_values) == 1:
            if "до" in text_clean.lower():
                max_alt = m_values[0]
            else:
                min_alt = m_values[0]
        elif len(m_values) >= 2:
            min_alt = min(m_values)
            max_alt = max(m_values)
            
    return {
        "min_alt_m": round(min_alt, 1),
        "max_alt_m": round(max_alt, 1),
        "reference": ref_type,
        "raw_text": text_clean
    }

class HackathonDataLoader:
    def __init__(self, data_dir: str = DATA_DIR):
        self.data_dir = data_dir
        self.survey_polygons: List[Dict[str, Any]] = []
        self.airspace_zones: List[Dict[str, Any]] = []
        self.obstacles: List[Dict[str, Any]] = []
        
        self._survey_tree: Optional[STRtree] = None
        self._zone_tree: Optional[STRtree] = None
        self._obstacle_tree: Optional[STRtree] = None
        
        self.is_loaded = False

    def load_all(self):
        if self.is_loaded:
            return
        print("Loading hackathon datasets...")
        self._load_survey_polygons()
        self._load_airspace_zones()
        self._load_obstacles()
        self.is_loaded = True
        print(f"Data successfully loaded: {len(self.survey_polygons)} survey parcels, {len(self.airspace_zones)} airspace zones, {len(self.obstacles)} obstacles.")

    def _load_survey_polygons(self):
        filepath = os.path.join(self.data_dir, "Границы полетов.kml")
        if not os.path.exists(filepath):
            print(f"Warning: {filepath} not found")
            return
            
        tree = ET.parse(filepath)
        root = tree.getroot()
        for elem in root.iter():
            if '}' in elem.tag:
                elem.tag = elem.tag.split('}', 1)[1]
                
        geometries = []
        for pm in root.findall('.//Placemark'):
            fid_node = pm.find(".//SimpleData[@name='FID']")
            fid = int(fid_node.text) if fid_node is not None and fid_node.text else len(self.survey_polygons) + 1
            
            coord_node = pm.find('.//coordinates')
            if coord_node is None or not coord_node.text:
                continue
                
            pts = []
            for token in coord_node.text.strip().split():
                parts = [p.strip() for p in token.split(',') if p.strip()]
                if len(parts) >= 2:
                    try:
                        pts.append((float(parts[0]), float(parts[1])))
                    except ValueError:
                        continue
            
            if len(pts) >= 3:
                poly = Polygon(pts)
                if not poly.is_valid:
                    poly = poly.buffer(0)
                if poly.is_valid and not poly.is_empty:
                    lon, lat = poly.centroid.x, poly.centroid.y
                    zone = max(1, min(60, int((lon + 180) / 6) + 1))
                    epsg = (32600 if lat >= 0 else 32700) + zone
                    to_utm = Transformer.from_crs("EPSG:4326", f"EPSG:{epsg}", always_xy=True).transform
                    item = {
                        "fid": fid,
                        "geometry": poly,
                        "centroid": (poly.centroid.x, poly.centroid.y),
                        "bounds": poly.bounds,
                        "area_m2": round(transform(to_utm, poly).area, 1)
                    }
                    self.survey_polygons.append(item)
                    geometries.append(poly)
                
        if geometries:
            self._survey_tree = STRtree(geometries)

    def _load_airspace_zones(self):
        filepath = os.path.join(self.data_dir, "Московская зона.kml")
        if not os.path.exists(filepath):
            print(f"Warning: {filepath} not found")
            return
            
        tree = ET.parse(filepath)
        root = tree.getroot()
        for elem in root.iter():
            if '}' in elem.tag:
                elem.tag = elem.tag.split('}', 1)[1]
                
        geometries = []
        for pm in root.findall('.//Placemark'):
            name = pm.findtext('name') or "Unknown Zone"
            ext = pm.find('ExtendedData')
            d_dict = {}
            if ext is not None:
                d_dict = {d.attrib.get('name'): d.findtext('value') for d in ext.findall('.//Data')}
                
            zone_type = d_dict.get('Type', 'zone')
            raw_altitudes = d_dict.get('Altitudes', '')
            alt_info = parse_altitude_text(raw_altitudes)
            
            coord_nodes = pm.findall('.//coordinates')
            polys = []
            for c_node in coord_nodes:
                if not c_node.text:
                    continue
                pts = []
                for token in c_node.text.strip().split():
                    parts = [p.strip() for p in token.split(',') if p.strip()]
                    if len(parts) >= 2:
                        try:
                            pts.append((float(parts[0]), float(parts[1])))
                        except ValueError:
                            continue
                if len(pts) >= 3:
                    p = Polygon(pts)
                    if p.is_valid:
                        polys.append(p)
                    else:
                        p_fixed = p.buffer(0)
                        if p_fixed.is_valid:
                            polys.append(p_fixed)
                            
            if polys:
                geom = polys[0] if len(polys) == 1 else MultiPolygon(polys)
                item = {
                    "name": name,
                    "type": zone_type,
                    "geometry": geom,
                    "altitude_info": alt_info,
                    "bounds": geom.bounds
                }
                self.airspace_zones.append(item)
                geometries.append(geom)
                
        if geometries:
            self._zone_tree = STRtree(geometries)

    def _load_obstacles(self):
        obstacle_filenames = [
            "obstacles_Московская область.kml",
            "высотные препятствия Приморский край.kml"
        ]
        
        geometries = []
        for fname in obstacle_filenames:
            filepath = os.path.join(self.data_dir, fname)
            if not os.path.exists(filepath):
                continue
                
            try:
                tree = ET.parse(filepath)
                root = tree.getroot()
                for elem in root.iter():
                    if '}' in elem.tag:
                        elem.tag = elem.tag.split('}', 1)[1]
                        
                for pm in root.findall('.//Placemark'):
                    name = pm.findtext('name') or "Tower"
                    c_node = pm.find('.//coordinates')
                    if c_node is None or not c_node.text:
                        continue
                        
                    pts = []
                    heights = []
                    for token in c_node.text.strip().split():
                        parts = [p.strip() for p in token.split(',') if p.strip()]
                        if len(parts) >= 2:
                            try:
                                lon = float(parts[0])
                                lat = float(parts[1])
                                pts.append((lon, lat))
                                if len(parts) >= 3:
                                    heights.append(float(parts[2]))
                            except ValueError:
                                continue
                                    
                    if pts:
                        height_m = max(heights) if heights else 50.0
                        poly = Polygon(pts) if len(pts) >= 3 else Point(pts[0]).buffer(0.0002) # ~20m буфер
                        item = {
                            "name": name,
                            "height_m": round(height_m, 1),
                            "geometry": poly,
                            "centroid": (poly.centroid.x, poly.centroid.y),
                            "source_file": fname
                        }
                        self.obstacles.append(item)
                        geometries.append(poly)
            except Exception as e:
                print(f"Error loading obstacles from {fname}: {e}")
                
        if geometries:
            self._obstacle_tree = STRtree(geometries)

    def get_obstacles_near(self, bounds: Tuple[float, float, float, float], buffer_deg: float = 0.05) -> List[Dict[str, Any]]:
        """Быстрый R-Tree поиск препятствий вокруг рабочей области за <1мс"""
        if not self._obstacle_tree:
            return []
        minx, miny, maxx, maxy = bounds
        query_poly = Polygon([
            (minx - buffer_deg, miny - buffer_deg),
            (maxx + buffer_deg, miny - buffer_deg),
            (maxx + buffer_deg, maxy + buffer_deg),
            (minx - buffer_deg, maxy + buffer_deg)
        ])
        indices = self._obstacle_tree.query(query_poly)
        return [self.obstacles[idx] for idx in indices]

    def get_zones_near(self, bounds: Tuple[float, float, float, float], buffer_deg: float = 0.05) -> List[Dict[str, Any]]:
        """Быстрый R-Tree поиск зон ограничений вокруг рабочей области"""
        if not self._zone_tree:
            return []
        minx, miny, maxx, maxy = bounds
        query_poly = Polygon([
            (minx - buffer_deg, miny - buffer_deg),
            (maxx + buffer_deg, miny - buffer_deg),
            (maxx + buffer_deg, maxy + buffer_deg),
            (minx - buffer_deg, maxy + buffer_deg)
        ])
        indices = self._zone_tree.query(query_poly)
        return [self.airspace_zones[idx] for idx in indices]

data_loader = HackathonDataLoader()
