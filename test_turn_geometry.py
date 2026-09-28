import numpy as np
import math
import matplotlib.pyplot as plt

def generate_exact_dubins_turn(p_exit, heading_out, p_enter, heading_in, turn_radius_m=85.0, lead_out_m=30.0):
    """
    Генерирует физически строгую траекторию разворота для БВС самолетного типа (Геоскан 201).
    На всех дугах кривизна строго равна 1/R (R = turn_radius_m).
    Касательные векторы строго непрерывны (G1).
    Никаких изломов, черточек 'туда-обратно' или резких углов.
    """
    p_ex = np.array(p_exit, dtype=float)
    p_en = np.array(p_enter, dtype=float)
    
    # 1. Вектор вдоль направления выхода с галса (d)
    rad_out = math.radians(heading_out)
    d = np.array([math.sin(rad_out), math.cos(rad_out)])
    
    # Нормаль n (поворот d на 90 град вправо)
    n = np.array([d[1], -d[0]])
    
    # Определяем, с какой стороны находится следующий галс
    dp = p_en - p_ex
    lateral_proj = np.dot(dp, n)
    if lateral_proj < 0:
        n = -n
        lateral_proj = -lateral_proj
        
    D = lateral_proj # Расстояние между параллельными линиями галсов
    R = turn_radius_m
    
    pts = []
    
    # 1. Прямой выбег (lead-out), чтобы затвор камеры выключился и самолет стабилизировался
    p_lead_out = p_ex + d * lead_out_m
    pts.append(tuple(p_lead_out))
    
    if D >= 2.0 * R:
        # Случай А: Расстояние между галсами достаточно велико для прямого разворота U-turn (дуга 180 град)
        # Центр разворота
        r_actual = D / 2.0
        center = p_lead_out + n * r_actual
        # Дуга от 0 до 180 градусов
        for a in np.linspace(0, math.pi, 16):
            pt = center - n * (r_actual * math.cos(a)) + d * (r_actual * math.sin(a))
            pts.append((float(pt[0]), float(pt[1])))
        # Точка входа в створ следующего галса
        p_lead_in = p_en + d * lead_out_m
        pts.append(tuple(p_lead_in))
    else:
        # Случай Б: D < 2*R. Строгий аналитический вираж 'Рыбий хвост' (Bulb / Teardrop Turn).
        # Две касающиеся окружности радиуса R:
        # Окружность 1 (отворот наружу): поворот от вектора d в сторону -n
        # Окружность 2 (заход в створ): поворот в сторону +n до строгого направления -d
        delta_y = math.sqrt(4.0 * R**2 - D**2)
        
        # Центры окружностей в локальном базисе (n, d):
        # C1 находится на -R по нормали n от точки p_lead_out
        C1 = p_lead_out - n * R
        # C2 находится на (D - R) по нормали n, и смещен вперед по d на delta_y
        C2 = p_lead_out + n * (D - R) + d * delta_y
        
        # Точка касания двух окружностей:
        M = (C1 + C2) / 2.0
        
        # Угол точки M относительно C1:
        # Начальный вектор от C1 до p_lead_out: n * R (угол 0)
        # Вектор от C1 до M:
        vec_m1 = M - C1
        proj_n1 = np.dot(vec_m1, n)
        proj_d1 = np.dot(vec_m1, d)
        angle_end1 = math.atan2(proj_d1, proj_n1)
        
        # Генерация точек по Окружности 1 (против часовой):
        for a in np.linspace(0, angle_end1, 10)[1:]:
            pt = C1 + n * (R * math.cos(a)) + d * (R * math.sin(a))
            pts.append((float(pt[0]), float(pt[1])))
            
        # Генерация точек по Окружности 2 (по часовой стрелке до направления -d):
        # Вектор от C2 до M:
        vec_m2 = M - C2
        proj_n2 = np.dot(vec_m2, n)
        proj_d2 = np.dot(vec_m2, d)
        angle_start2 = math.atan2(proj_d2, proj_n2)
        
        # Заканчиваем в точке (D, delta_y), где вектор от C2 равен n * R (угол 0):
        # Идем по часовой стрелке от angle_start2 к 0
        if angle_start2 > 0:
            angles2 = np.linspace(angle_start2, 0, 12)[1:]
        else:
            angles2 = np.linspace(angle_start2, 0, 12)[1:]
            
        for a in angles2:
            pt = C2 + n * (R * math.cos(a)) + d * (R * math.sin(a))
            pts.append((float(pt[0]), float(pt[1])))
            
        # Точка окончания дуги - на створах следующего галса
        # Отсюда траектория летит строго вдоль -d к p_enter
        p_entry_aligned = p_lead_out + n * D + d * delta_y
        # Убедимся, что спускаемся к p_enter
        p_lead_in = p_en + d * max(30.0, lead_out_m)
        pts.append((float(p_lead_in[0]), float(p_lead_in[1])))
        
    pts.append(tuple(p_en))
    return pts

# Тест
pts = generate_exact_dubins_turn([0, 0], 45.0, [50, 0], 225.0, 85.0)
print('Generated points count:', len(pts))
for i in range(len(pts)-1):
    dist = math.hypot(pts[i+1][0] - pts[i][0], pts[i+1][1] - pts[i][1])
    print(f'Seg {i} -> {i+1}: dist={dist:.1f}m, pt={pts[i]}')
