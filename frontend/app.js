// GEOSCAN FleetCommander AI - Frontend Application
// Dialog Design System Light Theme
const API_BASE = "http://localhost:8000";

let map;
let baseLayers = {};
let currentParcelLayer = null;
let currentSwathLayers = [];
let currentZoneLayer = null;
let currentObstacleLayer = null;
let drawnItems = null;

let currentMissionPlan = null;
let currentParcelGeoJSON = null;
let currentAngleMode = "auto"; // 'auto' | 'axis' | 'wind' | 'manual'

// Матрица совместимости сенсоров с моделями БВС Геоскан
const SENSOR_DRONE_COMPATIBILITY = {
  "sony_rx1r2": ["geoscan_201", "geoscan_gemini"],
  "sony_a6000": ["geoscan_201", "geoscan_gemini", "geoscan_801"],
  "lidar_agm_ms3": ["geoscan_801"],
  "multispectral": ["geoscan_201", "geoscan_gemini"],
  "thermal": ["geoscan_201", "geoscan_801"],
  "geophysics_mag": ["geoscan_801"],
  "rgb_and_lidar": ["geoscan_201", "geoscan_801"]
};

// Симуляция полета
let isSimPlaying = false;
let simInterval = null;
let simCurrentSeconds = 0;
let simMaxSeconds = 0;
let simDroneMarkers = {};

document.addEventListener("DOMContentLoaded", () => {
  initMap();
  initTabsAndPanel();
  initEventListeners();
  loadParcelsList();
  filterDronesBySensor();
  updatePhotogrammetryPreview();
});

// 1. ИНИЦИАЛИЗАЦИЯ СВЕТЛОЙ КАРТЫ (DIALOG AESTHETIC)
function initMap() {
  map = L.map("map", {
    center: [54.85, 38.65],
    zoom: 12,
    zoomControl: false
  });

  L.control.zoom({ position: "topright" }).addTo(map);

  // Светлая минималистичная подложка CartoDB Positron - идеально сочетается с #f7f7f7 и #ffffff
  const positronTiles = L.tileLayer("https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png", {
    attribution: "&copy; OpenStreetMap &copy; CARTO",
    maxZoom: 19
  });

  const satTiles = L.tileLayer("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}", {
    attribution: "&copy; Esri World Imagery",
    maxZoom: 19
  });

  const osmTiles = L.tileLayer("https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png", {
    attribution: "&copy; OpenStreetMap",
    maxZoom: 19
  });

  positronTiles.addTo(map);

  baseLayers = {
    "☀️ Светлая карта (CARTO Positron)": positronTiles,
    "🛰️ Спутник (Esri Satellite)": satTiles,
    "🗺️ OpenStreetMap": osmTiles
  };

  L.control.layers(baseLayers, null, { position: "topright" }).addTo(map);

  // Слой для рисования произвольного полигона
  drawnItems = new L.FeatureGroup();
  map.addLayer(drawnItems);

  const drawControl = new L.Control.Draw({
    position: "topright",
    draw: {
      polygon: {
        allowIntersection: false,
        showArea: true,
        shapeOptions: { color: "#f69251", weight: 2, fillOpacity: 0.15 }
      },
      polyline: false,
      rectangle: {
        shapeOptions: { color: "#f69251", weight: 2, fillOpacity: 0.15 }
      },
      circle: false,
      circlemarker: false,
      marker: false
    },
    edit: { featureGroup: drawnItems }
  });
  map.addControl(drawControl);

  map.on(L.Draw.Event.CREATED, (e) => {
    clearMissionArtifacts();
    drawnItems.clearLayers();
    if (currentParcelLayer) map.removeLayer(currentParcelLayer);
    
    const layer = e.layer;
    drawnItems.addLayer(layer);
    currentParcelGeoJSON = layer.toGeoJSON();
    
    document.getElementById("parcelSelect").value = "";
    document.getElementById("parcelInfoHint").innerText = "Нарисован пользовательский полигон";
    
    loadNearbyRestrictions(layer.getBounds());
  });
}

// 2. ИНИЦИАЛИЗАЦИЯ ТАБОВ И УПРАВЛЕНИЯ ПАНЕЛЬЮ
function initTabsAndPanel() {
  // Переключение табов
  const tabBtns = document.querySelectorAll(".tab-btn");
  tabBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const targetId = btn.getAttribute("data-tab");
      switchTab(targetId);
    });
  });

  // Кнопки "Далее"
  const nextBtns = document.querySelectorAll(".next-tab-btn");
  nextBtns.forEach((btn) => {
    btn.addEventListener("click", () => {
      const targetId = btn.getAttribute("data-target");
      switchTab(targetId);
    });
  });

  // Сворачивание панели
  const toggleBtn = document.getElementById("btnTogglePanel");
  const panel = document.getElementById("controlPanel");
  const statsRibbon = document.getElementById("statsRibbon");

  toggleBtn.addEventListener("click", () => {
    const isCollapsed = panel.classList.toggle("collapsed");
    toggleBtn.innerText = isCollapsed ? "▶" : "◀";
    if (statsRibbon) {
      statsRibbon.style.left = isCollapsed ? "20px" : "460px";
    }
  });
}

function switchTab(tabId) {
  document.querySelectorAll(".tab-btn").forEach((b) => {
    b.classList.toggle("active", b.getAttribute("data-tab") === tabId);
  });
  document.querySelectorAll(".tab-content").forEach((c) => {
    c.classList.toggle("active", c.id === tabId);
  });
}

// 3. ИНИЦИАЛИЗАЦИЯ СЛУШАТЕЛЕЙ СОБЫТИЙ
function initEventListeners() {
  // Селектор участков
  document.getElementById("parcelSelect").addEventListener("change", (e) => {
    const fid = e.target.value;
    if (fid) selectParcel(parseInt(fid));
  });

  document.getElementById("btnRefreshParcels").addEventListener("click", loadParcelsList);

  // Сенсор и GSD
  document.getElementById("sensorSelect").addEventListener("change", updatePhotogrammetryPreview);
  document.getElementById("gsdSlider").addEventListener("input", (e) => {
    document.getElementById("gsdValue").innerText = `${parseFloat(e.target.value).toFixed(1)} см/пикс`;
    updatePhotogrammetryPreview();
  });

  // Ветер
  function updateWindUI() {
    const speedInput = document.getElementById("windSpeedSlider");
    const dirInput = document.getElementById("windDirSlider");
    if (!speedInput || !dirInput) return;

    const val = parseFloat(speedInput.value).toFixed(1);
    document.getElementById("windSpeedVal").innerText = `${val} м/с`;
    document.getElementById("mapWindSpeed").innerText = `${val} м/с`;

    const deg = parseInt(dirInput.value, 10);
    const compassNames = ["С", "СВ", "В", "ЮВ", "Ю", "ЮЗ", "З", "СЗ"];
    const name = compassNames[Math.round(deg / 45) % 8];
    document.getElementById("windDirVal").innerText = `${deg}° (${name})`;
    document.getElementById("mapWindDir").innerText = `${deg}° ${name}`;
    
    // Вектор движения воздушных масс (куда дует ветер: от deg в сторону (deg + 180))
    // Глиф ▲ изначально направлен на Север (0°), поэтому поворот на (deg + 180) точно показывает вектор потока
    const arrowRot = (deg + 180) % 360;
    const compassEl = document.getElementById("windCompassArrow");
    const mapArrowEl = document.getElementById("mapWindArrow");
    if (compassEl) compassEl.style.transform = `rotate(${arrowRot}deg)`;
    if (mapArrowEl) mapArrowEl.style.transform = `rotate(${arrowRot}deg)`;

    if (currentAngleMode === "wind") {
      const manSlider = document.getElementById("manualAngleSlider");
      if (manSlider) manSlider.value = deg % 180;
      const manVal = document.getElementById("manualAngleVal");
      if (manVal) manVal.innerText = `${deg % 180}°`;
    }
  }

  document.getElementById("windSpeedSlider").addEventListener("input", updateWindUI);
  document.getElementById("windDirSlider").addEventListener("input", updateWindUI);
  updateWindUI();

  // Управление ориентацией галсов
  const btnAuto = document.getElementById("btnAngleAuto");
  const btnAxis = document.getElementById("btnAngleAxis");
  const btnWind = document.getElementById("btnAngleWind");
  const btnManual = document.getElementById("btnAngleManual");
  const manualBlock = document.getElementById("manualAngleBlock");
  const manualSlider = document.getElementById("manualAngleSlider");
  const manualVal = document.getElementById("manualAngleVal");
  const angleBadge = document.getElementById("angleModeBadge");

  function setAngleMode(mode) {
    currentAngleMode = mode;
    if (btnAuto) btnAuto.classList.toggle("active", mode === "auto");
    if (btnAxis) btnAxis.classList.toggle("active", mode === "axis");
    if (btnWind) btnWind.classList.toggle("active", mode === "wind");
    if (btnManual) btnManual.classList.toggle("active", mode === "manual");
    if (manualBlock) manualBlock.style.display = mode === "manual" ? "block" : "none";
    if (angleBadge) {
      if (mode === "auto") angleBadge.innerText = "Авто (Оптимум)";
      else if (mode === "axis") angleBadge.innerText = "Вдоль поля";
      else if (mode === "wind") angleBadge.innerText = "По ветру";
      else if (mode === "manual") angleBadge.innerText = "Ручной угол";
    }
  }

  if (btnAuto) btnAuto.addEventListener("click", () => setAngleMode("auto"));
  if (btnAxis) btnAxis.addEventListener("click", () => setAngleMode("axis"));
  if (btnWind) btnWind.addEventListener("click", () => setAngleMode("wind"));
  if (btnManual) btnManual.addEventListener("click", () => setAngleMode("manual"));
  if (manualSlider) {
    manualSlider.addEventListener("input", (e) => {
      if (manualVal) manualVal.innerText = `${e.target.value}°`;
    });
  }

  // Расчет миссии
  document.getElementById("btnCalculate").addEventListener("click", calculateMission);
  const btnTab = document.getElementById("btnCalculateTab");
  if (btnTab) btnTab.addEventListener("click", calculateMission);

  // Экспорт
  document.getElementById("btnExportKml").addEventListener("click", exportKml);
  document.getElementById("btnExportGeoJson").addEventListener("click", exportGeoJson);
  document.getElementById("btnExportPlan").addEventListener("click", exportPlan);

  // Плеер
  document.getElementById("btnPlayPause").addEventListener("click", toggleSimulation);
  document.getElementById("btnResetTime").addEventListener("click", resetSimulation);
  document.getElementById("timeSlider").addEventListener("input", (e) => {
    simCurrentSeconds = (parseFloat(e.target.value) / 1000) * simMaxSeconds;
    updateSimulationFrame();
  });

  // Переключение слоев
  document.getElementById("toggleZones").addEventListener("change", (e) => {
    if (currentZoneLayer) {
      if (e.target.checked) map.addLayer(currentZoneLayer);
      else map.removeLayer(currentZoneLayer);
    }
  });

  document.getElementById("toggleObstacles").addEventListener("change", (e) => {
    if (currentObstacleLayer) {
      if (e.target.checked) map.addLayer(currentObstacleLayer);
      else map.removeLayer(currentObstacleLayer);
    }
  });

  // Показать все зоны
  document.getElementById("btnFitAll").addEventListener("click", () => {
    if (currentZoneLayer && currentZoneLayer.getLayers().length > 0) {
      map.fitBounds(currentZoneLayer.getBounds(), { padding: [50, 50] });
    } else {
      map.setView([54.85, 38.65], 11);
    }
  });
}

// 4. ЗАГРУЗКА СПИСКА УЧАСТКОВ
async function loadParcelsList() {
  try {
    const res = await fetch(`${API_BASE}/api/parcels?limit=100`);
    const data = await res.json();
    
    const select = document.getElementById("parcelSelect");
    select.innerHTML = '<option value="">-- Выберите полигон АФС --</option>';
    
    data.parcels.forEach((p) => {
      const opt = document.createElement("option");
      opt.value = p.fid;
      opt.text = `Участок FID ${p.fid} (${p.area_ha} га)`;
      select.appendChild(opt);
    });

    document.getElementById("datasetStats").innerText = `${data.total} полигонов • 341 зона`;

    if (data.parcels.length > 0) {
      select.value = data.parcels[0].fid;
      selectParcel(data.parcels[0].fid);
    }
  } catch (err) {
    console.error("Ошибка загрузки участков:", err);
  }
}

// 5. ВЫБОР УЧАСТКА
async function selectParcel(fid) {
  try {
    clearMissionArtifacts();
    const statsRibbon = document.getElementById("statsRibbon");
    if (statsRibbon) statsRibbon.style.display = "none";

    const res = await fetch(`${API_BASE}/api/parcels/${fid}`);
    const geojson = await res.json();
    currentParcelGeoJSON = geojson;

    if (currentParcelLayer) map.removeLayer(currentParcelLayer);
    if (drawnItems) drawnItems.clearLayers();

    currentParcelLayer = L.geoJSON(geojson, {
      style: {
        color: "#181825",
        weight: 2,
        fillColor: "#f69251",
        fillOpacity: 0.12,
        dashArray: "4, 4"
      }
    }).addTo(map);

    const bounds = currentParcelLayer.getBounds();
    map.fitBounds(bounds, { padding: [80, 80] });

    document.getElementById("parcelInfoHint").innerText = `Площадь: ${geojson.properties.area_ha} га | FID: ${fid}`;

    loadNearbyRestrictions(bounds);
  } catch (err) {
    console.error("Ошибка загрузки полигона:", err);
  }
}

// 6. ЗАГРУЗКА ЗОН И ПРЕПЯТСТВИЙ РЯДОМ С ПОЛИГОНОМ
async function loadNearbyRestrictions(bounds) {
  const minx = bounds.getWest() - 0.05;
  const miny = bounds.getSouth() - 0.05;
  const maxx = bounds.getEast() + 0.05;
  const maxy = bounds.getNorth() + 0.05;

  try {
    // 4D зоны ограничений
    const resZ = await fetch(`${API_BASE}/api/airspace-zones?minx=${minx}&miny=${miny}&maxx=${maxx}&maxy=${maxy}`);
    const dataZ = await resZ.json();

    if (currentZoneLayer) map.removeLayer(currentZoneLayer);
    currentZoneLayer = L.geoJSON(dataZ, {
      style: {
        color: "#c97b84",
        weight: 1.5,
        fillColor: "#c97b84",
        fillOpacity: 0.12
      },
      onEachFeature: (feature, layer) => {
        const p = feature.properties;
        const alt = p.altitude_info;
        layer.bindPopup(`
          <div style="font-size:12px; line-height:1.45; font-family:Inter, sans-serif;">
            <strong style="color:#c97b84;">⛔ ${p.name}</strong> (${p.type})<br>
            <b>Высотный диапазон:</b> от ${alt.min_alt_m}м до ${alt.max_alt_m}м (${alt.reference})<br>
            <small style="color:#8b8b8b;">Стандарт 4D Airspace</small>
          </div>
        `);
      }
    });

    if (document.getElementById("toggleZones").checked) {
      currentZoneLayer.addTo(map);
    }

    // 3D препятствия (вышки)
    const resO = await fetch(`${API_BASE}/api/obstacles?minx=${minx}&miny=${miny}&maxx=${maxx}&maxy=${maxy}`);
    const dataO = await resO.json();

    if (currentObstacleLayer) map.removeLayer(currentObstacleLayer);
    currentObstacleLayer = L.geoJSON(dataO, {
      pointToLayer: (feature, latlng) => {
        const p = feature.properties;
        const rad = p.safety_radius_m || 70;
        const group = L.layerGroup([
          L.circle(latlng, {
            radius: rad,
            color: "#ef233c",
            fillColor: "#ef233c",
            fillOpacity: 0.08,
            weight: 1,
            dashArray: "3, 3"
          }),
          L.circleMarker(latlng, {
            radius: 5,
            color: "#b91c1c",
            fillColor: "#f69251",
            fillOpacity: 0.9,
            weight: 2
          })
        ]);
        group.bindPopup(`
          <div style="font-size:12px; font-family:Inter, sans-serif; line-height:1.45;">
            <strong style="color:#b91c1c;">🗼 ${p.name}</strong><br>
            Высота препятствия: <b>${p.height_m} м AGL</b><br>
            Зона безопасности: <b>${rad} м</b><br>
            <small style="color:#64748b;">GSD-контроль: автоматический горизонтальный обход</small>
          </div>
        `);
        return group;
      }
    });

    if (document.getElementById("toggleObstacles").checked) {
      currentObstacleLayer.addTo(map);
    }
  } catch (err) {
    console.error("Ошибка загрузки ограничений:", err);
  }
}

// Очистка артефактов предыдущей миссии
function clearMissionArtifacts() {
  if (simInterval) {
    clearInterval(simInterval);
    simInterval = null;
  }
  isSimPlaying = false;
  const playBtn = document.getElementById("btnPlayPause");
  if (playBtn) playBtn.innerText = "▶";

  // Удаление маркеров симуляции с карты
  Object.keys(simDroneMarkers).forEach((dId) => {
    const item = simDroneMarkers[dId];
    if (item && item.marker && map.hasLayer(item.marker)) {
      map.removeLayer(item.marker);
    }
  });
  simDroneMarkers = {};

  // Удаление линий траекторий и точек старта/посадки
  currentSwathLayers.forEach((l) => {
    if (map.hasLayer(l)) {
      map.removeLayer(l);
    }
  });
  currentSwathLayers = [];

  // Сброс шкалы времени и индикаторов
  simCurrentSeconds = 0;
  simMaxSeconds = 0;
  const slider = document.getElementById("timeSlider");
  if (slider) slider.value = 0;
  const curTime = document.getElementById("simCurrentTime");
  if (curTime) curTime.innerText = "00:00";
  const totTime = document.getElementById("simTotalTime");
  if (totTime) totTime.innerText = "00:00";

  const statusContainer = document.getElementById("droneLiveStatuses");
  if (statusContainer) statusContainer.innerHTML = "";
  const legendBox = document.getElementById("mapLegend");
  if (legendBox) legendBox.style.display = "none";
  const alertBanner = document.getElementById("resultsAlertBanner");
  if (alertBanner) alertBanner.style.display = "none";
  const feasCard = document.getElementById("feasibilityCard");
  if (feasCard) feasCard.style.display = "none";
}

// 7. ФИЛЬТРАЦИЯ И ПРОВЕРКА СОВМЕСТИМОСТИ ФЛОТА С СЕНСОРОМ
function filterDronesBySensor() {
  const sensorSelect = document.getElementById("sensorSelect");
  if (!sensorSelect) return;
  const sensorId = sensorSelect.value;
  const compatibleList = SENSOR_DRONE_COMPATIBILITY[sensorId] || ["geoscan_201", "geoscan_gemini", "geoscan_801"];

  const droneCards = [
    { id: "geoscan_201", chkId: "drone_201", cardId: "card_drone_201" },
    { id: "geoscan_gemini", chkId: "drone_gemini", cardId: "card_drone_gemini" },
    { id: "geoscan_801", chkId: "drone_801", cardId: "card_drone_801" }
  ];

  droneCards.forEach(d => {
    const chk = document.getElementById(d.chkId);
    const card = document.getElementById(d.cardId);
    if (!chk || !card) return;

    const isCompatible = compatibleList.includes(d.id);
    let badge = card.querySelector(".incompat-badge");

    if (isCompatible) {
      chk.disabled = false;
      card.classList.remove("drone-card-incompatible");
      if (badge) badge.remove();
    } else {
      chk.checked = false;
      chk.disabled = true;
      card.classList.add("drone-card-incompatible");
      if (!badge) {
        badge = document.createElement("div");
        badge.className = "incompat-badge";
        badge.innerText = "❌ Несовместим с выбранным сенсором";
        const body = card.querySelector(".drone-card-body");
        if (body) body.appendChild(badge);
        else card.appendChild(badge);
      }
    }
  });

  // Если ни один совместимый БВС не выбран, авто-выбираем первый совместимый
  const anyChecked = droneCards.some(d => {
    const chk = document.getElementById(d.chkId);
    return chk && chk.checked && !chk.disabled;
  });
  if (!anyChecked) {
    for (const d of droneCards) {
      if (compatibleList.includes(d.id)) {
        const chk = document.getElementById(d.chkId);
        if (chk) chk.checked = true;
        break;
      }
    }
  }
}

// 8. ПРЕВЬЮ ФОТОГРАММЕТРИИ И ПРОВЕРКА СОВМЕСТИМОСТИ
async function updatePhotogrammetryPreview() {
  filterDronesBySensor();

  const sensorSelect = document.getElementById("sensorSelect");
  const sensorId = sensorSelect.value;
  const gsdCm = parseFloat(document.getElementById("gsdSlider").value);

  const compatBox = document.getElementById("droneCompatNotice");
  const warningBox = document.getElementById("regulatoryWarningBox");
  if (compatBox) compatBox.style.display = "none";
  if (warningBox) warningBox.style.display = "none";

  // Мультисенсорный режим: RGB + LiDAR
  if (sensorId === "rgb_and_lidar") {
    document.getElementById("calcHeight").innerText = "185м / 125м";
    document.getElementById("calcSpacing").innerText = "66.5м / 50.0м";
    document.getElementById("calcTrigger").innerText = "31.8м / 0.1с";
    document.getElementById("calcMaxSpeed").innerText = "15.0 м/с";

    if (compatBox) {
      compatBox.style.display = "block";
      compatBox.innerHTML = "✨ <b>Комплексная съемка:</b> АФС высокой четкости (Геоскан 201) + Лазерное сканирование (Геоскан 801). Автоматически задействуется комбинированный флот.";
    }

    document.getElementById("drone_201").checked = true;
    document.getElementById("drone_801").checked = true;
    return;
  }

  // Лазерный сканер или квантовый магнитометр
  if (sensorId === "lidar_agm_ms3" || sensorId === "geophysics_mag") {
    document.getElementById("drone_801").checked = true;
    document.getElementById("drone_201").checked = false;
    if (compatBox) {
      compatBox.style.display = "block";
      compatBox.innerHTML = `💡 Для сенсора <b>${sensorSelect.options[sensorSelect.selectedIndex].text}</b> требуется тяжелый мультиротор <b>Геоскан 801</b> (подвес до 4.5 кг). Самолет Геоскан 201 исключен.`;
    }
  }

  try {
    const res = await fetch(`${API_BASE}/api/preview-photogrammetry?sensor_id=${sensorId}&target_gsd_cm=${gsdCm}`, {
      method: "POST"
    });
    const d = await res.json();

    document.getElementById("calcHeight").innerText = `${d.flight_height_m} м`;
    document.getElementById("calcSpacing").innerText = `${d.line_spacing_m} м`;
    document.getElementById("calcTrigger").innerText = `${d.trigger_dist_m} м`;
    document.getElementById("calcMaxSpeed").innerText = `${d.max_speed_anti_blur_ms} м/с`;

    if (d.flight_height_m > 150.0 && warningBox) {
      warningBox.style.display = "block";
      warningBox.innerHTML = `⚠️ <b>Высота H = ${d.flight_height_m} м &gt; 150 м:</b> Превышение лимита Постановления Правительства РФ № 138. Требуется зональное разрешение ЕС ОрВД.`;
    }

    if (d.flight_height_m < 90.0 && sensorId !== "lidar_agm_ms3" && sensorId !== "geophysics_mag" && compatBox) {
      compatBox.style.display = "block";
      compatBox.innerHTML = `⚠️ <b>Высота H = ${d.flight_height_m} м &lt; 90 м:</b> Самолет Геоскан 201 недопустим на малых высотах (мин. 90 м). Автоматически выбирается коптер Gemini/801.`;
    }
  } catch (err) {
    console.error("Ошибка расчета фотограмметрии:", err);
  }
}

// 9. РАСЧЕТ И ФОРМИРОВАНИЕ ПОЛЕТНОГО ЗАДАНИЯ
async function calculateMission() {
  if (!currentParcelGeoJSON) {
    alert("Пожалуйста, выберите или нарисуйте участок съемки!");
    return;
  }

  const drones = [];
  const d201 = document.getElementById("drone_201");
  const dGemini = document.getElementById("drone_gemini");
  const d801 = document.getElementById("drone_801");
  if (d201 && d201.checked && !d201.disabled) drones.push("geoscan_201");
  if (dGemini && dGemini.checked && !dGemini.disabled) drones.push("geoscan_gemini");
  if (d801 && d801.checked && !d801.disabled) drones.push("geoscan_801");

  if (drones.length === 0) {
    alert("Выберите хотя бы один совместимый БВС из доступного флота!");
    return;
  }

  // Полная зачистка старых траекторий перед новым расчетом
  clearMissionArtifacts();

  const optCriterion = document.querySelector('input[name="optCriterion"]:checked').value;
  const sensorSelectValue = document.getElementById("sensorSelect").value;
  const gsdCm = parseFloat(document.getElementById("gsdSlider").value);
  const windSpeed = parseFloat(document.getElementById("windSpeedSlider").value);
  const windDir = parseFloat(document.getElementById("windDirSlider").value);

  const payload = {
    polygon_geojson: currentParcelGeoJSON,
    target_gsd_cm: gsdCm,
    available_drones: drones,
    wind: { speed_ms: windSpeed, direction_deg: windDir },
    optimization_criterion: optCriterion
  };

  // Передача угла ориентации галсов
  if (currentAngleMode === "manual") {
    const manVal = parseFloat(document.getElementById("manualAngleSlider")?.value || "0");
    payload.sweep_angle_deg = manVal;
  } else if (currentAngleMode === "axis") {
    payload.sweep_angle_deg = -1.0; // Сигнал серверу: строго вдоль продольной оси полигона
  } else if (currentAngleMode === "wind") {
    payload.sweep_angle_deg = windDir % 180;
  } else {
    payload.sweep_angle_deg = null; // Автоматический расчет глобального оптимума сервером
  }

  const timeLimitVal = document.getElementById("maxTimeLimitInput")?.value;
  const dronesLimitVal = document.getElementById("maxDronesLimitInput")?.value;
  if (timeLimitVal && parseFloat(timeLimitVal) > 0) {
    payload.max_allowed_time_min = parseFloat(timeLimitVal);
  }
  if (dronesLimitVal && parseInt(dronesLimitVal, 10) > 0) {
    payload.max_available_drones = parseInt(dronesLimitVal, 10);
  }

  if (sensorSelectValue === "rgb_and_lidar") {
    payload.sensor_ids = ["sony_rx1r2", "lidar_agm_ms3"];
  } else {
    payload.sensor_id = sensorSelectValue;
  }

  const btn1 = document.getElementById("btnCalculate");
  const btn2 = document.getElementById("btnCalculateTab");
  if (btn1) { btn1.innerText = "Расчет траекторий..."; btn1.disabled = true; }
  if (btn2) { btn2.innerText = "Расчет траекторий..."; btn2.disabled = true; }

  try {
    const res = await fetch(`${API_BASE}/api/plan-mission`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload)
    });

    const plan = await res.json();
    currentMissionPlan = plan;

    renderMissionResults(plan);
  } catch (err) {
    console.error("Ошибка расчета миссии:", err);
    alert("Ошибка расчета: " + err.message);
  } finally {
    if (btn1) { btn1.innerText = "Сформировать задание"; btn1.disabled = false; }
    if (btn2) { btn2.innerText = "⚡ Сформировать полетное задание"; btn2.disabled = false; }
  }
}

// 9. ОТРИСОВКА РЕЗУЛЬТАТОВ РАСЧЕТА
function renderMissionResults(plan) {
  clearMissionArtifacts();

  const statsRibbon = document.getElementById("statsRibbon");
  statsRibbon.style.display = "flex";

  const alertBanner = document.getElementById("resultsAlertBanner");
  let bannerMsgs = [];
  if (plan.height_warning) bannerMsgs.push(plan.height_warning);
  if (plan.fleet_rejections && Object.keys(plan.fleet_rejections).length > 0) {
    Object.keys(plan.fleet_rejections).forEach((dId) => {
      const droneName = dId === "geoscan_201" ? "Геоскан 201" : dId;
      bannerMsgs.push(`⚠️ ${droneName}: ${plan.fleet_rejections[dId].join(", ")}`);
    });
  }
  if (bannerMsgs.length > 0 && alertBanner) {
    alertBanner.style.display = "flex";
    alertBanner.innerHTML = bannerMsgs.join("<br>");
  } else if (alertBanner) {
    alertBanner.style.display = "none";
  }

  document.getElementById("statArea").innerText = `${plan.metrics.total_survey_area_ha} га`;
  document.getElementById("statMakespan").innerText = `${plan.metrics.makespan_min} мин`;
  document.getElementById("statDistance").innerText = `${plan.metrics.total_fleet_distance_km} км`;
  document.getElementById("statSwaths").innerText = `${plan.metrics.total_swaths}`;
  document.getElementById("statDronesCount").innerText = `${plan.metrics.active_drones_count}`;

  const angleBadge = document.getElementById("angleModeBadge");
  if (angleBadge && plan.metrics.optimal_sweep_angle_deg !== undefined) {
    if (currentAngleMode === "auto") {
      angleBadge.innerText = `Авто: ${plan.metrics.optimal_sweep_angle_deg}°`;
    } else {
      angleBadge.innerText = `${plan.metrics.optimal_sweep_angle_deg}°`;
    }
  }

  // Отрисовка блока ТЭО и рекомендаций эксперта
  const feasibilityCard = document.getElementById("feasibilityCard");
  if (plan.feasibility && feasibilityCard) {
    feasibilityCard.style.display = "block";
    const badge = document.getElementById("feasibilityBadge");
    if (badge) {
      if (plan.feasibility.is_feasible) {
        badge.className = "feasibility-badge badge-feasible";
        badge.innerText = "ВЫПОЛНИМО";
      } else {
        badge.className = "feasibility-badge badge-infeasible";
        badge.innerText = "ТРЕБУЕТСЯ КОРРЕКТИРОВКА";
      }
    }
    
    const summaryEl = document.getElementById("advisorSummaryText");
    if (summaryEl) summaryEl.innerText = plan.feasibility.advisor_summary || "";
    
    // Сценарии
    const scenariosGrid = document.getElementById("scenariosGrid");
    if (scenariosGrid && plan.feasibility.fleet_scenarios) {
      const sc = plan.feasibility.fleet_scenarios;
      scenariosGrid.innerHTML = `
        <div class="scenario-pill">
          <strong>${sc.single_drone.name}</strong>
          <div class="scenario-metric">Время: ${sc.single_drone.makespan_min} мин • Замен АКБ: ${sc.single_drone.battery_swaps}</div>
          <div class="scenario-metric">Износ ТО: ${sc.single_drone.to_wear_score}</div>
        </div>
        <div class="scenario-pill">
          <strong>${sc.parallel_fleet.name}</strong>
          <div class="scenario-metric">Время: ${sc.parallel_fleet.makespan_min} мин • Замен АКБ: ${sc.parallel_fleet.battery_swaps}</div>
          <div class="scenario-metric">Износ ТО: ${sc.parallel_fleet.to_wear_score}</div>
        </div>
      `;
    }
  } else if (feasibilityCard) {
    feasibilityCard.style.display = "none";
  }

  // Отрисовка анализа препятствий
  const obsChip = document.getElementById("obstacleCountChip");
  const obsAlertBox = document.getElementById("obstacleAlertBox");
  if (plan.obstacle_analysis && plan.obstacle_analysis.length > 0) {
    if (obsChip) obsChip.innerHTML = `⚠️ Препятствий рядом: <b>${plan.obstacle_analysis.length}</b>`;
    if (obsAlertBox) {
      obsAlertBox.style.display = "block";
      obsAlertBox.innerHTML = `
        <div class="obstacle-badge-list">
          ${plan.obstacle_analysis.map(o => `
            <span class="obstacle-tag ${o.status === 'SAFE_OVERFLIGHT' ? 'safe' : ''}">
              ${o.status === 'SAFE_OVERFLIGHT' ? '✓' : '⚠️'} ${o.name} (H=${o.height_m}м, ${o.status === 'SAFE_OVERFLIGHT' ? 'перелёт +' + o.clearance_m + 'м' : 'облет 70м'})
            </span>
          `).join('')}
        </div>
      `;
    }
  } else {
    if (obsChip) obsChip.innerHTML = "✓ Зона свободна от высотных препятствий";
    if (obsAlertBox) obsAlertBox.style.display = "none";
  }

  const colors = {
    "geoscan_201": "#181825",    // Midnight Ink для самолета
    "geoscan_gemini": "#f69251", // Tangerine Tag для Gemini
    "geoscan_801": "#c97b84"     // Dusty Rose для 801
  };

  const legendContainer = document.getElementById("legendItems");
  legendContainer.innerHTML = "";

  plan.drone_plans.forEach((dp) => {
    const color = colors[dp.drone_id] || "#181825";
    const lineCoords = dp.geojson_linestring.coordinates.map(c => [c[1], c[0]]);

    const polyline = L.polyline(lineCoords, {
      color: color,
      weight: dp.drone_type === "fixed_wing" ? 3.5 : 2.5,
      opacity: 0.9,
      dashArray: dp.drone_type === "fixed_wing" ? null : "6, 4"
    }).addTo(map);

    polyline.bindPopup(`
      <div style="font-size:12px; font-family:Inter, sans-serif; line-height:1.5;">
        <strong>✈️ ${dp.drone_name}</strong><br>
        <b>Сенсор:</b> ${dp.sensor_name || 'RGB'}<br>
        <b>Высота полета H:</b> ${dp.flight_height_m} м<br>
        <b>Дистанция:</b> ${dp.distance_km} км | <b>Время:</b> ${dp.flight_time_min} мин<br>
        <b>Расход батареи:</b> ${dp.battery_used_pct}%, ост. ${dp.battery_remaining_pct}%<br>
        <span style="color:#10b981; font-weight:600;">Регламент ТО: ${dp.maintenance_info ? dp.maintenance_info.status : 'Норма'}</span>
      </div>
    `);

    currentSwathLayers.push(polyline);

    // Маркер старта ВПП
    const startWp = dp.waypoints[0];
    const marker = L.circleMarker([startWp.lat, startWp.lon], {
      radius: 7,
      color: color,
      fillColor: "#ffffff",
      fillOpacity: 1,
      weight: 3
    }).bindPopup(`<b>${dp.drone_name}</b><br>Точка старта ВПП (${dp.drone_type === 'fixed_wing' ? 'Катапульта' : 'VTOL'})`);
    marker.addTo(map);
    currentSwathLayers.push(marker);

    const row = document.createElement("div");
    row.className = "legend-row";
    row.innerHTML = `
      <div class="legend-color-dot" style="background-color: ${color}"></div>
      <span><b>${dp.drone_name}</b> [${dp.sensor_name ? dp.sensor_name.split(' ')[0] : 'RGB'}] (${dp.flight_time_min} мин, ост. ${dp.battery_remaining_pct}%)</span>
    `;
    legendContainer.appendChild(row);
  });

  document.getElementById("mapLegend").style.display = "block";

  initSimulation(plan);
}

// 10. 4D СИМУЛЯЦИЯ ПОЛЕТА ГРУППЫ БВС
function initSimulation(plan) {
  simMaxSeconds = plan.metrics.makespan_min * 60.0;
  document.getElementById("simTotalTime").innerText = formatTime(simMaxSeconds);
  document.getElementById("simCurrentTime").innerText = "00:00";
  document.getElementById("timeSlider").value = 0;

  const colors = {
    "geoscan_201": "#181825",
    "geoscan_gemini": "#f69251",
    "geoscan_801": "#c97b84"
  };

  const statusContainer = document.getElementById("droneLiveStatuses");
  statusContainer.innerHTML = "";

  plan.drone_plans.forEach((dp) => {
    const startPt = dp.waypoints[0];
    const marker = L.circleMarker([startPt.lat, startPt.lon], {
      radius: 7,
      color: "#ffffff",
      fillColor: colors[dp.drone_id] || "#181825",
      fillOpacity: 1,
      weight: 2.5
    }).addTo(map);

    simDroneMarkers[dp.drone_id] = {
      marker: marker,
      plan: dp
    };

    const statusTag = document.createElement("div");
    statusTag.className = "status-chip";
    statusTag.id = `status_${dp.drone_id}`;
    statusTag.innerHTML = `
      <div class="chip-dot"></div>
      <span><b>${dp.drone_name}:</b> <span class="tag-stage">Готов к вылету</span></span>
    `;
    statusContainer.appendChild(statusTag);
  });
}

function toggleSimulation() {
  const btn = document.getElementById("btnPlayPause");
  if (isSimPlaying) {
    clearInterval(simInterval);
    simInterval = null;
    isSimPlaying = false;
    btn.innerText = "▶";
  } else {
    isSimPlaying = true;
    btn.innerText = "⏸";
    simInterval = setInterval(() => {
      simCurrentSeconds += 2.0;
      if (simCurrentSeconds >= simMaxSeconds) {
        simCurrentSeconds = simMaxSeconds;
        toggleSimulation();
      }
      document.getElementById("timeSlider").value = (simCurrentSeconds / simMaxSeconds) * 1000;
      updateSimulationFrame();
    }, 50);
  }
}

function resetSimulation() {
  if (isSimPlaying) toggleSimulation();
  simCurrentSeconds = 0;
  document.getElementById("timeSlider").value = 0;
  updateSimulationFrame();
}

function updateSimulationFrame() {
  document.getElementById("simCurrentTime").innerText = formatTime(simCurrentSeconds);

  const stageLabels = {
    "TAKEOFF": "Взлет",
    "APPROACH_ENTRY": "Выход на створ галса (Tangential)",
    "SURVEY_LINE": "Рабочий проход (Съемка)",
    "SURVEY_LINE_END": "Конец галса",
    "TURN_DUBINS": "Вираж Дубинса (R=85м)",
    "TURN_VTOL": "Координированный разворот VTOL",
    "APPROACH_LANDING": "Заход на посадку по глиссаде",
    "LANDING": "Посадка (Парашют / VTOL)"
  };

  Object.keys(simDroneMarkers).forEach((droneId) => {
    const { marker, plan } = simDroneMarkers[droneId];
    const waypoints = plan.waypoints;
    const totalTime = plan.flight_time_s;

    let targetLat = waypoints[0].lat;
    let targetLon = waypoints[0].lon;
    let currentStage = "Ожидание старта";

    if (simCurrentSeconds >= totalTime) {
      const lastWp = waypoints[waypoints.length - 1];
      targetLat = lastWp.lat;
      targetLon = lastWp.lon;
      currentStage = "Миссия завершена (Посадка)";
    } else {
      const progress = simCurrentSeconds / totalTime;
      const targetIdx = Math.floor(progress * (waypoints.length - 1));
      const wp1 = waypoints[targetIdx];
      const wp2 = waypoints[Math.min(targetIdx + 1, waypoints.length - 1)];

      const subProgress = (progress * (waypoints.length - 1)) - targetIdx;
      targetLat = wp1.lat + (wp2.lat - wp1.lat) * subProgress;
      targetLon = wp1.lon + (wp2.lon - wp1.lon) * subProgress;
      const label = stageLabels[wp1.stage] || wp1.stage;
      currentStage = `${label} (${wp1.speed_ms} м/с)`;
    }

    marker.setLatLng([targetLat, targetLon]);

    const tag = document.querySelector(`#status_${droneId} .tag-stage`);
    if (tag) tag.innerText = currentStage;
  });
}

function formatTime(seconds) {
  const m = Math.floor(seconds / 60);
  const s = Math.floor(seconds % 60);
  return `${m.toString().padStart(2, "0")}:${s.toString().padStart(2, "0")}`;
}

// 11. ЭКСПОРТ РЕЗУЛЬТАТОВ
async function exportKml() {
  if (!currentMissionPlan) return;
  const dp = currentMissionPlan.drone_plans[0];
  const res = await fetch(`${API_BASE}/api/export/kml`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(dp)
  });
  const blob = await res.blob();
  downloadBlob(blob, `mission_${dp.drone_id}.kml`);
}

async function exportGeoJson() {
  if (!currentMissionPlan) return;
  const dp = currentMissionPlan.drone_plans[0];
  const res = await fetch(`${API_BASE}/api/export/geojson`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(dp)
  });
  const blob = await res.blob();
  downloadBlob(blob, `mission_${dp.drone_id}.geojson`);
}

async function exportPlan() {
  if (!currentMissionPlan) return;
  const dp = currentMissionPlan.drone_plans[0];
  const res = await fetch(`${API_BASE}/api/export/qgc`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(dp)
  });
  const blob = await res.blob();
  downloadBlob(blob, `mission_${dp.drone_id}.plan`);
}

function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}
