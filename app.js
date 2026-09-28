// GEOSCAN FleetCommander AI - Frontend Application
// Dialog Design System Light Theme
const DEFAULT_REMOTE_API = "http://84.201.161.204";
const API_BASE = (() => {
  const saved = localStorage.getItem("geoscan_api_url");
  if (saved) return saved.replace(/\/+$/, "");
  if (window.location.hostname.endsWith("github.io")) {
    return DEFAULT_REMOTE_API;
  }
  if (window.location.protocol === "file:" || window.location.port === "5500" || window.location.port === "3000") {
    return "http://localhost:8000";
  }
  return window.location.origin;
})();

let map;
let baseLayers = {};
let currentParcelLayer = null;
let currentSwathLayers = [];
let currentZoneLayer = null;
let currentObstacleLayer = null;
let drawnItems = null;

let currentMissionPlan = null;
let comparisonPlans = {};
let missionRevision = 0;
let activePlanController = null;
let currentParcelGeoJSON = null;
let currentAngleMode = "auto"; // 'auto' | 'axis' | 'wind' | 'manual'
let allParcels = [];
let launchPoints = [];
let launchMarkers = [];
let launchPlacementMode = null;
let allowedAreaGeoJSON = null;
let allowedAreaLayer = null;
let pendingAllowedArea = false;
let previewRevision = 0;

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
  checkApiHealth();
  initMap();
  initTabsAndPanel();
  initEventListeners();
  loadParcelsList();
  filterDronesBySensor();
  updatePhotogrammetryPreview();
});

async function checkApiHealth() {
  const pill = document.getElementById("apiStatusPill");
  if (!pill) return;
  pill.onclick = () => {
    const current = localStorage.getItem("geoscan_api_url") || API_BASE;
    const nextUrl = prompt("Укажите адрес бэкенд API (например, http://84.201.161.204 или http://localhost:8000):", current);
    if (nextUrl !== null) {
      if (nextUrl.trim() === "") {
        localStorage.removeItem("geoscan_api_url");
      } else {
        localStorage.setItem("geoscan_api_url", nextUrl.trim());
      }
      window.location.reload();
    }
  };
  try {
    const res = await fetch(`${API_BASE}/api/health`, { signal: AbortSignal.timeout(4000) });
    if (res.ok) {
      const data = await res.json();
      pill.innerHTML = `● API: Онлайн (${data.parcels_count} полигонов)`;
      pill.style.color = "#059669";
      pill.style.borderColor = "#a7f3d0";
      pill.style.background = "#ecfdf5";
    } else {
      throw new Error(`HTTP ${res.status}`);
    }
  } catch (err) {
    pill.innerHTML = `⚠️ API: Офлайн (${API_BASE})`;
    pill.style.color = "#dc2626";
    pill.style.borderColor = "#fecaca";
    pill.style.background = "#fef2f2";
  }
}

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

  osmTiles.addTo(map);

  baseLayers = {
    "Карта OpenStreetMap": osmTiles,
    "☀️ Светлая карта (CARTO Positron)": positronTiles,
    "🛰️ Спутник (Esri Satellite)": satTiles
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

  function computePolygonAreaHa(geojson) {
    try {
      const coords = geojson.geometry ? geojson.geometry.coordinates[0] : geojson.coordinates[0];
      if (!coords || coords.length < 3) return null;
      let area = 0.0;
      const refLat = coords[0][1];
      const mPerDegLon = 111320.0 * Math.cos((refLat * Math.PI) / 180.0);
      const mPerDegLat = 110574.0;
      for (let i = 0; i < coords.length - 1; i++) {
        const x1 = coords[i][0] * mPerDegLon;
        const y1 = coords[i][1] * mPerDegLat;
        const x2 = coords[i + 1][0] * mPerDegLon;
        const y2 = coords[i + 1][1] * mPerDegLat;
        area += (x1 * y2 - x2 * y1);
      }
      return (Math.abs(area) / 20000.0).toFixed(2);
    } catch (e) {
      return null;
    }
  }

  map.on(L.Draw.Event.CREATED, (e) => {
    invalidateMission();
    if (pendingAllowedArea) {
      pendingAllowedArea = false;
      if (allowedAreaLayer) map.removeLayer(allowedAreaLayer);
      allowedAreaLayer = e.layer;
      allowedAreaLayer.setStyle({ color: "#15803d", fillColor: "#22c55e", fillOpacity: 0.08, weight: 2 });
      allowedAreaLayer.addTo(map);
      allowedAreaGeoJSON = allowedAreaLayer.toGeoJSON();
      document.getElementById("allowedAreaHint").innerText = "Граница задана; весь маршрут должен оставаться внутри неё.";
      return;
    }
    drawnItems.clearLayers();
    if (currentParcelLayer) map.removeLayer(currentParcelLayer);
    
    const layer = e.layer;
    drawnItems.addLayer(layer);
    currentParcelGeoJSON = layer.toGeoJSON();
    
    document.getElementById("parcelSelect").value = "";
    const areaHa = computePolygonAreaHa(currentParcelGeoJSON);
    document.getElementById("parcelInfoHint").innerText = areaHa ? `Нарисован свой полигон (${areaHa} га)` : "Нарисован пользовательский полигон";
    
    loadNearbyRestrictions(layer.getBounds());
  });

  map.on("click", (event) => {
    if (!launchPlacementMode) return;
    addLaunchPoint(event.latlng, launchPlacementMode);
    launchPlacementMode = null;
    map.getContainer().style.cursor = "";
    document.querySelectorAll(".map-edit-actions button").forEach(button => button.classList.remove("active"));
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
  const openBtn = document.getElementById("btnOpenPanel");
  const panel = document.getElementById("controlPanel");

  toggleBtn.addEventListener("click", () => {
    const isCollapsed = panel.classList.toggle("collapsed");
    toggleBtn.innerText = isCollapsed ? "▶" : "◀";
    document.body.classList.toggle("panel-collapsed", isCollapsed);
    openBtn.hidden = !isCollapsed;
  });
  openBtn.addEventListener("click", () => toggleBtn.click());
}

function switchTab(tabId) {
  document.querySelectorAll(".tab-btn").forEach((b) => {
    b.classList.toggle("active", b.getAttribute("data-tab") === tabId);
  });
  document.querySelectorAll(".tab-content").forEach((c) => {
    c.classList.toggle("active", c.id === tabId);
  });
}

function showMissionMessage(message) {
  const box = document.getElementById("missionMessage");
  box.textContent = message;
  box.hidden = false;
  box.focus();
}

function escapeHtml(value) {
  return String(value ?? "").replace(/[&<>"']/g, character => ({
    "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;"
  })[character]);
}

function invalidateMission() {
  missionRevision += 1;
  if (activePlanController) activePlanController.abort();
  activePlanController = null;
  currentMissionPlan = null;
  comparisonPlans = {};
  clearMissionArtifacts();
  document.getElementById("statsRibbon").style.display = "none";
  document.getElementById("comparisonSection").hidden = true;
  document.getElementById("missionMessage").hidden = true;
  const btn1 = document.getElementById("btnCalculate");
  const btn2 = document.getElementById("btnCalculateTab");
  if (btn1) { btn1.innerText = "Сформировать задание"; btn1.disabled = false; }
  if (btn2) { btn2.innerText = "Сформировать полетное задание"; btn2.disabled = false; }
}

function renderLaunchPoints() {
  const list = document.getElementById("launchPointList");
  list.replaceChildren();
  launchMarkers.forEach(marker => map.removeLayer(marker));
  launchMarkers = [];
  launchPoints.forEach((point, index) => {
    const marker = L.marker([point.lat, point.lon], { draggable: true })
      .bindPopup(`${point.type === "base" ? "База" : "Резервная площадка"} ${index + 1}`)
      .addTo(map);
    marker.on("dragend", () => {
      const position = marker.getLatLng();
      point.lat = position.lat;
      point.lon = position.lng;
      invalidateMission();
      renderLaunchPoints();
    });
    launchMarkers.push(marker);
    const row = document.createElement("div");
    row.className = "launch-point-row";
    const label = document.createElement("span");
    label.textContent = `${point.type === "base" ? "База" : "Резервная"} ${index + 1}: ${point.lat.toFixed(5)}, ${point.lon.toFixed(5)}`;
    const remove = document.createElement("button");
    remove.type = "button";
    remove.textContent = "Удалить";
    remove.setAttribute("aria-label", `Удалить точку ${index + 1}`);
    remove.addEventListener("click", () => {
      launchPoints.splice(index, 1);
      invalidateMission();
      renderLaunchPoints();
    });
    row.append(label, remove);
    list.appendChild(row);
  });
}

function addLaunchPoint(latlng, type) {
  launchPoints.push({
    id: `${type}_${Date.now()}_${launchPoints.length}`,
    name: type === "base" ? "База оператора" : "Резервная площадка",
    lat: latlng.lat,
    lon: latlng.lng,
    type
  });
  invalidateMission();
  renderLaunchPoints();
}

function setLaunchPlacement(type, button) {
  launchPlacementMode = type;
  map.getContainer().style.cursor = "crosshair";
  document.querySelectorAll(".map-edit-actions button").forEach(item => item.classList.remove("active"));
  button.classList.add("active");
  showMissionMessage("Щёлкните по карте, чтобы поставить точку. После этого её можно перетащить.");
}

// 3. ИНИЦИАЛИЗАЦИЯ СЛУШАТЕЛЕЙ СОБЫТИЙ
function initEventListeners() {
  // Селектор участков
  document.getElementById("parcelSelect").addEventListener("change", (e) => {
    const fid = e.target.value;
    if (fid) selectParcel(parseInt(fid));
  });

  document.getElementById("btnRefreshParcels").addEventListener("click", loadParcelsList);
  document.getElementById("parcelSearch").addEventListener("input", renderParcelOptions);
  
  // Рисование своего полигона
  const btnDraw = document.getElementById("btnDrawCustom");
  if (btnDraw) {
    btnDraw.addEventListener("click", () => {
      pendingAllowedArea = false;
      showMissionMessage("Кликайте по карте для построения вершин полигона. Двойной клик или клик по первой точке — завершить рисование.");
      new L.Draw.Polygon(map, {
        allowIntersection: false,
        showArea: true,
        shapeOptions: { color: "#f69251", weight: 2, fillOpacity: 0.15 }
      }).enable();
    });
  }

  document.getElementById("btnAddBase").addEventListener("click", event => setLaunchPlacement("base", event.currentTarget));
  document.getElementById("btnAddEmergency").addEventListener("click", event => setLaunchPlacement("emergency_pad", event.currentTarget));
  document.getElementById("btnDrawAllowed").addEventListener("click", () => {
    pendingAllowedArea = true;
    new L.Draw.Polygon(map, { allowIntersection: false, shapeOptions: { color: "#15803d", fillOpacity: 0.08 } }).enable();
  });
  document.getElementById("btnClearAllowed").addEventListener("click", () => {
    if (allowedAreaLayer) map.removeLayer(allowedAreaLayer);
    allowedAreaLayer = null;
    allowedAreaGeoJSON = null;
    document.getElementById("allowedAreaHint").innerText = "Не задана. Ограничения из набора данных проверяются отдельно.";
    invalidateMission();
  });

  document.querySelectorAll("#sensorSelect, #gsdSlider, #windSpeedSlider, #windDirSlider, #manualAngleSlider, #maxTimeLimitInput, #maxDronesLimitInput, #overlapForwardInput, #overlapSideInput, #batterySwapInput, #drone_201, #drone_gemini, #drone_801")
    .forEach(input => input.addEventListener(input.type === "range" ? "input" : "change", invalidateMission));
  document.querySelectorAll("input[name='optCriterion']").forEach(input => input.addEventListener("change", () => {
    if (comparisonPlans[input.value]) {
      currentMissionPlan = comparisonPlans[input.value];
      renderMissionResults(currentMissionPlan);
      renderComparison();
    } else if (currentMissionPlan) {
      input.checked = false;
      document.querySelector(`input[name="optCriterion"][value="${currentMissionPlan.optimization_criterion}"]`).checked = true;
      showMissionMessage("Для выбранного критерия допустимый маршрут не найден. Измените условия и повторите расчёт.");
    } else invalidateMission();
  }));

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
    
    // Стрелка флюгера/компаса указывает строго на азимут направления ветра (согласно лимбу N вверху и тексту)
    const arrowRot = deg;
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
    invalidateMission();
    currentAngleMode = mode;
    if (btnAuto) btnAuto.classList.toggle("active", mode === "auto");
    if (btnAxis) btnAxis.classList.toggle("active", mode === "axis");
    if (btnWind) btnWind.classList.toggle("active", mode === "wind");
    if (btnManual) btnManual.classList.toggle("active", mode === "manual");
    if (manualBlock) manualBlock.style.display = mode === "manual" ? "block" : "none";
    if (angleBadge) {
      if (mode === "auto") angleBadge.innerText = "Авто (поиск)";
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
  document.getElementById("showAlternativeRoute").addEventListener("change", () => {
    if (currentMissionPlan) renderMissionResults(currentMissionPlan);
  });
  const chkEmerg = document.getElementById("showEmergencyRoute");
  if (chkEmerg) {
    chkEmerg.addEventListener("change", () => {
      if (currentMissionPlan) renderMissionResults(currentMissionPlan);
    });
  }
  const btnEmergPlan = document.getElementById("btnExportEmergPlan");
  if (btnEmergPlan) {
    btnEmergPlan.addEventListener("click", exportEmergencyPlan);
  }


  // Регуляторы заряда батарей БВС
  [["battery_201", "battery_val_201"], ["battery_gemini", "battery_val_gemini"], ["battery_801", "battery_val_801"]].forEach(([rangeId, valId]) => {
    const range = document.getElementById(rangeId);
    const valEl = document.getElementById(valId);
    if (range && valEl) {
      range.addEventListener("input", (e) => {
        valEl.innerText = `${e.target.value}%`;
        invalidateMission();
      });
    }
  });

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
    const res = await fetch(`${API_BASE}/api/parcels?limit=1000`);
    if (!res.ok) throw new Error(`Список участков: HTTP ${res.status}`);
    const data = await res.json();
    allParcels = data.parcels;
    renderParcelOptions();
    document.getElementById("datasetStats").innerText = `${allParcels.length} участков • зоны проверяются`;
    if (allParcels.length > 0) selectParcel(allParcels[0].index);
  } catch (err) {
    console.error("Ошибка загрузки участков:", err);
    showMissionMessage(`Не удалось загрузить участки: ${err.message}`);
  }
}

function renderParcelOptions() {
    const select = document.getElementById("parcelSelect");
    const previous = select.value;
    select.replaceChildren(new Option("— Выберите участок —", ""));
    const query = document.getElementById("parcelSearch").value.trim().toLowerCase();
    const matching = allParcels.filter(p => !query || String(p.fid).includes(query) || String(p.area_ha).includes(query));
    matching.forEach((p) => {
      const opt = document.createElement("option");
      opt.value = p.index;
      opt.text = `FID ${p.fid} · ${p.area_ha ? `${p.area_ha} га` : "<0,01 га"} · №${p.index + 1}`;
      select.appendChild(opt);
    });
    if (matching.some(p => String(p.index) === previous)) select.value = previous;
    document.getElementById("parcelCountHint").innerText = `Найдено ${matching.length} из ${allParcels.length} участков`;
    if (query && matching.length === 1 && String(matching[0].index) !== previous) {
      selectParcel(matching[0].index);
    }
}

// 5. ВЫБОР УЧАСТКА
async function selectParcel(index) {
  try {
    invalidateMission();
    const statsRibbon = document.getElementById("statsRibbon");
    if (statsRibbon) statsRibbon.style.display = "none";

    const res = await fetch(`${API_BASE}/api/parcels/by-index/${index}`);
    if (!res.ok) throw new Error(`Участок: HTTP ${res.status}`);
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

    document.getElementById("parcelSelect").value = String(index);
    const area = geojson.properties.area_ha ? `${geojson.properties.area_ha} га` : `${geojson.properties.area_m2} м²`;
    document.getElementById("parcelInfoHint").innerText = `Площадь: ${area} | FID: ${geojson.properties.fid}`;

    loadNearbyRestrictions(bounds);
  } catch (err) {
    console.error("Ошибка загрузки полигона:", err);
    showMissionMessage(`Не удалось загрузить участок: ${err.message}`);
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
            <strong style="color:#c97b84;">⛔ ${escapeHtml(p.name)}</strong> (${escapeHtml(p.type)})<br>
            <b>Высотный диапазон:</b> от ${escapeHtml(alt.min_alt_m)}м до ${escapeHtml(alt.max_alt_m)}м (${escapeHtml(alt.reference)})<br>
            <small style="color:#8b8b8b;">Время действия уточняется оператором</small>
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
            <strong style="color:#b91c1c;">🗼 ${escapeHtml(p.name)}</strong><br>
            Высота препятствия: <b>${p.height_m} м AGL</b><br>
            Зона безопасности: <b>${rad} м</b><br>
            <small style="color:#64748b;">Маршрут проверяется относительно буфера при расчёте</small>
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

// 7. ФИЛЬТРАЦИЯ И ПРОВЕРКА СОВМЕСТИМОСТИ ФЛОТА С СЕНСОРОМ И ВЫСОТОЙ
function filterDronesBySensor(flightHeightM = null) {
  const sensorSelect = document.getElementById("sensorSelect");
  if (!sensorSelect) return;
  const sensorId = sensorSelect.value;
  const compatibleList = SENSOR_DRONE_COMPATIBILITY[sensorId] || ["geoscan_201", "geoscan_gemini", "geoscan_801"];

  const droneCards = [
    { id: "geoscan_201", chkId: "drone_201", cardId: "card_drone_201", minAlt: 90.0, label: "самолета" },
    { id: "geoscan_gemini", chkId: "drone_gemini", cardId: "card_drone_gemini", minAlt: 30.0, label: "коптера Gemini" },
    { id: "geoscan_801", chkId: "drone_801", cardId: "card_drone_801", minAlt: 25.0, label: "тяжелого БВС 801" }
  ];

  droneCards.forEach(d => {
    const chk = document.getElementById(d.chkId);
    const card = document.getElementById(d.cardId);
    if (!chk || !card) return;

    const isSensorCompat = compatibleList.includes(d.id);
    const isHeightCompat = flightHeightM === null || flightHeightM >= d.minAlt;
    let badge = card.querySelector(".incompat-badge");

    if (isSensorCompat && isHeightCompat) {
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
        const body = card.querySelector(".drone-card-body");
        if (body) body.appendChild(badge);
        else card.appendChild(badge);
      }
      if (!isSensorCompat) {
        badge.innerText = "❌ Несовместим с выбранным сенсором";
      } else {
        badge.innerText = `❌ H = ${flightHeightM} м < ${d.minAlt} м (мин. для ${d.label})`;
      }
    }
  });

  // Если ни один совместимый БВС не выбран, авто-выбираем первый доступный
  const anyChecked = droneCards.some(d => {
    const chk = document.getElementById(d.chkId);
    return chk && chk.checked && !chk.disabled;
  });
  if (!anyChecked) {
    for (const d of droneCards) {
      const chk = document.getElementById(d.chkId);
      if (chk && !chk.disabled) {
        chk.checked = true;
        break;
      }
    }
  }
}

// 8. ПРЕВЬЮ ФОТОГРАММЕТРИИ И ПРОВЕРКА СОВМЕСТИМОСТИ
async function updatePhotogrammetryPreview() {
  const revision = ++previewRevision;
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
    if (compatBox) {
      compatBox.style.display = "block";
      compatBox.textContent = "Комплексная съёмка: два сенсора планируются по отдельности и затем ставятся в общее расписание.";
    }

    try {
      const data = await Promise.all(["sony_rx1r2", "lidar_agm_ms3"].map(async id => {
        const response = await fetch(`${API_BASE}/api/preview-photogrammetry?sensor_id=${id}&target_gsd_cm=${gsdCm}`, { method: "POST" });
        if (!response.ok) throw new Error("Превью недоступно");
        return response.json();
      }));
      if (revision !== previewRevision) return;
      document.getElementById("calcHeight").innerText = data.map(d => `${d.flight_height_m} м`).join(" / ");
      document.getElementById("calcSpacing").innerText = data.map(d => `${d.line_spacing_m} м`).join(" / ");
      document.getElementById("calcTrigger").innerText = data.map(d => `${d.trigger_dist_m} м`).join(" / ");
      document.getElementById("calcMaxSpeed").innerText = data.map(d => `${d.max_speed_anti_blur_ms} м/с`).join(" / ");
      const minH = Math.min(...data.map(d => d.flight_height_m));
      filterDronesBySensor(minH);
    } catch (err) {
      if (revision === previewRevision) showMissionMessage(`Не удалось рассчитать превью: ${err.message}`);
    }
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
    if (revision !== previewRevision) return;

    document.getElementById("calcHeight").innerText = `${d.flight_height_m} м`;
    document.getElementById("calcSpacing").innerText = `${d.line_spacing_m} м`;
    document.getElementById("calcTrigger").innerText = `${d.trigger_dist_m} м`;
    document.getElementById("calcMaxSpeed").innerText = `${d.max_speed_anti_blur_ms} м/с`;

    // Применяем проверку высоты полета ко всему флоту
    filterDronesBySensor(d.flight_height_m);

    if (d.flight_height_m > 150.0 && warningBox) {
      warningBox.style.display = "block";
      warningBox.textContent = `Высота H = ${d.flight_height_m} м превышает порог 150 м. Проверьте применимые ограничения и разрешения до вылета.`;
    }

    if (d.flight_height_m < 90.0 && sensorId !== "lidar_agm_ms3" && sensorId !== "geophysics_mag" && compatBox) {
      compatBox.style.display = "block";
      compatBox.innerHTML = `⚠️ <b>Высота H = ${d.flight_height_m} м &lt; 90 м:</b> Самолет Геоскан 201 недопустим на малых высотах (мин. 90 м). Автоматически выбран квадрокоптер Gemini.`;
    }
  } catch (err) {
    console.error("Ошибка расчета фотограмметрии:", err);
  }
}

// 9. РАСЧЕТ И ФОРМИРОВАНИЕ ПОЛЕТНОГО ЗАДАНИЯ
async function calculateMission() {
  if (!currentParcelGeoJSON) {
    showMissionMessage("Выберите или нарисуйте участок съемки.");
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
    showMissionMessage("Выберите хотя бы один совместимый БВС.");
    return;
  }

  if (launchPoints.length && !launchPoints.some(point => point.type === "base")) {
    showMissionMessage("Добавлена резервная площадка, но не задана основная база. Добавьте базу на карте.");
    switchTab("tab-territory");
    return;
  }

  for (const id of ["maxTimeLimitInput", "maxDronesLimitInput", "overlapForwardInput", "overlapSideInput", "batterySwapInput"]) {
    const input = document.getElementById(id);
    if (input.value !== "" && !input.checkValidity()) {
      showMissionMessage(`Проверьте значение поля «${input.closest("label")?.textContent.trim() || input.getAttribute("aria-label") || id}».`);
      switchTab("tab-weather");
      input.focus();
      return;
    }
  }

  invalidateMission();
  const revision = missionRevision;
  activePlanController = new AbortController();
  const signal = activePlanController.signal;

  const optCriterion = document.querySelector('input[name="optCriterion"]:checked').value;
  const sensorSelectValue = document.getElementById("sensorSelect").value;
  const gsdCm = parseFloat(document.getElementById("gsdSlider").value);
  const windSpeed = parseFloat(document.getElementById("windSpeedSlider").value);
  const windDir = parseFloat(document.getElementById("windDirSlider").value);

  const batteryLevels = {};
  const b201 = document.getElementById("battery_201");
  const bGemini = document.getElementById("battery_gemini");
  const b801 = document.getElementById("battery_801");
  if (b201) batteryLevels["geoscan_201"] = parseFloat(b201.value);
  if (bGemini) batteryLevels["geoscan_gemini"] = parseFloat(bGemini.value);
  if (b801) batteryLevels["geoscan_801"] = parseFloat(b801.value);

  const payload = {
    polygon_geojson: currentParcelGeoJSON,
    target_gsd_cm: gsdCm,
    available_drones: drones,
    drone_battery_levels: batteryLevels,
    wind: { speed_ms: windSpeed, direction_deg: windDir },
    optimization_criterion: optCriterion,
    launch_points: launchPoints.length ? launchPoints : null,
    allowed_airspace_geojson: allowedAreaGeoJSON,
    battery_swap_penalty_min: Number(document.getElementById("batterySwapInput").value || "15")
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
    payload.sweep_angle_deg = null;
  }

  const timeLimitVal = document.getElementById("maxTimeLimitInput")?.value;
  const dronesLimitVal = document.getElementById("maxDronesLimitInput")?.value;
  if (timeLimitVal && parseFloat(timeLimitVal) > 0) {
    payload.max_allowed_time_min = parseFloat(timeLimitVal);
  }
  if (dronesLimitVal && parseInt(dronesLimitVal, 10) > 0) {
    payload.max_available_drones = parseInt(dronesLimitVal, 10);
  }
  for (const [id, field] of [["overlapForwardInput", "overlap_forward"], ["overlapSideInput", "overlap_side"]]) {
    const input = document.getElementById(id);
    if (input.value !== "") payload[field] = Number(input.value) / 100;
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
    const criteria = ["min_makespan", "min_flight_time"];
    const responses = await Promise.all(criteria.map(async criterion => {
      try {
        const res = await fetch(`${API_BASE}/api/plan-mission`, {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ ...payload, optimization_criterion: criterion }),
          signal
        });
        const body = await res.json();
        return res.ok ? { plan: body } : { error: body.detail || `HTTP ${res.status}` };
      } catch (error) {
        if (error.name === "AbortError") throw error;
        return { error: error.message || "Сервис недоступен" };
      }
    }));
    if (revision !== missionRevision) return;
    criteria.forEach((criterion, index) => {
      if (responses[index].plan) comparisonPlans[criterion] = responses[index].plan;
    });
    if (!comparisonPlans[optCriterion]) {
      throw new Error(responses[criteria.indexOf(optCriterion)].error || "Маршрут не найден");
    }
    currentMissionPlan = comparisonPlans[optCriterion];
    renderMissionResults(currentMissionPlan);
    renderComparison(responses);
  } catch (err) {
    if (err.name === "AbortError" || revision !== missionRevision) return;
    console.error("Ошибка расчета миссии:", err);
    showMissionMessage(`Маршрут не найден: ${err.message}. Измените область, флот или условия и повторите расчёт.`);
  } finally {
    if (revision === missionRevision) {
      activePlanController = null;
      if (btn1) { btn1.innerText = "Сформировать задание"; btn1.disabled = false; }
      if (btn2) { btn2.innerText = "Сформировать полетное задание"; btn2.disabled = false; }
    }
  }
}

function renderComparison(responses = null) {
  const section = document.getElementById("comparisonSection");
  const grid = document.getElementById("comparisonGrid");
  grid.replaceChildren();
  const criteria = [
    ["min_makespan", "Быстрее завершить"],
    ["min_flight_time", "Меньше налёт"]
  ];
  criteria.forEach(([criterion, title], index) => {
    const plan = comparisonPlans[criterion];
    const card = document.createElement("button");
    card.type = "button";
    card.className = `comparison-card${currentMissionPlan === plan ? " selected" : ""}`;
    if (plan) {
      const m = plan.metrics;
      card.innerHTML = `<strong>${title}</strong><span>Завершение: ${m.makespan_min} мин · Налёт: ${m.total_fleet_time_min} мин</span><span>Дистанция: ${m.total_fleet_distance_km} км · Бортов: ${m.active_drones_count} · Покрытие: ${m.coverage_pct}%</span>`;
      card.addEventListener("click", () => {
        currentMissionPlan = plan;
        document.querySelector(`input[name="optCriterion"][value="${criterion}"]`).checked = true;
        renderMissionResults(plan);
        renderComparison();
      });
    } else {
      card.disabled = true;
      card.innerHTML = `<strong>${title}</strong><span>Допустимый маршрут не найден${responses?.[index]?.error ? `: ${escapeHtml(responses[index].error)}` : ""}</span>`;
    }
    grid.appendChild(card);
  });
  section.hidden = false;
  document.getElementById("comparisonNote").textContent = "Лучшие найденные варианты; оптимальность не доказана";
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
    alertBanner.innerHTML = bannerMsgs.map(escapeHtml).join("<br>");
  } else if (alertBanner) {
    alertBanner.style.display = "none";
  }

  document.getElementById("statArea").innerText = `${plan.metrics.total_survey_area_ha} га`;
  document.getElementById("statMakespan").innerText = `${plan.metrics.makespan_min} мин`;
  document.getElementById("statFlightTime").innerText = `${plan.metrics.total_fleet_time_min} мин`;
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
      if (plan.height_warning) {
        badge.className = "feasibility-badge badge-review";
        badge.innerText = "ТРЕБУЕТ ПРОВЕРКИ";
      } else if (plan.feasibility.is_feasible) {
        badge.className = "feasibility-badge badge-feasible";
        badge.innerText = "ВЫПОЛНИМО";
      } else {
        badge.className = "feasibility-badge badge-infeasible";
        badge.innerText = "ТРЕБУЕТСЯ КОРРЕКТИРОВКА";
      }
    }
    
    const summaryEl = document.getElementById("advisorSummaryText");
    if (summaryEl) {
      const search = plan.search && !Array.isArray(plan.search) ? plan.search : null;
      const searchNote = search ? ` Оценено ${search.evaluated_candidates} вариантов; показан лучший найденный маршрут, глобальный оптимум не доказан.` : "";
      const coverageNote = plan.metrics.coverage_pct !== undefined ? ` Покрытие съемкой: ${plan.metrics.coverage_pct}%.` : "";
      let emergNote = "";
      if (plan.drone_plans.some(dp => dp.emergency_diversion)) {
        const parts = plan.drone_plans
          .filter(dp => dp.emergency_diversion)
          .map(dp => `${dp.drone_name}: ${dp.emergency_diversion.pad_name} (${dp.emergency_diversion.distance_km} км, ${dp.emergency_diversion.flight_time_min} мин)`);
        emergNote = ` 🚨 Ближайшая резервная посадка: ${parts.join("; ")}.`;
      }
      summaryEl.innerText = (plan.feasibility.advisor_summary || "") + coverageNote + emergNote + searchNote;
    }
    
    // Сценарии
    const scenariosGrid = document.getElementById("scenariosGrid");
    if (scenariosGrid) scenariosGrid.innerHTML = "";
    if (scenariosGrid && plan.feasibility.fleet_scenarios) {
      const sc = plan.feasibility.fleet_scenarios;
      scenariosGrid.innerHTML = `
        <div class="scenario-pill">
          <strong>${escapeHtml(sc.single_drone.name)}</strong>
          <div class="scenario-metric">Время: ${sc.single_drone.makespan_min} мин • Замен АКБ: ${sc.single_drone.battery_swaps}</div>
          <div class="scenario-metric">Износ ТО: ${sc.single_drone.to_wear_score}</div>
        </div>
        <div class="scenario-pill">
          <strong>${escapeHtml(sc.parallel_fleet.name)}</strong>
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
              ${o.status === 'SAFE_OVERFLIGHT' ? '✓' : '⚠️'} ${escapeHtml(o.name)} (H=${o.height_m}м, ${o.status === 'SAFE_OVERFLIGHT' ? 'перелёт +' + o.clearance_m + 'м' : 'облет 70м'})
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
  legendContainer.replaceChildren();

  const alternative = Object.values(comparisonPlans).find(candidate => candidate !== plan);
  if (alternative && document.getElementById("showAlternativeRoute").checked) {
    alternative.drone_plans.forEach(dp => {
      const coords = dp.geojson_linestring.coordinates.map(c => [c[1], c[0]]);
      const line = L.polyline(coords, { color: "#7c3aed", weight: 2, opacity: 0.7, dashArray: "3, 7" }).addTo(map);
      currentSwathLayers.push(line);
    });
  }

  // Отрисовка аварийного схода на резервные площадки
  const hasEmergency = plan.drone_plans.some(dp => dp.emergency_diversion);
  const emergLabel = document.getElementById("emergOverlayLabel");
  if (emergLabel) {
    emergLabel.style.display = hasEmergency ? "inline-flex" : "none";
  }
  const btnEmergExport = document.getElementById("btnExportEmergPlan");
  if (btnEmergExport) {
    btnEmergExport.style.display = hasEmergency ? "inline-flex" : "none";
  }

  if (hasEmergency && document.getElementById("showEmergencyRoute")?.checked) {
    plan.drone_plans.forEach(dp => {
      if (!dp.emergency_diversion) return;
      const em = dp.emergency_diversion;
      const coords = em.geojson_linestring.coordinates.map(c => [c[1], c[0]]);
      const emergLine = L.polyline(coords, {
        color: "#dc2626",
        weight: 3,
        opacity: 0.9,
        dashArray: "6, 6"
      }).addTo(map);
      emergLine.bindPopup(`
        <div style="font-size:12px; font-family:Inter, sans-serif; line-height:1.5; min-width:220px;">
          <div style="font-weight:700; color:#dc2626; font-size:13px; margin-bottom:4px;">🚨 Аварийный сход на резервную ВПП</div>
          <div><b>Площадка:</b> ${escapeHtml(em.pad_name)}</div>
          <div><b>Борт:</b> ${escapeHtml(dp.drone_name)}</div>
          <div><b>Дистанция:</b> ${em.distance_km} км | <b>Время:</b> ${em.flight_time_min} мин</div>
          <div><b>Расход батареи:</b> ${em.battery_needed_pct}%</div>
          <div style="margin-top:6px; padding:6px; background:#fef2f2; border-left:3px solid #dc2626; border-radius:4px; font-size:11px;">
            <b>Процедура захода:</b><br>${escapeHtml(em.landing_procedure || "Заход на ВПП")}
          </div>
        </div>
      `);
      currentSwathLayers.push(emergLine);

      // Точка безопасного возврата (PSR - Point of Safe Return)
      if (em.psr_info) {
        const psrMarker = L.circleMarker([em.psr_info.lat, em.psr_info.lon], {
          radius: 6,
          color: "#e11d48",
          fillColor: "#fecdd3",
          fillOpacity: 1,
          weight: 2
        }).addTo(map);
        psrMarker.bindPopup(`<b>⚠️ ${escapeHtml(em.psr_info.description)}</b><br><b>Борт:</b> ${escapeHtml(dp.drone_name)}<br>Критический рубеж: возврат на базу возможен только до этой точки; после неё требуется сход на резервную ВПП.`);
        currentSwathLayers.push(psrMarker);
      }

      const emergMarker = L.circleMarker([em.lat, em.lon], {
        radius: 8,
        color: "#dc2626",
        fillColor: "#fee2e2",
        fillOpacity: 1,
        weight: 2.5
      }).bindPopup(`
        <div style="font-size:12px; font-family:Inter, sans-serif; line-height:1.5;">
          <b style="color:#dc2626;">🚨 Резервная площадка: ${escapeHtml(em.pad_name)}</b><br>
          <b>Координаты:</b> ${em.lat.toFixed(5)}, ${em.lon.toFixed(5)}<br>
          <b>Процедура:</b> ${escapeHtml(em.landing_procedure || "Посадка")}<br>
          <b>Дистанция от полигона:</b> ${em.distance_km} км
        </div>
      `);
      emergMarker.addTo(map);
      currentSwathLayers.push(emergMarker);
    });
  }

  plan.drone_plans.forEach((dp) => {
    const color = colors[dp.drone_id] || "#181825";
    const lineCoords = dp.geojson_linestring.coordinates.map(c => [c[1], c[0]]);
    const routeLayers = [];

    const polyline = L.polyline(lineCoords, {
      color: "#64748b",
      weight: 2,
      opacity: 0.7,
      dashArray: "5, 5"
    }).addTo(map);
    routeLayers.push(polyline);

    polyline.bindPopup(`
      <div style="font-size:12px; font-family:Inter, sans-serif; line-height:1.5;">
        <strong>${escapeHtml(dp.drone_name)}</strong><br>
        <b>Сенсор:</b> ${escapeHtml(dp.sensor_name || 'RGB')}<br>
        <b>Высота полета H:</b> ${dp.flight_height_m} м<br>
        <b>Дистанция:</b> ${dp.distance_km} км | <b>Время:</b> ${dp.flight_time_min} мин<br>
        <b>Расход батареи:</b> ${dp.battery_used_pct}%, ост. ${dp.battery_remaining_pct}%<br>
        <span style="color:#10b981; font-weight:600;">Регламент ТО: ${dp.maintenance_info ? dp.maintenance_info.status : 'Норма'}</span>
      </div>
    `);

    currentSwathLayers.push(polyline);

    if (dp.waypoints.length === lineCoords.length) {
      for (let i = 0; i < dp.waypoints.length - 1; i++) {
        if (dp.waypoints[i].stage !== "SURVEY_LINE" || dp.waypoints[i + 1].stage !== "SURVEY_LINE_END") continue;
        const surveyLine = L.polyline([lineCoords[i], lineCoords[i + 1]], {
          color, weight: 3.5, opacity: 0.95
        }).addTo(map);
        routeLayers.push(surveyLine);
        currentSwathLayers.push(surveyLine);
      }
    }

    // Маркер старта ВПП
    const startWp = dp.waypoints[0];
    const marker = L.circleMarker([startWp.lat, startWp.lon], {
      radius: 7,
      color: color,
      fillColor: "#ffffff",
      fillOpacity: 1,
      weight: 3
    }).bindPopup(`<b>${escapeHtml(dp.drone_name)}</b><br>Точка старта (${dp.drone_type === 'fixed_wing' ? 'Катапульта' : 'VTOL'})`);
    marker.addTo(map);
    currentSwathLayers.push(marker);
    routeLayers.push(marker);

    const row = document.createElement("label");
    row.className = "legend-row";
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.checked = true;
    checkbox.addEventListener("change", () => routeLayers.forEach(layer => {
      if (checkbox.checked) layer.addTo(map);
      else map.removeLayer(layer);
    }));
    const dot = document.createElement("span");
    dot.className = "legend-color-dot";
    dot.style.backgroundColor = color;
    const description = document.createElement("span");
    description.textContent = `${dp.drone_name} · ${dp.sensor_name || "RGB"} · вылет ${dp.sortie_index || 1} · ${dp.flight_time_min} мин · остаток ${dp.battery_remaining_pct}%`;
    row.append(checkbox, dot, description);
    legendContainer.appendChild(row);
  });

  const key = document.createElement("div");
  key.className = "legend-key";
  key.textContent = "Цветные линии — съёмка · серый пунктир — перелёт · фиолетовый — альтернатива";
  legendContainer.appendChild(key);

  document.getElementById("mapLegend").style.display = "block";

  initSimulation(plan);
  const routeBounds = L.featureGroup(currentSwathLayers).getBounds();
  if (routeBounds.isValid()) {
    const desktop = window.innerWidth > 900;
    map.fitBounds(routeBounds, {
      paddingTopLeft: desktop ? [document.body.classList.contains("panel-collapsed") ? 35 : 405, 80] : [20, 20],
      paddingBottomRight: desktop ? [25, Math.min(document.getElementById("statsRibbon").offsetHeight + 30, 430)] : [20, 20],
      maxZoom: 16
    });
  }
}

// 10. 4D СИМУЛЯЦИЯ ПОЛЕТА ГРУППЫ БВС
function initSimulation(plan) {
  simMaxSeconds = Math.max(plan.metrics.makespan_min * 60.0,
    ...plan.drone_plans.map(dp => (dp.start_time_s || 0) + dp.flight_time_s));
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

  plan.drone_plans.forEach((dp, index) => {
    const markerId = `${dp.drone_id}_${index}`;
    const startPt = dp.waypoints[0];
    const marker = L.circleMarker([startPt.lat, startPt.lon], {
      radius: 7,
      color: "#ffffff",
      fillColor: colors[dp.drone_id] || "#181825",
      fillOpacity: 1,
      weight: 2.5
    }).addTo(map);

    simDroneMarkers[markerId] = {
      marker: marker,
      plan: dp
    };

    const statusTag = document.createElement("div");
    statusTag.className = "status-chip";
    statusTag.id = `status_${markerId}`;
    statusTag.innerHTML = `
      <div class="chip-dot"></div>
      <span><b>${escapeHtml(dp.drone_name)} (вылет ${dp.sortie_index || 1}):</b> <span class="tag-stage">Готов к вылету</span></span>
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

  Object.keys(simDroneMarkers).forEach((markerId) => {
    const { marker, plan } = simDroneMarkers[markerId];
    const waypoints = plan.waypoints;
    const totalTime = plan.flight_time_s;
    const localTime = simCurrentSeconds - (plan.start_time_s || 0);

    let targetLat = waypoints[0].lat;
    let targetLon = waypoints[0].lon;
    let currentStage = "Ожидание старта";

    if (localTime >= totalTime) {
      const lastWp = waypoints[waypoints.length - 1];
      targetLat = lastWp.lat;
      targetLon = lastWp.lon;
      currentStage = "Миссия завершена (Посадка)";
    } else if (localTime >= 0) {
      const times = plan.waypoint_times_s;
      let targetIdx;
      let subProgress;
      if (times && times.length === waypoints.length) {
        let low = 0;
        let high = times.length - 2;
        while (low < high) {
          const mid = Math.floor((low + high + 1) / 2);
          if (times[mid] <= localTime) low = mid;
          else high = mid - 1;
        }
        targetIdx = low;
        const duration = times[targetIdx + 1] - times[targetIdx];
        subProgress = duration > 0 ? Math.max(0, Math.min(1, (localTime - times[targetIdx]) / duration)) : 1;
      } else {
        const progress = localTime / totalTime;
        targetIdx = Math.min(waypoints.length - 2, Math.floor(progress * (waypoints.length - 1)));
        subProgress = (progress * (waypoints.length - 1)) - targetIdx;
      }
      const wp1 = waypoints[targetIdx];
      const wp2 = waypoints[Math.min(targetIdx + 1, waypoints.length - 1)];
      targetLat = wp1.lat + (wp2.lat - wp1.lat) * subProgress;
      targetLon = wp1.lon + (wp2.lon - wp1.lon) * subProgress;
      const label = stageLabels[wp1.stage] || wp1.stage;
      currentStage = `${label} (${wp1.speed_ms} м/с)`;
    }

    marker.setLatLng([targetLat, targetLon]);

    const tag = document.querySelector(`#status_${markerId} .tag-stage`);
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
  await exportMission("kml", "kml");
}

async function exportGeoJson() {
  await exportMission("geojson", "geojson");
}

async function exportPlan() {
  await exportMission("qgc", "plan");
}

async function exportEmergencyPlan() {
  if (!currentMissionPlan) {
    showMissionMessage("Сначала рассчитайте полетное задание.");
    return;
  }
  const emergPlans = currentMissionPlan.drone_plans.filter(dp => dp.emergency_diversion);
  if (!emergPlans.length) {
    showMissionMessage("В текущем задании нет расчета аварийного схода на резервную площадку.");
    return;
  }
  try {
    const multiple = emergPlans.length > 1;
    const res = await fetch(`${API_BASE}/api/export/emergency-qgc`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(multiple ? emergPlans : emergPlans[0])
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const blob = await res.blob();
    downloadBlob(blob, multiple ? "emergency_missions.zip" : `emergency_${emergPlans[0].drone_id}.plan`);
  } catch (err) {
    showMissionMessage(`Экспорт аварийного плана не удался: ${err.message}`);
  }
}


async function exportMission(endpoint, extension) {
  if (!currentMissionPlan) {
    showMissionMessage("Сначала рассчитайте допустимый маршрут.");
    return;
  }
  try {
    const plans = currentMissionPlan.drone_plans;
    const multiple = plans.length > 1;
    const res = await fetch(`${API_BASE}/api/export/${endpoint}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(multiple ? plans : plans[0])
    });
    if (!res.ok) throw new Error(`HTTP ${res.status}`);
    const blob = await res.blob();
    downloadBlob(blob, multiple ? `missions_${extension}.zip` : `mission_${plans[0].drone_id}.${extension}`);
  } catch (err) {
    showMissionMessage(`Экспорт не удался: ${err.message}`);
  }
}

function downloadBlob(blob, filename) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  window.setTimeout(() => URL.revokeObjectURL(url), 30000);
}
