// HBA Studio Web Interface & Dual-Layer Three.js Visualizer
let ws = null;
let currentRunName = null;
let comparisonMode = 'after'; // 'before' | 'after' | 'overlay'
let autoScroll = true;
let totalLogLines = 0;

// Three.js State
let threeScene, threeCamera, threeRenderer, threeControls;
let currentSubmaps = []; // Metadata array
let rawPointsCache = new Map(); // submapId -> Float32Array
let submapObjectsBefore = new Map(); // submapId -> THREE.Points
let submapObjectsAfter = new Map();  // submapId -> THREE.Points
let trajLineBefore = null;
let trajLineAfter = null;
let trajLineDense = null;
let activeMarker = null;

let isMapVisible = true;
let isTrajVisible = true;
let isDenseTrajVisible = false;
let pointSize = 2.0;
let activeSubmapId = 0;
let isIsolated = false;
let isPlaying = false;
let playInterval = null;

// DOM Elements
const statusDot = document.getElementById('status-dot');
const statusText = document.getElementById('status-text');
const metricStage = document.getElementById('metric-stage');
const metricResidual = document.getElementById('metric-residual');
const metricDrop = document.getElementById('metric-drop');
const metricElapsed = document.getElementById('metric-elapsed');

const toggleLauncherBtn = document.getElementById('toggle-launcher-btn');
const closeLauncherBtn = document.getElementById('close-launcher-btn');
const launcherModal = document.getElementById('launcher-modal');
const launcherForm = document.getElementById('launcher-form');
const inputGlimSelect = document.getElementById('input-glim-select');
const inputPresetSelect = document.getElementById('input-preset-select');
const inputRunName = document.getElementById('input-run-name');
const startJobBtn = document.getElementById('start-job-btn');
const headerStopBtn = document.getElementById('header-stop-btn');

const toggleRunsBtn = document.getElementById('toggle-runs-btn');
const closeRunsBtn = document.getElementById('close-runs-btn');
const runsModal = document.getElementById('runs-modal');
const runsListContainer = document.getElementById('runs-list-container');
const headerRunSelect = document.getElementById('header-run-select');

const toggleConsoleBtn = document.getElementById('toggle-console-btn');
const closeConsoleBtn = document.getElementById('close-console-btn');
const consoleDrawer = document.getElementById('console-drawer');
const consoleBadgeCount = document.getElementById('console-badge-count');
const logConsole = document.getElementById('log-console');
const logCount = document.getElementById('log-count');
const autoscrollToggle = document.getElementById('autoscroll-toggle');
const clearLogsBtn = document.getElementById('clear-logs-btn');

const modeBeforeBtn = document.getElementById('mode-before-btn');
const modeAfterBtn = document.getElementById('mode-after-btn');
const modeOverlayBtn = document.getElementById('mode-overlay-btn');

const toggleMap = document.getElementById('toggle-map');
const toggleTraj = document.getElementById('toggle-traj');
const toggleDenseTraj = document.getElementById('toggle-dense-traj');
const paramPtSize = document.getElementById('param-pt-size');
const btnViewTop = document.getElementById('btn-view-top');
const btnViewIso = document.getElementById('btn-view-iso');

const scrubberSlider = document.getElementById('scrubber-slider');
const scrubberText = document.getElementById('scrubber-text');
const scrubberPlayBtn = document.getElementById('scrubber-play-btn');
const scrubberIsolateBtn = document.getElementById('scrubber-isolate-btn');
const viewportPlaceholder = document.getElementById('viewport-placeholder');

// Initialize on Load
window.addEventListener('DOMContentLoaded', () => {
  if (window.lucide) lucide.createIcons();
  initThree();
  initWebSocket();
  setupUIEvents();
  refreshGlimRuns();
  refreshHbaRuns();
});

// Setup Three.js Canvas
function initThree() {
  const container = document.getElementById('viewport-3d');
  const width = window.innerWidth;
  const height = window.innerHeight;

  threeScene = new THREE.Scene();
  threeScene.background = new THREE.Color(0x020617);

  // Robotics coordinate convention: Z-Up
  threeCamera = new THREE.PerspectiveCamera(55, width / height, 0.1, 5000);
  threeCamera.up.set(0, 0, 1);
  threeCamera.position.set(-20, -30, 25);

  threeRenderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'high-performance' });
  threeRenderer.setSize(width, height);
  threeRenderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  container.appendChild(threeRenderer.domElement);

  threeControls = new THREE.OrbitControls(threeCamera, threeRenderer.domElement);
  threeControls.enableDamping = true;
  threeControls.dampingFactor = 0.08;
  threeControls.screenSpacePanning = true;
  threeControls.target.set(0, 0, 0);

  // Z-up Grid & Coordinate Axes
  const grid = new THREE.GridHelper(200, 40, 0x1e293b, 0x0f172a);
  grid.rotation.x = Math.PI / 2;
  threeScene.add(grid);

  const axes = new THREE.AxesHelper(3.0);
  threeScene.add(axes);

  // Active Submap Marker Box
  const markerGeom = new THREE.BoxGeometry(4.0, 4.0, 2.5);
  const markerMat = new THREE.LineBasicMaterial({ color: 0x38bdf8, linewidth: 2, transparent: true, opacity: 0.85 });
  activeMarker = new THREE.LineSegments(new THREE.WireframeGeometry(markerGeom), markerMat);
  activeMarker.visible = false;
  threeScene.add(activeMarker);

  window.addEventListener('resize', onResize);

  function animate() {
    requestAnimationFrame(animate);
    threeControls.update();
    threeRenderer.render(threeScene, threeCamera);
  }
  animate();
}

function onResize() {
  if (!threeCamera || !threeRenderer) return;
  const w = window.innerWidth;
  const h = window.innerHeight;
  threeCamera.aspect = w / h;
  threeCamera.updateProjectionMatrix();
  threeRenderer.setSize(w, h);
}

// WebSocket Live Log Stream
function initWebSocket() {
  const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
  const wsUrl = `${protocol}//${window.location.host}/ws/logs`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    statusDot.className = 'h-2 w-2 rounded-full bg-emerald-500';
    statusText.textContent = 'Service Ready';
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      if (msg.type === 'init') {
        updateJobStatus(msg.job);
        if (msg.logs) msg.logs.forEach(appendLog);
      } else if (msg.type === 'status' || msg.type === 'progress') {
        updateJobStatus(msg.data);
      } else if (msg.type === 'log') {
        appendLog(msg.line);
      }
    } catch (e) {}
  };

  ws.onclose = () => {
    statusDot.className = 'h-2 w-2 rounded-full bg-slate-500';
    statusText.textContent = 'Disconnected';
    setTimeout(initWebSocket, 2000);
  };
}

function updateJobStatus(job) {
  if (!job) return;

  if (job.state === 'running') {
    statusDot.className = 'h-2 w-2 rounded-full bg-amber-400 animate-pulse';
    statusText.textContent = 'Optimizing';
    headerStopBtn.classList.remove('hidden');
    metricStage.textContent = job.layer_info || 'Running';
    metricStage.className = 'font-bold text-amber-400';
  } else if (job.state === 'completed') {
    statusDot.className = 'h-2 w-2 rounded-full bg-emerald-500';
    statusText.textContent = 'Completed';
    headerStopBtn.classList.add('hidden');
    metricStage.textContent = 'Done';
    metricStage.className = 'font-bold text-emerald-400';
    refreshHbaRuns();
    if (job.run_name && job.run_name !== currentRunName) {
      loadHbaRun(job.run_name);
    }
  } else if (job.state === 'failed') {
    statusDot.className = 'h-2 w-2 rounded-full bg-rose-500';
    statusText.textContent = 'Failed';
    headerStopBtn.classList.add('hidden');
    metricStage.textContent = 'Error';
    metricStage.className = 'font-bold text-rose-400';
  } else {
    statusDot.className = 'h-2 w-2 rounded-full bg-slate-500';
    statusText.textContent = 'Idle';
    headerStopBtn.classList.add('hidden');
    metricStage.textContent = 'Idle';
    metricStage.className = 'font-bold text-slate-400';
  }

  if (job.current_residual > 0) {
    metricResidual.textContent = job.current_residual.toFixed(4);
  }
  if (job.residual_reduction_pct > 0) {
    metricDrop.textContent = `${job.residual_reduction_pct.toFixed(2)}%`;
  }
  if (job.elapsed_sec > 0) {
    metricElapsed.textContent = `${job.elapsed_sec.toFixed(1)}s`;
  }
}

function appendLog(line) {
  totalLogLines++;
  logCount.textContent = `${totalLogLines} lines`;
  consoleBadgeCount.textContent = totalLogLines;
  consoleBadgeCount.classList.remove('hidden');

  const div = document.createElement('div');
  div.textContent = line;

  if (line.includes('[progress]')) {
    div.className = 'log-line-progress';
  } else if (line.includes('[layer]')) {
    div.className = 'log-line-layer';
  } else if (line.includes('[pgo]')) {
    div.className = 'log-line-pgo';
  } else if (line.toLowerCase().includes('error')) {
    div.className = 'log-line-err';
  } else {
    div.className = 'log-line-info';
  }

  logConsole.appendChild(div);

  if (autoScroll) {
    logConsole.scrollTop = logConsole.scrollHeight;
  }
}

// UI Event Handlers
function setupUIEvents() {
  toggleLauncherBtn.addEventListener('click', () => {
    runsModal.classList.add('hidden');
    launcherModal.classList.toggle('hidden');
  });
  closeLauncherBtn.addEventListener('click', () => launcherModal.classList.add('hidden'));

  toggleRunsBtn.addEventListener('click', () => {
    launcherModal.classList.add('hidden');
    runsModal.classList.toggle('hidden');
    refreshHbaRuns();
  });
  closeRunsBtn.addEventListener('click', () => runsModal.classList.add('hidden'));

  toggleConsoleBtn.addEventListener('click', () => {
    consoleDrawer.classList.toggle('hidden');
    consoleBadgeCount.classList.add('hidden');
  });
  closeConsoleBtn.addEventListener('click', () => consoleDrawer.classList.add('hidden'));

  clearLogsBtn.addEventListener('click', () => {
    logConsole.innerHTML = '';
    totalLogLines = 0;
    logCount.textContent = '0 lines';
  });

  autoscrollToggle.addEventListener('change', (e) => {
    autoScroll = e.target.checked;
  });

  headerStopBtn.addEventListener('click', async () => {
    await fetch('/api/jobs/stop', { method: 'POST' });
  });

  if (inputPresetSelect) {
    inputPresetSelect.addEventListener('change', () => {
      const preset = inputPresetSelect.value;
      if (preset === 'legacy') {
        document.getElementById('param-layers').value = '3';
        document.getElementById('param-voxel-size').value = '4.0';
        document.getElementById('param-eigen-ratio').value = '0.1';
      } else {
        document.getElementById('param-layers').value = '2';
        document.getElementById('param-voxel-size').value = '1.5';
        document.getElementById('param-eigen-ratio').value = '0.05';
      }
    });
  }

  // Launcher Form Submit
  launcherForm.addEventListener('submit', async (e) => {
    e.preventDefault();
    const glimPath = inputGlimSelect.value;
    if (!glimPath) return alert('Please select an input GLIM run.');

    const reqData = {
      source_type: 'glim',
      dataset_path: glimPath,
      run_name: inputRunName.value.trim() || null,
      config_preset: inputPresetSelect ? inputPresetSelect.value : 'default',
      total_layer_num: parseInt(document.getElementById('param-layers').value, 10),
      thread_num: parseInt(document.getElementById('param-threads').value, 10),
      voxel_size: parseFloat(document.getElementById('param-voxel-size').value),
      downsample_size: parseFloat(document.getElementById('param-downsample').value),
      eigen_ratio: parseFloat(document.getElementById('param-eigen-ratio').value),
      max_iter: parseInt(document.getElementById('param-max-iter').value, 10),
      save_maps: document.getElementById('flag-save-maps').checked,
      calc_mme: document.getElementById('flag-calc-mme').checked,
      zero_origin: document.getElementById('flag-zero-origin').checked
    };

    launcherModal.classList.add('hidden');
    consoleDrawer.classList.remove('hidden');

    try {
      const res = await fetch('/api/jobs/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(reqData)
      });
      if (!res.ok) {
        const err = await res.json();
        alert(`Failed to launch job: ${err.detail || 'Unknown error'}`);
      }
    } catch (err) {
      alert(`Network error: ${err.message}`);
    }
  });

  // Comparison Mode Toggles
  modeBeforeBtn.addEventListener('click', () => setComparisonMode('before'));
  modeAfterBtn.addEventListener('click', () => setComparisonMode('after'));
  modeOverlayBtn.addEventListener('click', () => setComparisonMode('overlay'));

  // Point Cloud & Trajectory Toggles
  toggleMap.addEventListener('change', (e) => {
    isMapVisible = e.target.checked;
    updateVisibility();
  });
  toggleTraj.addEventListener('change', (e) => {
    isTrajVisible = e.target.checked;
    if (trajLineBefore) trajLineBefore.visible = isTrajVisible && (comparisonMode === 'before' || comparisonMode === 'overlay');
    if (trajLineAfter) trajLineAfter.visible = isTrajVisible && (comparisonMode === 'after' || comparisonMode === 'overlay');
  });
  if (toggleDenseTraj) {
    toggleDenseTraj.addEventListener('change', (e) => {
      isDenseTrajVisible = e.target.checked;
      if (trajLineDense) trajLineDense.visible = isDenseTrajVisible;
    });
  }

  paramPtSize.addEventListener('input', (e) => {
    pointSize = parseFloat(e.target.value);
    submapObjectsBefore.forEach(pts => pts.material.size = pointSize);
    submapObjectsAfter.forEach(pts => pts.material.size = pointSize);
  });

  btnViewTop.addEventListener('click', () => {
    threeCamera.position.set(0, 0, 60);
    threeControls.target.set(0, 0, 0);
  });
  btnViewIso.addEventListener('click', () => {
    threeCamera.position.set(-20, -30, 25);
    threeControls.target.set(0, 0, 0);
  });

  // Scrubber Slider
  scrubberSlider.addEventListener('input', (e) => {
    activeSubmapId = parseInt(e.target.value, 10);
    updateActiveSubmap();
  });

  scrubberIsolateBtn.addEventListener('click', () => {
    isIsolated = !isIsolated;
    scrubberIsolateBtn.className = isIsolated
      ? 'px-2 py-1 rounded bg-emerald-600 text-white font-semibold border border-emerald-500'
      : 'px-2 py-1 rounded bg-slate-800 text-slate-300 border border-slate-700';
    updateVisibility();
  });

  scrubberPlayBtn.addEventListener('click', togglePlayScrubber);

  headerRunSelect.addEventListener('change', (e) => {
    if (e.target.value) loadHbaRun(e.target.value);
  });
}

function setComparisonMode(mode) {
  comparisonMode = mode;

  modeBeforeBtn.className = (mode === 'before')
    ? 'px-2.5 py-1 rounded-md transition bg-rose-600 text-white font-semibold flex items-center gap-1.5 shadow-sm'
    : 'px-2.5 py-1 rounded-md transition text-slate-400 hover:text-white flex items-center gap-1.5';

  modeAfterBtn.className = (mode === 'after')
    ? 'px-2.5 py-1 rounded-md transition bg-emerald-600 text-white font-semibold flex items-center gap-1.5 shadow-sm'
    : 'px-2.5 py-1 rounded-md transition text-slate-400 hover:text-white flex items-center gap-1.5';

  modeOverlayBtn.className = (mode === 'overlay')
    ? 'px-2.5 py-1 rounded-md transition bg-gradient-to-r from-rose-600 to-emerald-600 text-white font-semibold flex items-center gap-1.5 shadow-sm'
    : 'px-2.5 py-1 rounded-md transition text-slate-400 hover:text-white flex items-center gap-1.5';

  updateVisibility();
}

function updateVisibility() {
  const showBefore = (comparisonMode === 'before' || comparisonMode === 'overlay');
  const showAfter = (comparisonMode === 'after' || comparisonMode === 'overlay');

  submapObjectsBefore.forEach((pts, id) => {
    pts.visible = isMapVisible && showBefore && (!isIsolated || id === activeSubmapId);
  });
  submapObjectsAfter.forEach((pts, id) => {
    pts.visible = isMapVisible && showAfter && (!isIsolated || id === activeSubmapId);
  });

  if (trajLineBefore) trajLineBefore.visible = isTrajVisible && showBefore;
  if (trajLineAfter) trajLineAfter.visible = isTrajVisible && showAfter;
  if (trajLineDense) trajLineDense.visible = isDenseTrajVisible;
}

function updateActiveSubmap() {
  scrubberText.textContent = `${activeSubmapId} / ${Math.max(0, currentSubmaps.length - 1)}`;

  if (currentSubmaps.length > activeSubmapId) {
    const sm = currentSubmaps[activeSubmapId];
    const pos = (comparisonMode === 'before') ? sm.pos_before : sm.pos_after;
    if (activeMarker) {
      activeMarker.position.set(pos[0], pos[1], pos[2]);
      activeMarker.visible = true;
    }
  }

  updateVisibility();
}

function togglePlayScrubber() {
  isPlaying = !isPlaying;
  scrubberPlayBtn.innerHTML = isPlaying
    ? '<i data-lucide="pause" class="w-3.5 h-3.5 fill-current"></i>'
    : '<i data-lucide="play" class="w-3.5 h-3.5 fill-current"></i>';
  if (window.lucide) lucide.createIcons();

  if (isPlaying) {
    playInterval = setInterval(() => {
      activeSubmapId = (activeSubmapId + 1) % currentSubmaps.length;
      scrubberSlider.value = activeSubmapId;
      updateActiveSubmap();
    }, 150);
  } else {
    clearInterval(playInterval);
  }
}

// Data Fetching & Submap Rendering
async function refreshGlimRuns() {
  try {
    const res = await fetch('/api/glim-runs');
    if (!res.ok) return;
    const runs = await res.json();
    inputGlimSelect.innerHTML = '';

    if (runs.length === 0) {
      inputGlimSelect.innerHTML = '<option value="">No GLIM runs found in /data/glim_results</option>';
      return;
    }

    runs.forEach(r => {
      const opt = document.createElement('option');
      opt.value = r.path;
      opt.textContent = `${r.name} (${r.submaps_count} submaps, ${r.size_human})`;
      inputGlimSelect.appendChild(opt);
    });
  } catch (e) {}
}

async function refreshHbaRuns() {
  try {
    const res = await fetch('/api/runs');
    if (!res.ok) return;
    const runs = await res.json();

    runsListContainer.innerHTML = '';
    headerRunSelect.innerHTML = '<option value="">Select HBA Run...</option>';

    if (runs.length === 0) {
      runsListContainer.innerHTML = '<div class="text-slate-500 text-center py-4">No completed HBA runs yet.</div>';
      return;
    }

    runs.forEach(r => {
      // Header dropdown
      const opt = document.createElement('option');
      opt.value = r.name;
      opt.textContent = `${r.name} (${r.submaps_count} scans)`;
      if (r.name === currentRunName) opt.selected = true;
      headerRunSelect.appendChild(opt);

      // Modal List Card
      const card = document.createElement('div');
      card.className = 'bg-slate-950 border border-slate-800 rounded-lg p-2.5 hover:border-slate-700 transition flex items-center justify-between';
      card.innerHTML = `
        <div class="flex-1 min-w-0 pr-2">
          <div class="font-semibold text-slate-200 truncate">${r.name}</div>
          <div class="text-[10px] text-slate-500 flex items-center gap-2 mt-0.5">
            <span>${r.submaps_count} submaps</span>
            <span>•</span>
            <span class="text-emerald-400 font-medium">-${r.residual_reduction_pct.toFixed(1)}% res</span>
            <span>•</span>
            <span>${r.elapsed_sec.toFixed(1)}s</span>
            ${r.has_dense_traj ? '<span>•</span><span class="text-cyan-400 font-medium">20Hz LiDAR</span>' : ''}
          </div>
        </div>
        <button class="px-2 py-1 bg-emerald-600/30 hover:bg-emerald-600/50 text-emerald-300 rounded text-[11px] font-medium transition" onclick="loadHbaRun('${r.name}')">
          Load
        </button>
      `;
      runsListContainer.appendChild(card);
    });

    // Auto load first run if nothing loaded
    if (!currentRunName && runs.length > 0) {
      loadHbaRun(runs[0].name);
    }
  } catch (e) {}
}

async function loadHbaRun(runName) {
  currentRunName = runName;
  runsModal.classList.add('hidden');
  viewportPlaceholder.classList.add('hidden');
  headerRunSelect.value = runName;

  // Clear existing point clouds and trajectories
  submapObjectsBefore.forEach(pts => threeScene.remove(pts));
  submapObjectsAfter.forEach(pts => threeScene.remove(pts));
  submapObjectsBefore.clear();
  submapObjectsAfter.clear();
  rawPointsCache.clear();

  if (trajLineBefore) { threeScene.remove(trajLineBefore); trajLineBefore = null; }
  if (trajLineAfter) { threeScene.remove(trajLineAfter); trajLineAfter = null; }
  if (trajLineDense) { threeScene.remove(trajLineDense); trajLineDense = null; }
  if (toggleDenseTraj) {
    toggleDenseTraj.disabled = true;
    toggleDenseTraj.checked = false;
    toggleDenseTraj.parentElement.classList.add('opacity-40');
    isDenseTrajVisible = false;
  }

  // Load Submaps Metadata
  const smRes = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps`);
  if (!smRes.ok) return alert('Failed to load submap metadata');
  currentSubmaps = await smRes.json();

  scrubberSlider.max = Math.max(0, currentSubmaps.length - 1);
  scrubberSlider.value = 0;
  activeSubmapId = 0;
  scrubberText.textContent = `0 / ${currentSubmaps.length - 1}`;

  // Load Trajectories
  loadTrajectory(runName, false); // Before
  loadTrajectory(runName, true);  // After
  loadDenseTrajectory(runName);   // Dense (20Hz LiDAR)

  // Center camera on dataset bounding box
  if (currentSubmaps.length > 0) {
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    currentSubmaps.forEach(s => {
      const p = s.pos_before;
      minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
    });
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    threeControls.target.set(cx, cy, 0);
    threeCamera.position.set(cx - 30, cy - 40, 30);
  }

  // Progressively stream submaps
  streamSubmaps(runName);
}

async function loadTrajectory(runName, isAfter) {
  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/trajectory?opt=${isAfter}`);
    if (!res.ok) return;
    const text = await res.text();
    const lines = text.trim().split('\n');

    const points = [];
    lines.forEach(l => {
      const parts = l.trim().split(/\s+/);
      if (parts.length >= 8) {
        points.push(new THREE.Vector3(parseFloat(parts[1]), parseFloat(parts[2]), parseFloat(parts[3])));
      }
    });

    if (points.length < 2) return;

    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const color = isAfter ? 0x10b981 : 0xf43f5e; // Green vs Red
    const mat = new THREE.LineBasicMaterial({ color: color, linewidth: 3 });
    const line = new THREE.Line(geom, mat);

    threeScene.add(line);
    if (isAfter) trajLineAfter = line;
    else trajLineBefore = line;

    updateVisibility();
  } catch (e) {}
}

async function loadDenseTrajectory(runName) {
  try {
    const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/trajectory?dense=true`);
    if (!res.ok) {
      if (toggleDenseTraj) {
        toggleDenseTraj.disabled = true;
        toggleDenseTraj.checked = false;
        toggleDenseTraj.parentElement.classList.add('opacity-40');
        isDenseTrajVisible = false;
      }
      return;
    }
    const text = await res.text();
    const lines = text.trim().split('\n');

    const points = [];
    lines.forEach(l => {
      const parts = l.trim().split(/\s+/);
      if (parts.length >= 8) {
        points.push(new THREE.Vector3(parseFloat(parts[1]), parseFloat(parts[2]), parseFloat(parts[3])));
      }
    });

    if (points.length < 2) return;

    const geom = new THREE.BufferGeometry().setFromPoints(points);
    const mat = new THREE.LineBasicMaterial({ color: 0x06b6d4, linewidth: 2 }); // Cyan (dense 20Hz LiDAR)
    const line = new THREE.Line(geom, mat);

    threeScene.add(line);
    trajLineDense = line;

    if (toggleDenseTraj) {
      toggleDenseTraj.disabled = false;
      toggleDenseTraj.checked = true;
      toggleDenseTraj.parentElement.classList.remove('opacity-40');
      isDenseTrajVisible = true;
    }
    trajLineDense.visible = isDenseTrajVisible;
  } catch (e) {
    if (toggleDenseTraj) {
      toggleDenseTraj.disabled = true;
      toggleDenseTraj.checked = false;
      toggleDenseTraj.parentElement.classList.add('opacity-40');
      isDenseTrajVisible = false;
    }
  }
}

async function streamSubmaps(runName) {
  for (let i = 0; i < currentSubmaps.length; i++) {
    if (currentRunName !== runName) break;
    await loadSubmap(runName, currentSubmaps[i]);
    // Small yield to keep UI responsive
    if (i % 8 === 0) await new Promise(r => setTimeout(r, 20));
  }
}

async function loadSubmap(runName, sm) {
  let positions = rawPointsCache.get(sm.id);

  if (!positions) {
    try {
      const res = await fetch(`/api/runs/${encodeURIComponent(runName)}/submaps/${sm.id}/points`);
      if (!res.ok) return;
      const buf = await res.arrayBuffer();
      positions = new Float32Array(buf);
      rawPointsCache.set(sm.id, positions);
    } catch (e) {
      return;
    }
  }

  // 1. Create Before Object (Warm Red/Orange Gradient)
  const geomB = new THREE.BufferGeometry();
  geomB.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  applyPose(geomB, sm.pose_before_raw);
  applyColors(geomB, 0xf43f5e, 0xfb923c); // Red to Orange

  const matB = new THREE.PointsMaterial({
    size: pointSize,
    vertexColors: true,
    sizeAttenuation: false,
    transparent: true,
    opacity: 0.75
  });
  const ptsB = new THREE.Points(geomB, matB);
  threeScene.add(ptsB);
  submapObjectsBefore.set(sm.id, ptsB);

  // 2. Create After Object (Cool Cyan/Green Gradient)
  const geomA = new THREE.BufferGeometry();
  geomA.setAttribute('position', new THREE.BufferAttribute(new Float32Array(positions), 3));
  applyPose(geomA, sm.pose_after_raw);
  applyColors(geomA, 0x10b981, 0x38bdf8); // Emerald to Sky Blue

  const matA = new THREE.PointsMaterial({
    size: pointSize,
    vertexColors: true,
    sizeAttenuation: false,
    transparent: true,
    opacity: 0.85
  });
  const ptsA = new THREE.Points(geomA, matA);
  threeScene.add(ptsA);
  submapObjectsAfter.set(sm.id, ptsA);

  updateVisibility();
}

function applyPose(geom, rawPose) {
  if (!rawPose || rawPose.length < 7) return;
  // Format: tx ty tz qw qx qy qz
  const [tx, ty, tz, qw, qx, qy, qz] = rawPose;
  const q = new THREE.Quaternion(qx, qy, qz, qw);
  const p = new THREE.Vector3(tx, ty, tz);

  const mat = new THREE.Matrix4().compose(p, q, new THREE.Vector3(1, 1, 1));
  geom.applyMatrix4(mat);
}

function applyColors(geom, hexColor1, hexColor2) {
  const count = geom.attributes.position.count;
  const colors = new Float32Array(count * 3);
  const c1 = new THREE.Color(hexColor1);
  const c2 = new THREE.Color(hexColor2);

  const pos = geom.attributes.position.array;
  let minZ = Infinity, maxZ = -Infinity;
  for (let i = 0; i < count; i++) {
    const z = pos[i * 3 + 2];
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }
  const span = Math.max(0.01, maxZ - minZ);

  for (let i = 0; i < count; i++) {
    const z = pos[i * 3 + 2];
    const t = (z - minZ) / span;
    const c = c1.clone().lerp(c2, t);
    colors[i * 3 + 0] = c.r;
    colors[i * 3 + 1] = c.g;
    colors[i * 3 + 2] = c.b;
  }
  geom.setAttribute('color', new THREE.BufferAttribute(colors, 3));
}
