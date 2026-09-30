#!/usr/bin/env bash
set -e

# ==============================================================================
# HBA Standalone Runner & Web Studio (via Docker + CUDA)
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GIT_SHA="$(git -C "$SCRIPT_DIR" rev-parse --short=7 HEAD 2>/dev/null || echo "standalone")"
if ! git -C "$SCRIPT_DIR" diff --quiet HEAD 2>/dev/null; then
  GIT_SHA="${GIT_SHA}-dirty"
fi
IMAGE_TAG="${IMAGE_TAG:-$GIT_SHA}"
IMAGE_NAME="${HBA_IMAGE:-hba:standalone}"

function show_help() {
  cat << 'EOF'
Usage: ./run_hba.sh [options]

Mode Options:
  --web [port]          Launch HBA Web Studio & 3D Comparison Dashboard (default port: 8081)
  --view <path>         Read-only studio on a ground-truth store: a gt/ folder, a <name>.gt folder or a runs/<run-id> folder
                        (must be under DATA_DIR; use --port <n> to change the port, default: 8081)
  --rebuild             Rebuild the docker image before running
  -h, --help            Show this help message

Presets & Configurations:
  --config <name|path>  Config preset ('default' or 'legacy') or path to JSON config
  --preset <name>       Preset name ('default' [optimal] or 'legacy' [upstream paper])

Optimization Options (passed to hba_standalone):
  --glim <dir>          Path to GLIM output directory (e.g. ~/data/glim_results/aimbag)
  --input <dir>         Path to standard HBA directory (contains pcd/ and pose.json)
  -o, --output <dir>    Output results directory (default: /data/hba_results/<name>)
  --layers <int>        Number of hierarchical layers (default: 2 [legacy: 3])
  --threads <int>       Number of worker threads (default: 16)
  --voxel-size <float>  Initial voxel grid size in meters (default: 1.5 [legacy: 4.0])
  --downsample <float>  Point cloud downsample leaf size (default: 0.1)
  --eigen-ratio <float> Surface plane threshold ratio (default: 0.05 [legacy: 0.1])
  --reject-ratio <float> Residual outlier rejection ratio (default: 0.05)
  --max-iter <int>      Max Levenberg-Marquardt damping iterations (default: 10)
  --save-maps           Generate and save map_before.pcd and map_after.pcd
  --calc-mme            Calculate Mean Map Entropy (MME) before and after
  --zero-origin         Reference trajectory relative to first pose

Examples:
  # Launch interactive web studio on port 8081:
  ./run_hba.sh --web 8081

  # Browse stored GLIM/HBA runs read-only:
  ./run_hba.sh --view ~/data/2026-Sep-Slam-Start/gt --port 8082

  # Run headless optimization on GLIM results (uses optimal default):
  ./run_hba.sh --glim ~/data/glim_results/aimbag -o ~/data/hba_results/aimbag_refined --save-maps

  # Run headless optimization using legacy paper defaults:
  ./run_hba.sh --glim ~/data/glim_results/aimbag -o ~/data/hba_results/aimbag_legacy --config legacy

  # Run optimization on standard PCD directory:
  ./run_hba.sh --input ~/data/kitti07 -o ~/data/hba_results/kitti07_refined
EOF
  exit 0
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${DATA_DIR:-$HOME/data}"
mkdir -p "$DATA_DIR"

WEB_MODE=0
WEB_PORT=8081
VIEW_PATH=""
REBUILD=0
HBA_ARGS=()
INPUT_DATASET_NAME=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --web)
      WEB_MODE=1
      if [[ -n "$2" && "$2" =~ ^[0-9]+$ ]]; then
        WEB_PORT="$2"
        shift 2
      else
        shift
      fi
      ;;
    --view)
      if [[ -z "$2" ]]; then
        echo "Error: --view requires a path." >&2
        exit 1
      fi
      VIEW_PATH="$2"
      shift 2
      ;;
    --port)
      if [[ ! "$2" =~ ^[0-9]+$ ]]; then
        echo "Error: --port requires a number." >&2
        exit 1
      fi
      WEB_PORT="$2"
      shift 2
      ;;
    --rebuild)
      REBUILD=1
      shift
      ;;
    -h|--help)
      show_help
      ;;
    --config|--preset)
      ARG_VAL="$2"
      if [[ "$ARG_VAL" == "$SCRIPT_DIR/config"* ]]; then
        CONTAINER_PATH="/opt/hba/config${ARG_VAL#$SCRIPT_DIR/config}"
      elif [[ "$ARG_VAL" == "$HOME/data"* ]]; then
        CONTAINER_PATH="/data${ARG_VAL#$HOME/data}"
      else
        CONTAINER_PATH="$ARG_VAL"
      fi
      HBA_ARGS+=("$1" "$CONTAINER_PATH")
      shift 2
      ;;
    --glim)
      INPUT_DATASET_NAME="$2"
      # Remap host path to container path if inside DATA_DIR
      ARG_PATH="$2"
      if [[ "$ARG_PATH" == "$HOME/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#$HOME/data}"
      elif [[ "$ARG_PATH" == "/home/$USER/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#/home/$USER/data}"
      else
        CONTAINER_PATH="$ARG_PATH"
      fi
      HBA_ARGS+=("--glim" "$CONTAINER_PATH")
      shift 2
      ;;
    --input)
      INPUT_DATASET_NAME="$2"
      ARG_PATH="$2"
      if [[ "$ARG_PATH" == "$HOME/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#$HOME/data}"
      elif [[ "$ARG_PATH" == "/home/$USER/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#/home/$USER/data}"
      else
        CONTAINER_PATH="$ARG_PATH"
      fi
      HBA_ARGS+=("--input" "$CONTAINER_PATH")
      shift 2
      ;;
    -o|--output)
      ARG_PATH="$2"
      if [[ "$ARG_PATH" == "$HOME/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#$HOME/data}"
      elif [[ "$ARG_PATH" == "/home/$USER/data"* ]]; then
        CONTAINER_PATH="/data${ARG_PATH#/home/$USER/data}"
      else
        CONTAINER_PATH="$ARG_PATH"
      fi
      HBA_ARGS+=("-o" "$CONTAINER_PATH")
      shift 2
      ;;
    *)
      HBA_ARGS+=("$1")
      shift
      ;;
  esac
done

# Resolve --view path (must exist and be inside DATA_DIR) to its container path
if [[ -n "$VIEW_PATH" ]]; then
  if [[ ! -d "$VIEW_PATH" ]]; then
    echo "Error: --view path does not exist or is not a directory: $VIEW_PATH" >&2
    exit 1
  fi
  VIEW_ABS="$(cd "$VIEW_PATH" && pwd -P)"
  DATA_ABS="$(cd "$DATA_DIR" && pwd -P)"
  if [[ "$VIEW_ABS" != "$DATA_ABS" && "$VIEW_ABS" != "$DATA_ABS"/* ]]; then
    echo "Error: --view path must be inside DATA_DIR ($DATA_ABS): $VIEW_ABS" >&2
    exit 1
  fi
  VIEW_CONTAINER_PATH="/data${VIEW_ABS#$DATA_ABS}"
fi

# 1. GPU & TTY Detection
DOCKER_GPU_FLAGS=""
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  DOCKER_GPU_FLAGS="--gpus all"
fi

DOCKER_TTY_FLAGS=""
if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY_FLAGS="-it"
fi

# 2. Check / Build / Pull Image
IMAGE_EXISTS=$(docker images -q "$IMAGE_NAME" 2>/dev/null || true)
if [[ -z "$IMAGE_EXISTS" && "$REBUILD" -eq 0 ]]; then
  # Try pulling from GHCR if available
  if docker pull "ghcr.io/aim-bagr/hba:${IMAGE_TAG}" 2>/dev/null; then
    IMAGE_NAME="ghcr.io/aim-bagr/hba:${IMAGE_TAG}"
    IMAGE_EXISTS=1
  elif docker pull "ghcr.io/aim-bagr/hba:latest" 2>/dev/null; then
    IMAGE_NAME="ghcr.io/aim-bagr/hba:latest"
    IMAGE_EXISTS=1
  elif docker images -q "hba:standalone" 2>/dev/null | grep -q .; then
    IMAGE_NAME="hba:standalone"
    IMAGE_EXISTS=1
  fi
fi

if [[ -z "$IMAGE_EXISTS" || "$REBUILD" -eq 1 ]]; then
  echo "Building Docker image: $IMAGE_NAME..."
  FULL_GIT_SHA="$(git -C "$SCRIPT_DIR" rev-parse HEAD 2>/dev/null || echo "unknown")"
  EXTRA_TAG=(-t "hba:standalone")
  # A custom HBA_IMAGE must never retag hba:standalone
  [[ -n "${HBA_IMAGE:-}" ]] && EXTRA_TAG=()
  docker build --build-arg GIT_SHA="$FULL_GIT_SHA" -t "$IMAGE_NAME" "${EXTRA_TAG[@]}" "$SCRIPT_DIR"
fi

# 3a. Read-only View Mode (stored results under DATA_DIR)
if [[ -n "$VIEW_PATH" ]]; then
  VIEW_CLEAN="$(basename "$VIEW_ABS" | tr -c 'A-Za-z0-9_.\n-' '-')"
  VIEW_NAME="hba-view-${VIEW_CLEAN}"
  # Replace only a container with this same name
  docker rm -f "$VIEW_NAME" >/dev/null 2>&1 || true

  echo "=========================================================="
  echo "   Launching HBA Web Studio (read-only view mode)         "
  echo "=========================================================="
  echo "  Web Interface URL : http://localhost:${WEB_PORT}"
  echo "  Viewing           : ${VIEW_ABS} -> ${VIEW_CONTAINER_PATH} (read-only)"
  echo "  Container Name    : ${VIEW_NAME}"
  echo "  Image Name        : ${IMAGE_NAME}"
  echo "=========================================================="

  docker run --rm \
    --name "$VIEW_NAME" \
    --label slam-eval.tool=hba \
    --label slam-eval.service=view \
    --user "$(id -u):$(id -g)" \
    -e HBA_VIEW_PATH="$VIEW_CONTAINER_PATH" \
    -p "${WEB_PORT}:${WEB_PORT}" \
    -v "${DATA_DIR}:/data:ro" \
    -v "${SCRIPT_DIR}/config:/opt/hba/config:ro" \
    -v "${SCRIPT_DIR}/web:/opt/hba/web:ro" \
    -v "${SCRIPT_DIR}/server:/opt/hba/server:ro" \
    -w /opt/hba \
    --entrypoint python3 \
    "$IMAGE_NAME" \
    -m uvicorn server.main:app --host 0.0.0.0 --port "${WEB_PORT}" --reload
  exit 0
fi

# 3. Web Studio Mode
if [[ "$WEB_MODE" -eq 1 ]]; then
  # Check if previous container is occupying the port or name
  EXISTING_CONTAINER=$(docker ps --filter "publish=${WEB_PORT}" -q 2>/dev/null || true)
  if [[ -n "$EXISTING_CONTAINER" ]]; then
    echo "Stopping previous container on port ${WEB_PORT} ($EXISTING_CONTAINER)..."
    docker stop "$EXISTING_CONTAINER" >/dev/null 2>&1 || true
    sleep 1
  fi
  docker rm -f hba-web hba_web_studio >/dev/null 2>&1 || true

  echo "=========================================================="
  echo "   Launching HBA Web Studio & Dual-Layer Visualizer       "
  echo "=========================================================="
  echo "  Web Interface URL : http://localhost:${WEB_PORT}"
  echo "  Mapped Data Root  : ${DATA_DIR} -> /data"
  echo "  GPU Acceleration  : ${DOCKER_GPU_FLAGS:-Disabled (CPU)}"
  echo "  Container Name    : hba-web"
  echo "  Image Name        : ${IMAGE_NAME}"
  echo "=========================================================="

  docker run --rm \
    --name hba-web \
    --label slam-eval.tool=hba \
    --label slam-eval.service=web \
    $DOCKER_GPU_FLAGS \
    --user "$(id -u):$(id -g)" \
    --ipc=host \
    -p "${WEB_PORT}:${WEB_PORT}" \
    -v "${DATA_DIR}:/data:rw" \
    -v "${SCRIPT_DIR}/config:/opt/hba/config:ro" \
    -v "${SCRIPT_DIR}/web:/opt/hba/web:ro" \
    -v "${SCRIPT_DIR}/server:/opt/hba/server:ro" \
    -w /opt/hba \
    --entrypoint python3 \
    "$IMAGE_NAME" \
    -m uvicorn server.main:app --host 0.0.0.0 --port "${WEB_PORT}" --reload
  exit 0
fi

# 4. Interactive zenity fallback if run without arguments
if [[ ${#HBA_ARGS[@]} -eq 0 ]] && command -v zenity &>/dev/null && [[ -n "$DISPLAY" ]]; then
  SELECTED_DIR=$(zenity --file-selection --directory --title="Select GLIM results or HBA dataset directory" 2>/dev/null || true)
  if [[ -n "$SELECTED_DIR" ]]; then
    INPUT_DATASET_NAME="$SELECTED_DIR"
    if [[ "$SELECTED_DIR" == "$HOME/data"* ]]; then
      CONTAINER_PATH="/data${SELECTED_DIR#$HOME/data}"
    else
      CONTAINER_PATH="$SELECTED_DIR"
    fi
    HBA_ARGS+=("--glim" "$CONTAINER_PATH" "-o" "/data/hba_results/run_$(date +%Y%m%d_%H%M%S)" "--save-maps")
  else
    show_help
  fi
fi

if [[ ${#HBA_ARGS[@]} -eq 0 ]]; then
  show_help
fi

# 5. Headless Optimization CLI Run
DATASET_CLEAN="$(basename "${INPUT_DATASET_NAME%/}" | tr -c 'A-Za-z0-9_.\n-' '-')"
RUN_TS="$(date +%Y%m%d-%H%M)"
CONTAINER_NAME="hba-${DATASET_CLEAN:-run}-${RUN_TS}"

echo "Executing HBA Standalone Runner inside container ($CONTAINER_NAME)..."
docker run --rm $DOCKER_TTY_FLAGS \
  --name "$CONTAINER_NAME" \
  --label slam-eval.tool=hba \
  --label slam-eval.dataset="${DATASET_CLEAN:-unknown}" \
  --label slam-eval.run-id="${RUN_TS}" \
  $DOCKER_GPU_FLAGS \
  --user "$(id -u):$(id -g)" \
  --ipc=host \
  -v "${DATA_DIR}:/data:rw" \
  -v "${SCRIPT_DIR}/config:/opt/hba/config:ro" \
  "$IMAGE_NAME" "${HBA_ARGS[@]}"
