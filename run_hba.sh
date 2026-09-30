#!/usr/bin/env bash
set -e

# ==============================================================================
# HBA Standalone Runner & Web Studio (via Docker + CUDA)
# ==============================================================================

IMAGE_NAME="hba:standalone"

function show_help() {
  cat << 'EOF'
Usage: ./run_hba.sh [options]

Mode Options:
  --web [port]          Launch HBA Web Studio & 3D Comparison Dashboard (default port: 8081)
  --rebuild             Rebuild the docker image before running
  -h, --help            Show this help message

Optimization Options (passed to hba_standalone):
  --glim <dir>          Path to GLIM output directory (e.g. ~/data/glim_results/aimbag)
  --input <dir>         Path to standard HBA directory (contains pcd/ and pose.json)
  -o, --output <dir>    Output results directory (default: /data/hba_results/<name>)
  --layers <int>        Number of hierarchical layers (default: 3)
  --threads <int>       Number of worker threads (default: 16)
  --voxel-size <float>  Initial voxel grid size in meters (default: 4.0)
  --downsample <float>  Point cloud downsample leaf size (default: 0.1)
  --eigen-ratio <float> Surface plane threshold ratio (default: 0.1)
  --reject-ratio <float> Residual outlier rejection ratio (default: 0.05)
  --max-iter <int>      Max Levenberg-Marquardt damping iterations (default: 10)
  --save-maps           Generate and save map_before.pcd and map_after.pcd
  --calc-mme            Calculate Mean Map Entropy (MME) before and after
  --zero-origin         Reference trajectory relative to first pose

Examples:
  # Launch interactive web studio on port 8081:
  ./run_hba.sh --web 8081

  # Run headless optimization on GLIM results:
  ./run_hba.sh --glim ~/data/glim_results/aimbag -o ~/data/hba_results/aimbag_refined --save-maps

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
REBUILD=0
HBA_ARGS=()

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
    --rebuild)
      REBUILD=1
      shift
      ;;
    -h|--help)
      show_help
      ;;
    --glim)
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

# 1. GPU & TTY Detection
DOCKER_GPU_FLAGS=""
if command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  DOCKER_GPU_FLAGS="--gpus all"
fi

DOCKER_TTY_FLAGS=""
if [ -t 0 ] && [ -t 1 ]; then
  DOCKER_TTY_FLAGS="-it"
fi

# 2. Check / Build Image
IMAGE_EXISTS=$(docker images -q "$IMAGE_NAME" 2>/dev/null || true)
if [[ -z "$IMAGE_EXISTS" || "$REBUILD" -eq 1 ]]; then
  echo "Building Docker image: $IMAGE_NAME..."
  docker build -t "$IMAGE_NAME" "$SCRIPT_DIR"
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
  docker rm -f hba_web_studio >/dev/null 2>&1 || true

  echo "=========================================================="
  echo "   Launching HBA Web Studio & Dual-Layer Visualizer       "
  echo "=========================================================="
  echo "  Web Interface URL : http://localhost:${WEB_PORT}"
  echo "  Mapped Data Root  : ${DATA_DIR} -> /data"
  echo "  GPU Acceleration  : ${DOCKER_GPU_FLAGS:-Disabled (CPU)}"
  echo "=========================================================="

  docker run --rm \
    --name hba_web_studio \
    $DOCKER_GPU_FLAGS \
    --user "$(id -u):$(id -g)" \
    --ipc=host \
    -p "${WEB_PORT}:${WEB_PORT}" \
    -v "${DATA_DIR}:/data:rw" \
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
echo "Executing HBA Standalone Runner inside container..."
docker run --rm $DOCKER_TTY_FLAGS \
  $DOCKER_GPU_FLAGS \
  --user "$(id -u):$(id -g)" \
  --ipc=host \
  -v "${DATA_DIR}:/data:rw" \
  "$IMAGE_NAME" "${HBA_ARGS[@]}"
