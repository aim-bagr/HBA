# HBA: Hierarchical Bundle Adjustment (Standalone & Web Studio)

[![CI / Build & Deploy Container](https://github.com/aim-bagr/HBA/actions/workflows/docker.yml/badge.svg)](https://github.com/aim-bagr/HBA/actions/workflows/docker.yml)
[![Container Registry](https://img.shields.io/badge/ghcr.io-aim--bagr%2Fhba-blue?logo=docker)](https://github.com/aim-bagr/HBA/pkgs/container/hba)

This repository is an enhanced standalone fork of HKU-MARS [HBA](https://github.com/hku-mars/HBA) (*Large-Scale LiDAR Consistent Mapping Using Hierarchical LiDAR Bundle Adjustment*, IEEE RA-L 2023).

> [!NOTE]
> **Looking for the original research paper, mathematical formulation, or legacy ROS1/Catkin launch files?**  
> Please refer to the upstream repository: **[hku-mars/HBA](https://github.com/hku-mars/HBA)**.

---

## 1. Fork Highlights & Features

This fork transforms HBA into a **pure C++17 standalone optimization engine**, modern **Web Studio visualizer**, and **offline ground-truth generator** conforming to the [`STANDALONE_SLAM_RUNBOOK.md`](file:///home/vishal/Code/STANDALONE_SLAM_RUNBOOK.md):

* **Zero ROS / Catkin Dependencies**: Completely decoupled from ROS, roscpp, and Catkin. The standalone binary (`hba_standalone`) builds directly against Eigen, PCL, and GTSAM.
* **Direct GLIM & PCD Ingestion**:
  * **GLIM Submaps**: Direct binary ingestion of raw GLIM submap folders (`points_compact.bin` + `data.txt`).
  * **Standard PCD Directories**: Supports classic HBA directory structures (`pcd/` + `pose.json`).
* **Dense 20Hz LiDAR Trajectory Refinement**:
  * Ingests GLIM's dense per-scan trajectory (`traj_lidar.txt`).
  * Computes $\mathrm{SE}(3)$ delta interpolation across optimized keyframes.
  * Exports refined dense LiDAR trajectory (`trajectory_lidar_refined.txt`) in standard TUM format for downstream SLAM evaluation.
* **Web Studio & Dual-Layer Visualizer**:
  * Built-in FastAPI + Three.js dashboard with zero host dependencies.
  * Real-time WebSocket streaming of console logs and residual convergence.
  * Dual-layer 3D trajectory visualizer (Input vs. Refined keyframes) and dense 20Hz LiDAR trajectory overlay.
  * Submap scrubber to isolate and inspect individual submaps along the trajectory.
* **Containerized Execution & Runner (`run_hba.sh`)**:
  * Out-of-the-box GPU acceleration (`--gpus all`) and CPU fallback.
  * Automatic pulling from GitHub Container Registry (GHCR) with local build fallbacks.
  * Standardized container naming (`hba-web`, `hba-<dataset>-<timestamp>`) and evaluation metadata labels (`slam-eval.*`).
* **Standardized Output Artifacts**:
  * Replaces ambiguous output names with `poses_keyframes_input.txt` and `poses_keyframes_refined.txt` (TUM format).
  * Computes Mean Map Entropy (MME) before and after optimization (`--calc-mme`).
  * Merges and saves full global point cloud maps (`map_before.pcd`, `map_after.pcd`).

---

## 2. Container Releases (GHCR)

Pre-built Docker images with CUDA 12.2 support are built and published automatically via GitHub Actions:

```text
ghcr.io/aim-bagr/hba:latest      # Latest stable build from main branch
ghcr.io/aim-bagr/hba:<git-sha7>   # Pinned commit build (e.g. 481752f)
```

To pull the latest image directly:
```bash
docker pull ghcr.io/aim-bagr/hba:latest
```

Every container image embeds OCI image specifications, including:
```dockerfile
LABEL org.opencontainers.image.revision=<full-git-sha>
```

---

## 3. Quick Start

### 3.1 Web Studio & 3D Visualizer

Launch the interactive Web Studio:
```bash
./run_hba.sh --web 8081
```

Then open **[http://localhost:8081](http://localhost:8081)** in your browser:
* **Launcher Panel**: Select any GLIM or PCD dataset in `~/data/` and configure optimization parameters.
* **Live Console**: Watch hierarchical layers, LM damping iterations, and GTSAM pose graph convergence stream in real time.
* **3D Comparison**:
  * Toggle **Input Keyframes** (🔴 red) vs. **Refined Keyframes** (🟢 green).
  * Toggle **20Hz LiDAR Dense** trajectory (🔷 cyan).
  * Step through submaps using the submap slider to inspect point cloud alignment.

### 3.2 Headless Optimization (GLIM Output)

Run HBA optimization on GLIM submaps and refine the dense LiDAR trajectory:
```bash
./run_hba.sh --glim ~/data/glim_results/aimbag \
             -o ~/data/hba_results/aimbag_refined \
             --calc-mme \
             --save-maps
```

### 3.3 Headless Optimization (Standard PCD Dataset)

Run optimization on a standard dataset folder containing `pcd/` and `pose.json`:
```bash
./run_hba.sh --input ~/data/kitti07 \
             -o ~/data/hba_results/kitti07_refined \
             --save-maps
```

---

## 4. CLI Options Reference

The host runner [`run_hba.sh`](file:///home/vishal/Code/HBA/run_hba.sh) forwards optimization flags to the `hba_standalone` binary inside the container:

| Option | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `--web [port]` | Flag/Int | `8081` | Launch the Web Studio dashboard and visualizer |
| `--glim <dir>` | Path | — | Path to GLIM results directory (must contain `traj_lidar.txt` and submaps) |
| `--input <dir>` | Path | — | Path to standard HBA directory (containing `pcd/` and `pose.json`) |
| `-o, --output <dir>`| Path | `~/data/hba_results/<name>` | Output directory for refined poses, trajectories, and maps |
| `--layers <int>` | Int | `3` | Number of hierarchical BA layers |
| `--threads <int>` | Int | `16` | Worker thread count for parallel point cloud operations |
| `--voxel-size <m>` | Float | `4.0` | Initial voxel grid size in meters |
| `--downsample <m>` | Float | `0.1` | Voxel leaf size for point cloud downsampling |
| `--eigen-ratio <f>`| Float | `0.1` | Surface plane validity threshold ratio |
| `--reject-ratio <f>`| Float | `0.05` | Outlier residual rejection ratio |
| `--max-iter <int>` | Int | `10` | Maximum Levenberg-Marquardt damping iterations per layer |
| `--calc-mme` | Flag | `false` | Calculate Mean Map Entropy (MME) before and after optimization |
| `--save-maps` | Flag | `false` | Merge and export `map_before.pcd` and `map_after.pcd` |
| `--zero-origin` | Flag | `false` | Translate output trajectories relative to the first pose |
| `--rebuild` | Flag | `false` | Force a local Docker image rebuild before execution |

---

## 5. Output Artifacts

After optimization completes, the output directory contains:

```text
<output_dir>/
├── trajectory_lidar_refined.txt    # 20Hz dense refined trajectory (TUM: stamp x y z qx qy qz qw)
├── poses_keyframes_input.txt       # Initial keyframe poses (TUM format)
├── poses_keyframes_refined.txt     # Refined keyframe poses after HBA (TUM format)
├── poses_keyframes_input.json      # Initial keyframe poses (tx ty tz qw qx qy qz)
├── poses_keyframes_refined.json    # Refined keyframe poses (tx ty tz qw qx qy qz)
├── summary.json                    # Residual reduction %, initial/final residuals, MME, runtime
├── map_before.pcd                  # Global merged map prior to HBA (with --save-maps)
└── map_after.pcd                   # Global merged map after HBA (with --save-maps)
```

---

## 6. Upstream Attribution & Citation

If using the HBA optimization algorithm in academic research, please cite the original HKU-MARS paper:

```bibtex
@ARTICLE{10024300,
  author={Liu, Xiyuan and Liu, Zheng and Kong, Fanze and Zhang, Fu},
  journal={IEEE Robotics and Automation Letters}, 
  title={Large-Scale LiDAR Consistent Mapping Using Hierarchical LiDAR Bundle Adjustment}, 
  year={2023},
  volume={8},
  number={3},
  pages={1523-1530},
  doi={10.1109/LRA.2023.3238902}
}
```

For the original project details, see **[hku-mars/HBA](https://github.com/hku-mars/HBA)**.

---

## 7. License

The code is licensed under the [GNU General Public License v2.0](LICENSE) in accordance with the upstream repository.
