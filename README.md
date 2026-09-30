# HBA (AIM Standalone Fork)

This repository is an optimized standalone fork of [hku-mars/HBA](https://github.com/hku-mars/HBA) developed by the **Autonomous Infrastructure Mapping (AIM)** team.

> [!NOTE]
> **Looking for the core algorithm, theory, or upstream ROS packages?**  
> Please refer to the upstream repository at **[hku-mars/HBA](https://github.com/hku-mars/HBA)**.

---

## Container Releases (GHCR)

Pre-built CUDA-accelerated Docker containers are published automatically to GitHub Container Registry (**GHCR**):

```bash
# Pull the latest stable container from main
docker pull ghcr.io/aim-bagr/hba:latest

# Or pull an immutable build tagged by git commit SHA (7 chars)
docker pull ghcr.io/aim-bagr/hba:<git-sha7>
```

Every container image carries standard OCI revision metadata (`org.opencontainers.image.revision=<full-sha>`) and evaluation labels (`slam-eval.tool=hba`).

---

## Fork Features & Improvements

This fork transforms HBA into a production-grade, standalone LiDAR map optimization engine and evaluation tool with zero host pollution:

### 1. Standalone Optimization Engine (`hba_standalone`)
- **Zero ROS Dependencies**: Pure C++17 binary linking directly against Eigen, PCL, and GTSAM without requiring ROS or Catkin workspaces.
- **Direct GLIM & PCD Ingestion**: Ingests raw GLIM binary submap folders (`points_compact.bin` + `data.txt`) or standard PCD directories (`pcd/` + `pose.json`).
- **Standardized TUM Output**: Emits sparse keyframe poses (`poses_keyframes_input.txt` & `poses_keyframes_refined.txt`) in standard TUM format (`stamp x y z qx qy qz qw`).
- **MME & Global Map Export**: Computes Mean Map Entropy before and after optimization (`--calc-mme`) and merges global point cloud maps (`--save-maps`).

### 2. Dense LiDAR Trajectory Refinement
- **SE(3) Delta Interpolation**: Ingests high-rate GLIM trajectories (`traj_lidar.txt`) and applies $\mathrm{SE}(3)$ delta interpolation across optimized keyframes.
- **Ground-Truth Candidate Export**: Emits high-frequency refined LiDAR poses to `trajectory_lidar_refined.txt` (TUM format) for downstream SLAM evaluation.

### 3. Headless Web Studio & 3D Visualizer
- **FastAPI + Three.js Dashboard**: Browser-based mission control for launching optimization runs and visualizing results (`./run_hba.sh --web`).
- **Real-Time Console Streaming**: Live WebSocket streaming of hierarchical layer progress, LM damping iterations, and GTSAM graph convergence.
- **Dual-Layer 3D Comparison**: Compare Input (🔴 red) vs. Refined (🟢 green) keyframes, or overlay dense 20Hz LiDAR trajectories (🔷 cyan).
- **Submap Isolation & Scrubber**: Step through and inspect individual submaps along the trajectory.

---

## Quickstart

The easiest way to run this fork is using the [`run_hba.sh`](run_hba.sh) launcher script. It automatically handles GPU detection (`--gpus all`), automated container pulling from GHCR, and volume mounts.

### 1. Run Optimization on GLIM Results

```bash
./run_hba.sh --glim ~/data/glim_results/my_dataset \
             -o ~/data/hba_results/my_dataset_refined \
             --calc-mme \
             --save-maps
```

### 2. Run Optimization on Standard PCD Dataset

```bash
./run_hba.sh --input ~/data/kitti07 \
             -o ~/data/hba_results/kitti07_refined \
             --save-maps
```

### 3. Launch the 3D Web Studio

```bash
./run_hba.sh --web 8081
```
Open **`http://localhost:8081`** in your browser to launch runs, stream logs, and visually inspect map alignment.

> [!TIP]
> Run `./run_hba.sh --help` for the full list of optimization parameters, layer limits, voxel sizes, and filtering thresholds.

---

## Running Directly with Docker / GHCR

You can also run the published container directly without cloning the repository:

```bash
docker run --rm --gpus all \
  --user "$(id -u):$(id -g)" \
  --ipc=host \
  -v /path/to/data:/data:rw \
  ghcr.io/aim-bagr/hba:latest \
  --glim /data/glim_results/my_dataset \
  -o /data/hba_results/my_dataset_refined \
  --calc-mme \
  --save-maps
```

---

## Container & Evaluation Conventions

This repository adheres to the container naming and labeling standard defined in [`docs/STANDALONE_SLAM_RUNBOOK.md`](../STANDALONE_SLAM_RUNBOOK.md):

| Mode | Container Name Format | Standard Labels |
| :--- | :--- | :--- |
| **Batch Optimization** | `hba-<dataset>-<yyyymmdd-hhmm>` | `slam-eval.tool=hba`, `slam-eval.dataset=<name>`, `slam-eval.run-id=<ts>` |
| **Web Service** | `hba-web` | `slam-eval.tool=hba`, `slam-eval.service=web` |

---

## Upstream Attribution & Citation

HBA was developed by Xiyuan Liu, Zheng Liu, Fanze Kong, and Fu Zhang at the MARS Lab, The University of Hong Kong. If you use HBA in academic work, please cite:

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

This package is licensed under the [GNU General Public License v2.0](LICENSE).
