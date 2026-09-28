import asyncio
import os
import re
import signal
import time
import shutil
import json
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Set
from server.models import HBARunRequest, HBAJobProgress, GLIMRunSummary, HBARunSummary

PROGRESS_REGEX = re.compile(
    r"\[progress\] state=(?P<state>\w+)(?: \| layer=(?P<layer>[^\|]+))?(?: \| iter=(?P<iter>[^\|]+))?(?: \| residual=(?P<residual>[\d\.]+))?(?: \| step_drop=(?P<step_drop>[\d\.\-]+)%)?"
)

def format_bytes(size: int) -> str:
    for unit in ['B', 'KB', 'MB', 'GB', 'TB']:
        if size < 1024.0:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} PB"

class ProcessManager:
    def __init__(self, data_root: str = "/data", binary_path: str = "/usr/local/bin/hba_standalone"):
        # Auto-detect data_root if running on host
        if not os.path.exists(data_root) and os.path.exists("/home/vishal/data"):
            data_root = "/home/vishal/data"
        if not os.path.exists(binary_path) and os.path.exists("/home/vishal/Code/HBA/build/hba_standalone"):
            binary_path = "/home/vishal/Code/HBA/build/hba_standalone"

        self.data_root = Path(data_root)
        self.glim_results_dir = self.data_root / "glim_results"
        self.hba_results_dir = self.data_root / "hba_results"
        self.binary_path = Path(binary_path)

        try:
            self.glim_results_dir.mkdir(parents=True, exist_ok=True)
            self.hba_results_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            pass

        self.active_process: Optional[asyncio.subprocess.Process] = None
        self.job: HBAJobProgress = HBAJobProgress(state="idle")
        self.log_history: List[str] = []
        self.max_log_lines = 2000
        self.subscribers: Set[asyncio.Queue] = set()
        self.start_wall_time: float = 0.0

    def subscribe(self) -> asyncio.Queue:
        q = asyncio.Queue()
        self.subscribers.add(q)
        return q

    def unsubscribe(self, q: asyncio.Queue):
        self.subscribers.discard(q)

    async def broadcast(self, message: dict):
        dead_queues = []
        for q in self.subscribers:
            try:
                q.put_nowait(message)
            except Exception:
                dead_queues.append(q)
        for q in dead_queues:
            self.subscribers.discard(q)

    def list_glim_runs(self) -> List[GLIMRunSummary]:
        runs = []
        if not self.glim_results_dir.exists():
            return runs

        for entry in sorted(self.glim_results_dir.iterdir(), reverse=True):
            if not entry.is_dir():
                continue
            submaps = [d for d in entry.iterdir() if d.is_dir() and d.name.isdigit()]
            if not submaps:
                continue
            traj_tum = (entry / "trajectory_tum.txt").exists() or (entry / "traj_lidar.txt").exists()
            total_size = sum(f.stat().st_size for f in entry.glob("**/*") if f.is_file())
            st = entry.stat()
            runs.append(GLIMRunSummary(
                name=entry.name,
                path=str(entry),
                submaps_count=len(submaps),
                has_trajectory=traj_tum,
                size_bytes=total_size,
                size_human=format_bytes(total_size),
                modified_at=datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
            ))
        return runs

    def list_hba_runs(self) -> List[HBARunSummary]:
        runs = []
        if not self.hba_results_dir.exists():
            return runs

        for entry in sorted(self.hba_results_dir.iterdir(), reverse=True):
            if not entry.is_dir():
                continue
            traj_b = (entry / "poses_keyframes_input.txt").exists() or (entry / "trajectory_tum_before.txt").exists()
            traj_a = (entry / "poses_keyframes_refined.txt").exists() or (entry / "trajectory_tum_after.txt").exists()
            maps = (entry / "map_before.pcd").exists() and (entry / "map_after.pcd").exists()
            
            submaps_count = 0
            res_reduction = 0.0
            elapsed = 0.0
            summary_file = entry / "summary.json"
            if summary_file.exists():
                try:
                    data = json.loads(summary_file.read_text())
                    submaps_count = data.get("num_scans", 0)
                    res_reduction = data.get("residual_reduction_pct", 0.0)
                    elapsed = data.get("elapsed_sec", 0.0)
                except Exception:
                    pass

            total_size = sum(f.stat().st_size for f in entry.glob("**/*") if f.is_file())
            st = entry.stat()
            runs.append(HBARunSummary(
                name=entry.name,
                path=str(entry),
                submaps_count=submaps_count,
                has_before_traj=traj_b,
                has_after_traj=traj_a,
                has_maps=maps,
                residual_reduction_pct=res_reduction,
                elapsed_sec=elapsed,
                size_bytes=total_size,
                size_human=format_bytes(total_size),
                modified_at=datetime.fromtimestamp(st.st_mtime).strftime("%Y-%m-%d %H:%M:%S")
            ))
        return runs

    async def start_job(self, req: HBARunRequest) -> HBAJobProgress:
        if self.job.state in ["running", "optimizing", "global_ba", "pgo"]:
            raise RuntimeError("An optimization job is already running.")

        dataset_path = Path(req.dataset_path)
        if not dataset_path.exists():
            raise RuntimeError(f"Input dataset path does not exist: {dataset_path}")

        run_name = req.run_name
        if not run_name:
            source_stem = dataset_path.name
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_name = f"{source_stem}_{timestamp}"

        output_dir = self.hba_results_dir / run_name
        output_dir.mkdir(parents=True, exist_ok=True)

        meta = {
            "dataset_path": str(dataset_path),
            "source_type": req.source_type,
            "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        }
        try:
            (output_dir / "input_meta.json").write_text(json.dumps(meta, indent=2))
        except Exception:
            pass

        # Build CLI command line
        cmd = [str(self.binary_path)]
        if req.source_type == "glim" or (dataset_path / "000000").exists():
            cmd.extend(["--glim", str(dataset_path)])
        else:
            cmd.extend(["--input", str(dataset_path)])

        cmd.extend(["-o", str(output_dir)])
        cmd.extend(["--layers", str(req.total_layer_num)])
        cmd.extend(["--threads", str(req.thread_num)])
        cmd.extend(["--voxel-size", str(req.voxel_size)])
        cmd.extend(["--downsample", str(req.downsample_size)])
        cmd.extend(["--eigen-ratio", str(req.eigen_ratio)])
        cmd.extend(["--reject-ratio", str(req.reject_ratio)])
        cmd.extend(["--max-iter", str(req.max_iter)])

        if req.save_maps:
            cmd.append("--save-maps")
        if req.calc_mme:
            cmd.append("--calc-mme")
        if req.zero_origin:
            cmd.append("--zero-origin")

        self.log_history = []
        self.start_wall_time = time.time()
        self.job = HBAJobProgress(
            state="running",
            run_name=run_name,
            dataset_path=str(dataset_path),
            output_dir=str(output_dir),
            started_at=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            layer_info="Initializing",
            progress_pct=0.0
        )

        try:
            self.active_process = await asyncio.create_subprocess_exec(
                *cmd,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT
            )
        except Exception as e:
            self.job.state = "failed"
            self.job.error_message = str(e)
            raise RuntimeError(f"Failed to launch hba_standalone: {e}")

        # Spawn reader task
        asyncio.create_task(self._process_monitor_loop(self.active_process, output_dir))
        await self.broadcast({"type": "status", "data": self.job.dict()})
        return self.job

    async def _process_monitor_loop(self, proc: asyncio.subprocess.Process, output_dir: Path):
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            text = line.decode('utf-8', errors='replace').rstrip()
            if not text:
                continue

            self.log_history.append(text)
            if len(self.log_history) > self.max_log_lines:
                self.log_history.pop(0)

            await self._parse_progress_line(text)
            await self.broadcast({"type": "log", "line": text})

        return_code = await proc.wait()
        self.job.return_code = return_code
        self.job.elapsed_sec = time.time() - self.start_wall_time

        if return_code == 0:
            self.job.state = "completed"
            self.job.progress_pct = 100.0
            summary_file = output_dir / "summary.json"
            if summary_file.exists():
                try:
                    summary = json.loads(summary_file.read_text())
                    self.job.initial_residual = summary.get("initial_residual", 0.0)
                    self.job.current_residual = summary.get("final_residual", 0.0)
                    self.job.residual_reduction_pct = summary.get("residual_reduction_pct", 0.0)
                    self.job.mme_before = summary.get("mme_before")
                    self.job.mme_after = summary.get("mme_after")
                except Exception:
                    pass
        elif self.job.state != "stopped":
            self.job.state = "failed"
            self.job.error_message = f"Process exited with error code {return_code}"

        self.active_process = None
        await self.broadcast({"type": "status", "data": self.job.dict()})

    async def _parse_progress_line(self, line: str):
        if "state=local_ba" in line:
            self.job.state = "running"
            m = re.search(r"layer=([^\|]+)", line)
            if m:
                self.job.layer_info = f"Layer {m.group(1)} Local BA"
        elif "state=global_ba" in line:
            self.job.state = "running"
            m = re.search(r"iter=([^\|]+)", line)
            it = m.group(1) if m else ""
            self.job.layer_info = f"Global BA ({it})"
            m_res = re.search(r"residual=([\d\.]+)", line)
            if m_res:
                self.job.current_residual = float(m_res.group(1))
        elif "state=pgo" in line:
            self.job.state = "running"
            self.job.layer_info = "Pose Graph Optimization"
        elif "residual_reduction=" in line:
            m = re.search(r"residual_reduction=([\d\.]+)%", line)
            if m:
                self.job.residual_reduction_pct = float(m.group(1))

        self.job.elapsed_sec = time.time() - self.start_wall_time
        await self.broadcast({"type": "progress", "data": self.job.dict()})

    async def stop_job(self) -> HBAJobProgress:
        if self.active_process and self.active_process.returncode is None:
            self.job.state = "stopped"
            try:
                self.active_process.send_signal(signal.SIGINT)
                await asyncio.sleep(0.5)
                if self.active_process.returncode is None:
                    self.active_process.terminate()
            except Exception:
                pass
        return self.job
