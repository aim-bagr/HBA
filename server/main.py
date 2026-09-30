import os
import re
import json
import struct
import asyncio
from typing import Optional, List, Dict, Set
from pathlib import Path
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware

from server.models import HBARunRequest, HBAJobProgress
from server.manager import ProcessManager

app = FastAPI(title="HBA Consistent Mapping Studio", version="1.0.0")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

manager = ProcessManager()

# Static web directory
web_dir = Path("/opt/hba/web")
if not web_dir.exists() and Path("./web").exists():
    web_dir = Path("./web").resolve()

@app.get("/api/status")
async def get_status():
    return manager.job.dict()

@app.get("/api/mode")
async def get_mode():
    return {"view_mode": manager.view_mode, "view_path": str(manager.view_path) if manager.view_path else None}

@app.get("/api/glim-runs")
async def get_glim_runs():
    return [r.dict() for r in manager.list_glim_runs()]

@app.get("/api/runs")
async def get_hba_runs():
    return [r.dict() for r in manager.list_hba_runs()]

@app.post("/api/jobs/start")
async def start_job(req: HBARunRequest):
    try:
        job = await manager.start_job(req)
        return job.dict()
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except RuntimeError as e:
        raise HTTPException(status_code=409, detail=str(e))
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/jobs/stop")
async def stop_job():
    try:
        job = await manager.stop_job()
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    return job.dict()

@app.get("/api/jobs/logs")
async def get_logs():
    return {"lines": manager.log_history}

@app.get("/api/runs/{run_name}/trajectory")
async def get_run_trajectory(run_name: str, opt: bool = True, dense: bool = False):
    run_dir = manager.hba_run_dir(run_name)
    if not run_dir.exists():
        raise HTTPException(status_code=404, detail="Run directory not found")

    if dense:
        candidates = ["trajectory_lidar_refined.txt"]
    elif opt:
        candidates = ["poses_keyframes_refined.txt", "trajectory_tum_after.txt"]
    else:
        candidates = ["poses_keyframes_input.txt", "trajectory_tum_before.txt"]

    file_path = None
    for cand_name in candidates:
        p = run_dir / cand_name
        if p.exists():
            file_path = p
            break

    if not file_path:
        raise HTTPException(status_code=404, detail=f"Trajectory file {candidates[0]} not found")

    return FileResponse(path=str(file_path), filename=f"{run_name}_{file_path.name}", media_type="text/plain")

@app.get("/api/runs/{run_name}/summary")
async def get_run_summary(run_name: str):
    file_path = manager.hba_run_dir(run_name) / "summary.json"
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Summary not found")
    try:
        data = json.loads(file_path.read_text())
        return data
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/runs/{run_name}/files/{filename}")
async def get_run_file(run_name: str, filename: str):
    file_path = manager.hba_run_dir(run_name) / filename
    if not file_path.exists() or not file_path.is_file():
        raise HTTPException(status_code=404, detail="File not found")
    return FileResponse(path=str(file_path), filename=filename)

# Cache for submap metadata
_submap_metadata_cache = {}

def find_source_dir(run_name: str) -> Optional[Path]:
    if manager.view_mode:
        return manager.view_glim_dir(run_name)
    run_dir = manager.hba_run_dir(run_name)
    meta_file = run_dir / "input_meta.json"
    if meta_file.exists():
        try:
            meta = json.loads(meta_file.read_text())
            if "dataset_path" in meta and Path(meta["dataset_path"]).exists():
                return Path(meta["dataset_path"])
        except Exception:
            pass

    summary_file = run_dir / "summary.json"
    if summary_file.exists():
        try:
            sum_data = json.loads(summary_file.read_text())
            if "dataset_path" in sum_data and Path(sum_data["dataset_path"]).exists():
                return Path(sum_data["dataset_path"])
        except Exception:
            pass

    # Match by exact name or prefix
    prefix = run_name.split("_202")[0]
    if (manager.glim_results_dir / run_name).exists():
        return manager.glim_results_dir / run_name
    if (manager.glim_results_dir / prefix).exists():
        return manager.glim_results_dir / prefix

    # Substring match (e.g. "aimbag" in "test_aimbag" or "docker_aimbag_test")
    if manager.glim_results_dir.exists():
        for glim_run in manager.glim_results_dir.iterdir():
            if glim_run.is_dir() and (glim_run.name in run_name or run_name in glim_run.name):
                return glim_run

    # If submaps count matches any GLIM run
    if summary_file.exists():
        try:
            sum_data = json.loads(summary_file.read_text())
            num_scans = sum_data.get("num_scans")
            if num_scans and manager.glim_results_dir.exists():
                for glim_run in manager.glim_results_dir.iterdir():
                    if glim_run.is_dir():
                        submaps = [d for d in glim_run.iterdir() if d.is_dir() and d.name.isdigit()]
                        if len(submaps) == num_scans:
                            return glim_run
        except Exception:
            pass

    return None

@app.get("/api/runs/{run_name}/submaps")
async def get_run_submaps(run_name: str):
    if run_name in _submap_metadata_cache:
        return _submap_metadata_cache[run_name]

    run_dir = manager.hba_run_dir(run_name)
    if not run_dir.exists():
        raise HTTPException(status_code=404, detail="Run not found")

    # Read keyframe poses: input and refined (with fallback to legacy names)
    poses_before_file = run_dir / "poses_keyframes_input.json"
    if not poses_before_file.exists():
        poses_before_file = run_dir / "pose_before.json"

    poses_after_file = run_dir / "poses_keyframes_refined.json"
    if not poses_after_file.exists():
        poses_after_file = run_dir / "pose_after.json"
    source_dir = find_source_dir(run_name)

    poses_b = []
    if poses_before_file.exists():
        for line in poses_before_file.read_text().strip().split("\n"):
            if not line.strip(): continue
            parts = [float(v) for v in line.split()]
            poses_b.append(parts)

    poses_a = []
    if poses_after_file.exists():
        for line in poses_after_file.read_text().strip().split("\n"):
            if not line.strip(): continue
            parts = [float(v) for v in line.split()]
            poses_a.append(parts)

    submaps = []
    num_scans = max(len(poses_b), len(poses_a))

    for i in range(num_scans):
        pos_b = poses_b[i][:3] if i < len(poses_b) else [0, 0, 0]
        pos_a = poses_a[i][:3] if i < len(poses_a) else pos_b

        num_pts = 15000
        if source_dir:
            pts_file = source_dir / f"{i:06d}" / "points_compact.bin"
            if pts_file.exists():
                num_pts = pts_file.stat().st_size // 12

        submaps.append({
            "id": i,
            "num_points": num_pts,
            "pos_before": pos_b,
            "pos_after": pos_a,
            "pos_input": pos_b,
            "pos_refined": pos_a,
            "pose_before_raw": poses_b[i] if i < len(poses_b) else None,
            "pose_after_raw": poses_a[i] if i < len(poses_a) else None,
            "pose_input_raw": poses_b[i] if i < len(poses_b) else None,
            "pose_refined_raw": poses_a[i] if i < len(poses_a) else None,
            "source_dir": str(source_dir) if source_dir else None
        })

    _submap_metadata_cache[run_name] = submaps
    return submaps

@app.get("/api/runs/{run_name}/submaps/{submap_id}/points")
async def get_submap_points(run_name: str, submap_id: int):
    source_dir = find_source_dir(run_name)
    if not source_dir:
        raise HTTPException(status_code=404, detail="Source dataset directory not found for this run")

    pts_file = source_dir / f"{submap_id:06d}" / "points_compact.bin"
    if not pts_file.exists():
        raise HTTPException(status_code=404, detail=f"Submap points not found: {pts_file}")

    return FileResponse(path=str(pts_file), media_type="application/octet-stream")

@app.websocket("/ws/logs")
async def websocket_logs(ws: WebSocket):
    await ws.accept()
    queue = manager.subscribe()
    try:
        # Send current status and log history on connect
        await ws.send_text(json.dumps({"type": "init", "job": manager.job.dict(), "logs": manager.log_history}))
        while True:
            msg = await queue.get()
            await ws.send_text(json.dumps(msg))
    except (WebSocketDisconnect, asyncio.CancelledError):
        pass
    finally:
        manager.unsubscribe(queue)

# Serve web static assets
if web_dir.exists():
    app.mount("/", StaticFiles(directory=str(web_dir), html=True), name="web")
