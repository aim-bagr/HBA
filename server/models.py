from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any

class HBARunRequest(BaseModel):
    source_type: str = Field("glim", description="Input type: 'glim' or 'dataset'")
    dataset_path: str = Field(..., description="Path to GLIM run directory or HBA dataset directory")
    run_name: Optional[str] = Field(None, description="Custom name for output folder in /data/hba_results")
    total_layer_num: int = Field(3, description="Number of hierarchical layers (default: 3)")
    thread_num: int = Field(16, description="Number of worker threads (default: 16)")
    voxel_size: float = Field(4.0, description="Initial voxel size in meters (default: 4.0)")
    downsample_size: float = Field(0.1, description="Leaf size for downsampling scans in meters (default: 0.1)")
    eigen_ratio: float = Field(0.1, description="Surface plane threshold ratio (default: 0.1)")
    reject_ratio: float = Field(0.05, description="Outlier residual rejection ratio (default: 0.05)")
    max_iter: int = Field(10, description="Max Levenberg-Marquardt damping iterations per window (default: 10)")
    save_maps: bool = Field(True, description="Save merged before/after PCD point cloud maps (default: True)")
    calc_mme: bool = Field(False, description="Calculate Mean Map Entropy metric before & after (default: False)")
    zero_origin: bool = Field(False, description="Re-zero origin relative to first scan (default: False)")

class HBAJobProgress(BaseModel):
    state: str = "idle"  # idle, running, completed, stopped, failed
    run_name: Optional[str] = None
    dataset_path: Optional[str] = None
    output_dir: Optional[str] = None
    started_at: Optional[str] = None
    elapsed_sec: float = 0.0
    layer_info: str = "idle"
    progress_pct: float = 0.0
    initial_residual: float = 0.0
    current_residual: float = 0.0
    residual_reduction_pct: float = 0.0
    mme_before: Optional[float] = None
    mme_after: Optional[float] = None
    return_code: Optional[int] = None
    error_message: Optional[str] = None

class GLIMRunSummary(BaseModel):
    name: str
    path: str
    submaps_count: int
    has_trajectory: bool
    size_bytes: int
    size_human: str
    modified_at: str

class HBARunSummary(BaseModel):
    name: str
    path: str
    submaps_count: int
    has_before_traj: bool
    has_after_traj: bool
    has_maps: bool
    residual_reduction_pct: float
    elapsed_sec: float
    size_bytes: int
    size_human: str
    modified_at: str
