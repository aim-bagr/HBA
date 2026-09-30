#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>
#include <filesystem>
#include <cmath>
#include <numeric>
#include <algorithm>

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>

#include "tools.hpp"
#include "mypcl.hpp"
#include "ba.hpp"
#include "hba.hpp"

namespace fs = std::filesystem;

int pcd_name_fill_num = 0;

struct HBAOptions {
  std::string glim_dir = "";
  std::string input_dir = "";
  std::string output_dir = "./results";
  std::string config_name = "default";
  int total_layer_num = 2;   // Optimal default (previously 3)
  int thread_num = 16;
  int pcd_fill_num = 0;
  double voxel_size = 1.5;   // Optimal default (previously 4.0)
  double downsample_size = 0.1;
  double eigen_ratio = 0.05; // Optimal default (previously 0.1)
  double reject_ratio = 0.05;
  int max_iter = 10;
  bool save_maps = false;
  bool calc_mme = true;
  bool zero_origin = false;
};

void apply_preset(HBAOptions& opt, const std::string& name) {
  if (name == "legacy" || name == "upstream" || name == "default_legacy") {
    opt.config_name = "legacy";
    opt.total_layer_num = 3;
    opt.voxel_size = 4.0;
    opt.eigen_ratio = 0.1;
    opt.downsample_size = 0.1;
    opt.reject_ratio = 0.05;
    opt.max_iter = 10;
  } else if (name == "default" || name == "optimal") {
    opt.config_name = "default";
    opt.total_layer_num = 2;
    opt.voxel_size = 1.5;
    opt.eigen_ratio = 0.05;
    opt.downsample_size = 0.1;
    opt.reject_ratio = 0.05;
    opt.max_iter = 10;
  }
}

bool load_config_file(const std::string& filepath, HBAOptions& opt) {
  std::ifstream file(filepath);
  if (!file.is_open()) return false;
  std::string line;
  while (std::getline(file, line)) {
    auto parse_val = [&](const std::string& key, auto& out) {
      size_t pos = line.find("\"" + key + "\"");
      if (pos != std::string::npos) {
        size_t col = line.find(':', pos);
        if (col != std::string::npos) {
          std::string rest = line.substr(col + 1);
          size_t val_start = rest.find_first_not_of(" \t\"");
          size_t val_end = rest.find_last_not_of(" \t,\"\r\n");
          if (val_start != std::string::npos && val_end != std::string::npos && val_end >= val_start) {
            std::string sval = rest.substr(val_start, val_end - val_start + 1);
            if constexpr (std::is_same_v<std::decay_t<decltype(out)>, int>) {
              out = std::stoi(sval);
            } else if constexpr (std::is_same_v<std::decay_t<decltype(out)>, double>) {
              out = std::stod(sval);
            } else if constexpr (std::is_same_v<std::decay_t<decltype(out)>, std::string>) {
              out = sval;
            }
          }
        }
      }
    };
    parse_val("name", opt.config_name);
    parse_val("total_layer_num", opt.total_layer_num);
    parse_val("voxel_size", opt.voxel_size);
    parse_val("eigen_ratio", opt.eigen_ratio);
    parse_val("downsample_size", opt.downsample_size);
    parse_val("reject_ratio", opt.reject_ratio);
    parse_val("max_iter", opt.max_iter);
    size_t mme_pos = line.find("\"calc_mme\"");
    if (mme_pos != std::string::npos) {
      if (line.find("false", mme_pos) != std::string::npos || line.find("0", mme_pos) != std::string::npos) {
        opt.calc_mme = false;
      } else if (line.find("true", mme_pos) != std::string::npos || line.find("1", mme_pos) != std::string::npos) {
        opt.calc_mme = true;
      }
    }
  }
  return true;
}

void print_help() {
  std::cout << "Usage: hba_standalone [options]\n\n"
            << "Input Options:\n"
            << "  --glim <dir>          Path to GLIM output directory (contains 000000, 000001, ...)\n"
            << "  --input <dir>         Path to standard HBA directory (contains pcd/ and pose.json)\n"
            << "  -o, --output <dir>    Output results directory (default: ./results)\n\n"
            << "Configuration & Presets:\n"
            << "  --config <name|path>  Preset name ('default' or 'legacy') or path to JSON file\n"
            << "  --preset <name>       Preset name ('default' [optimal] or 'legacy' [upstream paper])\n\n"
            << "Optimization Options:\n"
            << "  --layers <int>        Total hierarchical layers (default: 2 [legacy: 3])\n"
            << "  --threads <int>       Number of worker threads (default: 16)\n"
            << "  --voxel-size <float>  Initial voxel grid size in meters (default: 1.5 [legacy: 4.0])\n"
            << "  --downsample <float>  Point cloud downsample leaf size (default: 0.1)\n"
            << "  --eigen-ratio <float> Surface plane threshold ratio (default: 0.05 [legacy: 0.1])\n"
            << "  --reject-ratio <float> Residual outlier rejection ratio (default: 0.05)\n"
            << "  --max-iter <int>      Max iterations per local BA window (default: 10)\n"
            << "  --pcd-fill <int>      Leading zero digits in PCD filenames (default: 0)\n"
            << "  --zero-origin         Reference poses relative to first pose (default: false)\n\n"
            << "Output Artifacts & Metrics:\n"
            << "  --save-maps           Generate and save map_before.pcd and map_after.pcd\n"
            << "  --calc-mme            Calculate Mean Map Entropy (MME) metric before & after (default: true)\n"
            << "  --no-mme              Skip Mean Map Entropy (MME) metric calculation\n"
            << "  -h, --help            Show this help message\n";
}

bool parse_args(int argc, char** argv, HBAOptions& opt) {
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      print_help();
      exit(0);
    } else if ((arg == "--config" || arg == "--preset") && i + 1 < argc) {
      std::string val = argv[++i];
      if (val == "legacy" || val == "upstream" || val == "default_legacy" || val == "default" || val == "optimal") {
        apply_preset(opt, val);
      } else if (fs::exists(val)) {
        if (!load_config_file(val, opt)) {
          std::cerr << "Warning: Failed to parse config file: " << val << "\n";
        }
      } else if (fs::exists("/opt/hba/config/config_" + val + ".json")) {
        load_config_file("/opt/hba/config/config_" + val + ".json", opt);
      } else if (fs::exists("config/config_" + val + ".json")) {
        load_config_file("config/config_" + val + ".json", opt);
      } else {
        std::cerr << "Warning: Unknown preset or missing config file: " << val << "\n";
      }
    } else if (arg == "--glim" && i + 1 < argc) {
      opt.glim_dir = argv[++i];
    } else if (arg == "--input" && i + 1 < argc) {
      opt.input_dir = argv[++i];
    } else if ((arg == "-o" || arg == "--output") && i + 1 < argc) {
      opt.output_dir = argv[++i];
    } else if (arg == "--layers" && i + 1 < argc) {
      opt.total_layer_num = std::stoi(argv[++i]);
    } else if (arg == "--threads" && i + 1 < argc) {
      opt.thread_num = std::stoi(argv[++i]);
    } else if (arg == "--voxel-size" && i + 1 < argc) {
      opt.voxel_size = std::stod(argv[++i]);
    } else if (arg == "--downsample" && i + 1 < argc) {
      opt.downsample_size = std::stod(argv[++i]);
    } else if (arg == "--eigen-ratio" && i + 1 < argc) {
      opt.eigen_ratio = std::stod(argv[++i]);
    } else if (arg == "--reject-ratio" && i + 1 < argc) {
      opt.reject_ratio = std::stod(argv[++i]);
    } else if (arg == "--max-iter" && i + 1 < argc) {
      opt.max_iter = std::stoi(argv[++i]);
    } else if (arg == "--pcd-fill" && i + 1 < argc) {
      opt.pcd_fill_num = std::stoi(argv[++i]);
    } else if (arg == "--save-maps") {
      opt.save_maps = true;
    } else if (arg == "--calc-mme") {
      opt.calc_mme = true;
    } else if (arg == "--no-mme" || arg == "--skip-mme") {
      opt.calc_mme = false;
    } else if (arg == "--zero-origin") {
      opt.zero_origin = true;
    } else {
      std::cerr << "Unknown argument: " << arg << "\n";
      print_help();
      return false;
    }
  }
  return true;
}

void cut_voxel(std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*>& feat_map,
               pcl::PointCloud<PointType>& feat_pt,
               Eigen::Quaterniond q, Eigen::Vector3d t, int fnum,
               double voxel_size, int window_size, float eigen_ratio)
{
  float loc_xyz[3];
  for(PointType& p_c: feat_pt.points)
  {
    Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
    Eigen::Vector3d pvec_tran = q * pvec_orig + t;

    for(int j = 0; j < 3; j++)
    {
      loc_xyz[j] = pvec_tran[j] / voxel_size;
      if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
      iter->second->vec_orig[fnum].push_back(pvec_orig);
      iter->second->vec_tran[fnum].push_back(pvec_tran);

      iter->second->sig_orig[fnum].push(pvec_orig);
      iter->second->sig_tran[fnum].push(pvec_tran);
    }
    else
    {
      OCTO_TREE_ROOT* ot = new OCTO_TREE_ROOT(window_size, eigen_ratio);
      ot->vec_orig[fnum].push_back(pvec_orig);
      ot->vec_tran[fnum].push_back(pvec_tran);
      ot->sig_orig[fnum].push(pvec_orig);
      ot->sig_tran[fnum].push(pvec_tran);

      ot->voxel_center[0] = (0.5+position.x) * voxel_size;
      ot->voxel_center[1] = (0.5+position.y) * voxel_size;
      ot->voxel_center[2] = (0.5+position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->layer = 0;
      feat_map[position] = ot;
    }
  }
}

void parallel_comp(LAYER& layer, int thread_id, LAYER& next_layer)
{
  int& part_length = layer.part_length;
  for(int i = thread_id * part_length; i < (thread_id + 1) * part_length; i++)
  {
    std::vector<pcl::PointCloud<PointType>::Ptr> src_pc(WIN_SIZE);
    double residual_cur = 0, residual_pre = 0;
    std::vector<IMUST> x_buf(WIN_SIZE);
    for(int j = 0; j < WIN_SIZE; j++)
    {
      x_buf[j].R = layer.pose_vec[i * GAP + j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i * GAP + j].t;
      src_pc[j] = layer.pcds[i * GAP + j]->makeShared();
    }

    size_t mem_cost = 0;
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

      for(size_t j = 0; j < WIN_SIZE; j++)
      {
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size);
        cut_voxel(surf_map, *src_pc[j], Eigen::Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, WIN_SIZE, layer.eigen_ratio);
      }
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();

      VOX_HESS voxhess(WIN_SIZE);
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->tras_opt(voxhess);

      VOX_OPTIMIZER opt_lsv(WIN_SIZE);
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec;
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;

      if((loop > 0 && std::abs(residual_pre - residual_cur) / std::abs(residual_cur) < 0.05) || loop == layer.max_iter - 1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;
        for(int j = 0; j < WIN_SIZE * (WIN_SIZE - 1) / 2; j++)
          layer.hessians[i * (WIN_SIZE - 1) * WIN_SIZE / 2 + j] = hess_vec[j];
        break;
      }
      residual_pre = residual_cur;
    }

    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < WIN_SIZE; j++)
    {
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Eigen::Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
    }
    downsample_voxel(*pc_keyframe, 0.05);
    next_layer.pcds[i] = pc_keyframe;
  }
}

void parallel_tail(LAYER& layer, int thread_id, LAYER& next_layer)
{
  int& part_length = layer.part_length;
  int& left_gap_num = layer.left_gap_num;

  for(int i = thread_id * part_length; i < thread_id * part_length + left_gap_num; i++)
  {
    std::vector<pcl::PointCloud<PointType>::Ptr> src_pc(WIN_SIZE);
    double residual_cur = 0, residual_pre = 0;
    std::vector<IMUST> x_buf(WIN_SIZE);
    for(int j = 0; j < WIN_SIZE; j++)
    {
      x_buf[j].R = layer.pose_vec[i * GAP + j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i * GAP + j].t;
      src_pc[j] = layer.pcds[i * GAP + j]->makeShared();
    }

    size_t mem_cost = 0;
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
      for(size_t j = 0; j < WIN_SIZE; j++)
      {
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size);
        cut_voxel(surf_map, *src_pc[j], Eigen::Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, WIN_SIZE, layer.eigen_ratio);
      }
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();

      VOX_HESS voxhess(WIN_SIZE);
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->tras_opt(voxhess);

      VOX_OPTIMIZER opt_lsv(WIN_SIZE);
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec;
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;

      if((loop > 0 && std::abs(residual_pre - residual_cur) / std::abs(residual_cur) < 0.05) || loop == layer.max_iter - 1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;
        for(int j = 0; j < WIN_SIZE * (WIN_SIZE - 1) / 2; j++)
          layer.hessians[i * (WIN_SIZE - 1) * WIN_SIZE / 2 + j] = hess_vec[j];
        break;
      }
      residual_pre = residual_cur;
    }

    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < WIN_SIZE; j++)
    {
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Eigen::Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
    }
    downsample_voxel(*pc_keyframe, 0.05);
    next_layer.pcds[i] = pc_keyframe;
  }

  if(layer.tail > 0)
  {
    int i = thread_id * part_length + left_gap_num;
    std::vector<pcl::PointCloud<PointType>::Ptr> src_pc(layer.last_win_size);
    double residual_cur = 0, residual_pre = 0;
    std::vector<IMUST> x_buf(layer.last_win_size);
    for(int j = 0; j < layer.last_win_size; j++)
    {
      x_buf[j].R = layer.pose_vec[i * GAP + j].q.toRotationMatrix();
      x_buf[j].p = layer.pose_vec[i * GAP + j].t;
      src_pc[j] = layer.pcds[i * GAP + j]->makeShared();
    }

    size_t mem_cost = 0;
    for(int loop = 0; loop < layer.max_iter; loop++)
    {
      std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;
      for(size_t j = 0; j < (size_t)layer.last_win_size; j++)
      {
        if(layer.downsample_size > 0) downsample_voxel(*src_pc[j], layer.downsample_size);
        cut_voxel(surf_map, *src_pc[j], Eigen::Quaterniond(x_buf[j].R), x_buf[j].p,
                  j, layer.voxel_size, layer.last_win_size, layer.eigen_ratio);
      }
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->recut();

      VOX_HESS voxhess(layer.last_win_size);
      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        iter->second->tras_opt(voxhess);

      VOX_OPTIMIZER opt_lsv(layer.last_win_size);
      opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
      PLV(6) hess_vec;
      opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

      for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
        delete iter->second;

      if((loop > 0 && std::abs(residual_pre - residual_cur) / std::abs(residual_cur) < 0.05) || loop == layer.max_iter - 1)
      {
        if(layer.mem_costs[thread_id] < mem_cost) layer.mem_costs[thread_id] = mem_cost;
        for(int j = 0; j < layer.last_win_size * (layer.last_win_size - 1) / 2; j++)
          layer.hessians[i * (WIN_SIZE - 1) * WIN_SIZE / 2 + j] = hess_vec[j];
        break;
      }
      residual_pre = residual_cur;
    }

    pcl::PointCloud<PointType>::Ptr pc_keyframe(new pcl::PointCloud<PointType>);
    for(size_t j = 0; j < (size_t)layer.last_win_size; j++)
    {
      Eigen::Quaterniond q_tmp;
      Eigen::Vector3d t_tmp;
      assign_qt(q_tmp, t_tmp, Eigen::Quaterniond(x_buf[0].R.inverse() * x_buf[j].R),
                x_buf[0].R.inverse() * (x_buf[j].p - x_buf[0].p));

      pcl::PointCloud<PointType>::Ptr pc_oneframe(new pcl::PointCloud<PointType>);
      mypcl::transform_pointcloud(*src_pc[j], *pc_oneframe, t_tmp, q_tmp);
      pc_keyframe = mypcl::append_cloud(pc_keyframe, *pc_oneframe);
    }
    downsample_voxel(*pc_keyframe, 0.05);
    next_layer.pcds[i] = pc_keyframe;
  }
}

void global_ba(LAYER& layer, double& init_res, double& final_res)
{
  int window_size = layer.pose_vec.size();
  std::vector<IMUST> x_buf(window_size);
  for(int i = 0; i < window_size; i++)
  {
    x_buf[i].R = layer.pose_vec[i].q.toRotationMatrix();
    x_buf[i].p = layer.pose_vec[i].t;
  }

  std::vector<pcl::PointCloud<PointType>::Ptr> src_pc(window_size);
  for(int i = 0; i < window_size; i++)
    src_pc[i] = (*layer.pcds[i]).makeShared();

  double residual_cur = 0, residual_pre = 0;
  size_t mem_cost = 0, max_mem = 0;
  for(int loop = 0; loop < layer.max_iter; loop++)
  {
    std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

    for(int i = 0; i < window_size; i++)
    {
      if(layer.downsample_size > 0) downsample_voxel(*src_pc[i], layer.downsample_size);
      cut_voxel(surf_map, *src_pc[i], Eigen::Quaterniond(x_buf[i].R), x_buf[i].p, i,
                layer.voxel_size, window_size, layer.eigen_ratio * 2);
    }
    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      iter->second->recut();

    VOX_HESS voxhess(window_size);
    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      iter->second->tras_opt(voxhess);

    VOX_OPTIMIZER opt_lsv(window_size);
    opt_lsv.remove_outlier(x_buf, voxhess, layer.reject_ratio);
    PLV(6) hess_vec;
    opt_lsv.damping_iter(x_buf, voxhess, residual_cur, hess_vec, mem_cost);

    for(auto iter = surf_map.begin(); iter != surf_map.end(); ++iter)
      delete iter->second;

    if (loop == 0) init_res = residual_cur;
    final_res = residual_cur;

    double drop_pct = (residual_pre > 1e-6) ? (residual_pre - residual_cur) / residual_pre * 100.0 : 0.0;
    std::cout << "[progress] state=global_ba | iter=" << loop + 1 << "/" << layer.max_iter
              << " | residual=" << std::fixed << std::setprecision(4) << residual_cur
              << " | step_drop=" << std::setprecision(2) << drop_pct << "%" << std::endl;

    if(loop > 0 && std::abs(residual_pre - residual_cur) / std::abs(residual_cur) < 0.05 || loop == layer.max_iter - 1)
    {
      if(max_mem < mem_cost) max_mem = mem_cost;
      #ifdef FULL_HESS
      for(int i = 0; i < window_size * (window_size - 1) / 2; i++)
        layer.hessians[i] = hess_vec[i];
      #endif
      break;
    }
    residual_pre = residual_cur;
  }
  for(int i = 0; i < window_size; i++)
  {
    layer.pose_vec[i].q = Eigen::Quaterniond(x_buf[i].R);
    layer.pose_vec[i].t = x_buf[i].p;
  }
}

void distribute_thread(LAYER& layer, LAYER& next_layer)
{
  int& thread_num = layer.thread_num;
  for(int i = 0; i < thread_num; i++) {
    if(i < thread_num - 1)
      layer.mthreads[i] = new std::thread(parallel_comp, std::ref(layer), i, std::ref(next_layer));
    else
      layer.mthreads[i] = new std::thread(parallel_tail, std::ref(layer), i, std::ref(next_layer));
  }
  for(int i = 0; i < thread_num; i++)
  {
    layer.mthreads[i]->join();
    delete layer.mthreads[i];
  }
}

double compute_mme(pcl::PointCloud<PointType>::Ptr cloud, int threads = 8)
{
  if (!cloud || cloud->points.size() < 50) return 0.0;

  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(cloud);

  size_t total_pts = cloud->points.size();
  std::vector<double> entropies(threads, 0.0);
  std::vector<size_t> counts(threads, 0);

  auto worker = [&](int tid) {
    size_t start = tid * (total_pts / threads);
    size_t end = (tid == threads - 1) ? total_pts : (tid + 1) * (total_pts / threads);
    double local_sum = 0.0;
    size_t valid = 0;

    for (size_t i = start; i < end; ++i) {
      std::vector<int> idxs;
      std::vector<float> dists;
      if (kdtree.radiusSearch(cloud->points[i], 0.3, idxs, dists) > 15) {
        pcl::PointCloud<PointType> local_cloud;
        for (int idx : idxs) local_cloud.points.push_back(cloud->points[idx]);

        Eigen::Vector4f centroid;
        Eigen::Matrix3f cov = Eigen::Matrix3f::Identity();
        pcl::compute3DCentroid(local_cloud, centroid);
        pcl::computeCovarianceMatrixNormalized(local_cloud, centroid, cov);

        double det = static_cast<double>(((2.0 * M_PI * M_E) * cov).determinant());
        if (det > 1e-12) {
          local_sum += 0.5 * std::log(det);
          valid++;
        }
      }
    }
    entropies[tid] = local_sum;
    counts[tid] = valid;
  };

  std::vector<std::thread> workers;
  for (int t = 0; t < threads; ++t) workers.emplace_back(worker, t);
  for (auto& w : workers) w.join();

  double sum_entropy = std::accumulate(entropies.begin(), entropies.end(), 0.0);
  size_t total_valid = std::accumulate(counts.begin(), counts.end(), 0);
  return (total_valid > 0) ? (sum_entropy / total_valid) : 0.0;
}

pcl::PointCloud<PointType>::Ptr build_merged_map(const std::vector<mypcl::pose>& poses,
                                                 const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds,
                                                 double leaf_size = 0.05)
{
  pcl::PointCloud<PointType>::Ptr merged(new pcl::PointCloud<PointType>);
  for (size_t i = 0; i < poses.size() && i < clouds.size(); ++i) {
    if (!clouds[i] || clouds[i]->empty()) continue;
    pcl::PointCloud<PointType> transformed;
    mypcl::transform_pointcloud(*clouds[i], transformed, poses[i].t, poses[i].q);
    merged = mypcl::append_cloud(merged, transformed);
  }
  if (leaf_size > 0 && !merged->empty()) {
    downsample_voxel(*merged, leaf_size);
  }
  return merged;
}

int main(int argc, char** argv)
{
  HBAOptions opt;
  if (!parse_args(argc, argv, opt)) return 1;

  std::cout << "========================================================\n"
            << "        HBA: Standalone Bundle Adjustment Engine        \n"
            << "========================================================\n";

  fs::create_directories(opt.output_dir);

  std::vector<mypcl::pose> init_poses;
  std::vector<pcl::PointCloud<PointType>::Ptr> init_clouds;
  std::vector<double> timestamps;
  std::vector<mypcl::tum_pose> dense_poses;

  auto t_start = std::chrono::steady_clock::now();

  // 1. Ingest Data
  if (!opt.glim_dir.empty()) {
    std::string dense_path = opt.glim_dir + "/traj_lidar.txt";
    if (!fs::exists(dense_path)) {
      std::cerr << "Error: Required dense trajectory not found: " << dense_path << "\n";
      return 1;
    }
    dense_poses = mypcl::read_tum_trajectory(dense_path);
    if (dense_poses.empty()) {
      std::cerr << "Error: Failed to read dense trajectory from: " << dense_path << "\n";
      return 1;
    }
    std::cout << "[loader] Found " << dense_poses.size() << " dense LiDAR poses in " << dense_path << std::endl;
    std::cout << "[loader] Loading GLIM submaps from: " << opt.glim_dir << std::endl;
    std::vector<std::string> submap_dirs;
    for (const auto& entry : fs::directory_iterator(opt.glim_dir)) {
      if (entry.is_directory()) {
        std::string filename = entry.path().filename().string();
        if (std::all_of(filename.begin(), filename.end(), ::isdigit)) {
          submap_dirs.push_back(entry.path().string());
        }
      }
    }
    std::sort(submap_dirs.begin(), submap_dirs.end());

    if (submap_dirs.empty()) {
      std::cerr << "Error: No numbered submap directories found in " << opt.glim_dir << "\n";
      return 1;
    }

    std::cout << "[loader] Found " << submap_dirs.size() << " GLIM submaps. Loading..." << std::endl;
    for (size_t i = 0; i < submap_dirs.size(); ++i) {
      pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
      mypcl::pose p;
      double stamp = 0.0;
      if (mypcl::load_glim_submap(submap_dirs[i], *cloud, p, stamp)) {
        init_clouds.push_back(cloud);
        init_poses.push_back(p);
        timestamps.push_back(stamp);
      }
      if ((i + 1) % 50 == 0 || i + 1 == submap_dirs.size()) {
        std::cout << "[loader] Loaded " << i + 1 << "/" << submap_dirs.size() << " submaps\n";
      }
    }
  } else if (!opt.input_dir.empty()) {
    std::cout << "[loader] Loading HBA dataset from: " << opt.input_dir << std::endl;
    init_poses = mypcl::read_pose(opt.input_dir + "/pose.json");
    if (init_poses.empty()) {
      std::cerr << "Error: Could not read poses from " << opt.input_dir << "/pose.json\n";
      return 1;
    }
    init_clouds.resize(init_poses.size());
    timestamps.resize(init_poses.size());
    for (size_t i = 0; i < init_poses.size(); ++i) {
      pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>);
      mypcl::loadPCD(opt.input_dir + "/", opt.pcd_fill_num, cloud, i, "pcd/");
      init_clouds[i] = cloud;
      timestamps[i] = static_cast<double>(i) * 0.1;
    }
    std::cout << "[loader] Loaded " << init_poses.size() << " PCD scans\n";
  } else {
    std::cerr << "Error: Either --glim <dir> or --input <dir> must be specified.\n";
    print_help();
    return 1;
  }

  size_t num_scans = init_poses.size();
  if (num_scans < 10) {
    std::cerr << "Error: Dataset has only " << num_scans << " scans. HBA requires at least 10 scans.\n";
    return 1;
  }

  // Auto-validate and clamp total_layer_num to avoid over-reduction
  int effective_layers = opt.total_layer_num;
  int current_count = num_scans;
  int max_possible_layers = 1;
  while (current_count > WIN_SIZE) {
    max_possible_layers++;
    int gap_num = (current_count - WIN_SIZE) / GAP;
    current_count = gap_num + 1;
  }
  if (effective_layers > max_possible_layers) {
    std::cout << "[config] Adjusting total_layer_num from " << effective_layers
              << " to " << max_possible_layers << " (dataset size: " << num_scans << ")\n";
    effective_layers = max_possible_layers;
  }

  std::cout << "[config] Preset: " << opt.config_name
            << " | Total Scans: " << num_scans << " | Layers: " << effective_layers
            << " | Voxel Size: " << opt.voxel_size << "m | Eigen Ratio: " << opt.eigen_ratio
            << " | Worker Threads: " << opt.thread_num << "\n";

  // Write initial keyframe poses
  mypcl::write_tum_trajectory(init_poses, timestamps, opt.output_dir + "/poses_keyframes_input.txt");
  mypcl::write_pose_file(init_poses, opt.output_dir + "/poses_keyframes_input.json");

  // 2. Initialize HBA
  HBA hba(effective_layers, init_poses, init_clouds, opt.thread_num,
          opt.voxel_size, opt.eigen_ratio, opt.downsample_size,
          opt.reject_ratio, opt.max_iter);

  for (int i = 0; i < effective_layers; ++i) {
    hba.layers[i].voxel_size = opt.voxel_size;
    hba.layers[i].downsample_size = opt.downsample_size;
    hba.layers[i].eigen_ratio = opt.eigen_ratio;
    hba.layers[i].reject_ratio = opt.reject_ratio;
    hba.layers[i].max_iter = opt.max_iter;
  }

  // 3. Hierarchical Bundle Adjustment
  for (int i = 0; i < effective_layers - 1; ++i) {
    std::cout << "--------------------------------------------------------\n"
              << "[progress] state=local_ba | layer=" << i + 1 << "/" << effective_layers
              << " | windows=" << hba.layers[i].gap_num + 1 << std::endl;

    auto t_layer_start = std::chrono::steady_clock::now();
    distribute_thread(hba.layers[i], hba.layers[i + 1]);
    hba.update_next_layer_state(i);
    auto t_layer_end = std::chrono::steady_clock::now();
    double layer_sec = std::chrono::duration<double>(t_layer_end - t_layer_start).count();
    std::cout << "[layer] Layer " << i + 1 << " complete in " << std::fixed << std::setprecision(2) << layer_sec << "s\n";
  }

  // 4. Global Bundle Adjustment on Top Layer
  std::cout << "--------------------------------------------------------\n"
            << "[progress] state=global_ba | layer=" << effective_layers << "/" << effective_layers
            << " | poses=" << hba.layers[effective_layers - 1].pose_vec.size() << std::endl;

  double init_residual = 0.0, final_residual = 0.0;
  global_ba(hba.layers[effective_layers - 1], init_residual, final_residual);

  // 5. Pose Graph Optimization (PGO)
  std::cout << "--------------------------------------------------------\n"
            << "[progress] state=pgo | solving GTSAM factor graph..." << std::endl;

  std::vector<mypcl::pose> final_poses = hba.pose_graph_optimization(opt.zero_origin);

  // 6. Write Output Trajectories & Poses
  mypcl::write_tum_trajectory(final_poses, timestamps, opt.output_dir + "/poses_keyframes_refined.txt");
  mypcl::write_pose_file(final_poses, opt.output_dir + "/poses_keyframes_refined.json");

  // 7. Write Dense Refined Trajectory
  if (!dense_poses.empty()) {
    std::string dense_output_path = opt.output_dir + "/trajectory_lidar_refined.txt";
    std::cout << "[trajectory] Refining " << dense_poses.size() << " dense LiDAR poses -> " << dense_output_path << std::endl;
    mypcl::refine_dense_trajectory(dense_poses, init_poses, final_poses, timestamps, dense_output_path);
  }

  auto t_end = std::chrono::steady_clock::now();
  double total_sec = std::chrono::duration<double>(t_end - t_start).count();

  double res_drop_pct = (init_residual > 1e-6) ? (init_residual - final_residual) / init_residual * 100.0 : 0.0;

  // 8. Optional MME and Merged Maps
  double mme_before = 0.0, mme_after = 0.0;
  if (opt.calc_mme) {
    std::cout << "[metrics] Computing Mean Map Entropy (MME) before HBA...\n";
    auto map_b = build_merged_map(init_poses, init_clouds, 0.1);
    mme_before = compute_mme(map_b, opt.thread_num);

    std::cout << "[metrics] Computing Mean Map Entropy (MME) after HBA...\n";
    auto map_a = build_merged_map(final_poses, init_clouds, 0.1);
    mme_after = compute_mme(map_a, opt.thread_num);
    std::cout << "[metrics] MME before: " << std::fixed << std::setprecision(4) << mme_before
              << " | MME after: " << mme_after << " (change: " << mme_after - mme_before << ")\n";
  }

  if (opt.save_maps) {
    std::cout << "[maps] Saving map_before.pcd...\n";
    auto map_b = build_merged_map(init_poses, init_clouds, 0.05);
    pcl::io::savePCDFileBinary(opt.output_dir + "/map_before.pcd", *map_b);

    std::cout << "[maps] Saving map_after.pcd...\n";
    auto map_a = build_merged_map(final_poses, init_clouds, 0.05);
    pcl::io::savePCDFileBinary(opt.output_dir + "/map_after.pcd", *map_a);
    std::cout << "[maps] Maps saved successfully to " << opt.output_dir << "\n";
  }

  // 9. Write summary.json
  std::string dataset_path = !opt.glim_dir.empty() ? opt.glim_dir : opt.input_dir;
  std::ofstream sum_file(opt.output_dir + "/summary.json");
  sum_file << "{\n"
           << "  \"dataset_path\": \"" << dataset_path << "\",\n"
           << "  \"config_preset\": \"" << opt.config_name << "\",\n"
           << "  \"num_scans\": " << num_scans << ",\n"
           << "  \"dense_poses_count\": " << dense_poses.size() << ",\n"
           << "  \"total_layers\": " << effective_layers << ",\n"
           << "  \"threads\": " << opt.thread_num << ",\n"
           << "  \"initial_residual\": " << init_residual << ",\n"
           << "  \"final_residual\": " << final_residual << ",\n"
           << "  \"residual_reduction_pct\": " << res_drop_pct << ",\n"
           << "  \"mme_before\": " << mme_before << ",\n"
           << "  \"mme_after\": " << mme_after << ",\n"
           << "  \"mme_change\": " << (mme_after - mme_before) << ",\n"
           << "  \"elapsed_sec\": " << total_sec << ",\n"
           << "  \"output_dir\": \"" << opt.output_dir << "\"\n"
           << "}\n";
  sum_file.close();

  std::cout << "========================================================\n"
            << "[progress] state=completed | residual_reduction=" << std::setprecision(2) << res_drop_pct << "%";
  if (opt.calc_mme) {
    std::cout << " | mme_change=" << std::showpos << std::setprecision(4) << (mme_after - mme_before) << std::noshowpos;
  }
  std::cout << " | elapsed=" << std::setprecision(2) << total_sec << "s\n"
            << "Results saved to: " << opt.output_dir << "\n"
            << "  - poses_keyframes_input.txt\n"
            << "  - poses_keyframes_refined.txt\n";
  if (!dense_poses.empty()) {
    std::cout << "  - trajectory_lidar_refined.txt\n";
  }
  std::cout << "  - poses_keyframes_input.json\n"
            << "  - poses_keyframes_refined.json\n"
            << "  - summary.json\n"
            << "========================================================\n";

  return 0;
}
