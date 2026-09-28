#ifndef MYPCL_HPP
#define MYPCL_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <set>
#include <algorithm>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/StdVector>

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > vector_vec3d;
typedef std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> > vector_quad;
// typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZ PointType;
// typedef pcl::PointXYZI PointType;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

namespace mypcl
{
  struct pose
  {
    pose(Eigen::Quaterniond _q = Eigen::Quaterniond(1, 0, 0, 0),
         Eigen::Vector3d _t = Eigen::Vector3d(0, 0, 0)):q(_q), t(_t){}
    Eigen::Quaterniond q;
    Eigen::Vector3d t;
  };

  void loadPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num,
               std::string prefix = "")
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::loadPCDFile(filePath + prefix + ss.str() + ".pcd", *pc);
  }

  void savdPCD(std::string filePath, int pcd_fill_num, pcl::PointCloud<PointType>::Ptr& pc, int num)
  {
    std::stringstream ss;
    if(pcd_fill_num > 0)
      ss << std::setw(pcd_fill_num) << std::setfill('0') << num;
    else
      ss << num;
    pcl::io::savePCDFileBinary(filePath + ss.str() + ".pcd", *pc);
  }
  
  std::vector<pose> read_pose(std::string filename,
                              Eigen::Quaterniond qe = Eigen::Quaterniond(1, 0, 0, 0),
                              Eigen::Vector3d te = Eigen::Vector3d(0, 0, 0))
  {
    std::vector<pose> pose_vec;
    std::fstream file;
    file.open(filename);
    double tx, ty, tz, w, x, y, z;
    while(!file.eof())
    {
      file >> tx >> ty >> tz >> w >> x >> y >> z;
      Eigen::Quaterniond q(w, x, y, z);
      Eigen::Vector3d t(tx, ty, tz);
      pose_vec.push_back(pose(qe * q, qe * t + te));
    }
    file.close();
    return pose_vec;
  }

  void transform_pointcloud(pcl::PointCloud<PointType> const& pc_in,
                            pcl::PointCloud<PointType>& pt_out,
                            Eigen::Vector3d t,
                            Eigen::Quaterniond q)
  {
    size_t size = pc_in.points.size();
    pt_out.points.resize(size);
    for(size_t i = 0; i < size; i++)
    {
      Eigen::Vector3d pt_cur(pc_in.points[i].x, pc_in.points[i].y, pc_in.points[i].z);
      Eigen::Vector3d pt_to;
      // if(pt_cur.norm()<0.3) continue;
      pt_to = q * pt_cur + t;
      pt_out.points[i].x = pt_to.x();
      pt_out.points[i].y = pt_to.y();
      pt_out.points[i].z = pt_to.z();
      // pt_out.points[i].r = pc_in.points[i].r;
      // pt_out.points[i].g = pc_in.points[i].g;
      // pt_out.points[i].b = pc_in.points[i].b;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr append_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc1,
                                                      pcl::PointCloud<pcl::PointXYZRGB> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
      pc1->points[i].r = pc2.points[i-size1].r;
      pc1->points[i].g = pc2.points[i-size1].g;
      pc1->points[i].b = pc2.points[i-size1].b;
      // pc1->points[i].intensity = pc2.points[i-size1].intensity;
    }
    return pc1;
  }

  pcl::PointCloud<PointType>::Ptr append_cloud(pcl::PointCloud<PointType>::Ptr pc1,
                                               pcl::PointCloud<PointType> pc2)
  {
    size_t size1 = pc1->points.size();
    size_t size2 = pc2.points.size();
    pc1->points.resize(size1 + size2);
    for(size_t i = size1; i < size1 + size2; i++)
    {
      pc1->points[i].x = pc2.points[i-size1].x;
      pc1->points[i].y = pc2.points[i-size1].y;
      pc1->points[i].z = pc2.points[i-size1].z;
      // pc1->points[i].r = pc2.points[i-size1].r;
      // pc1->points[i].g = pc2.points[i-size1].g;
      // pc1->points[i].b = pc2.points[i-size1].b;
      // pc1->points[i].intensity = pc2.points[i-size1].intensity;
    }
    return pc1;
  }

  double compute_inlier_ratio(std::vector<double> residuals, double ratio)
  {
    std::set<double> dis_vec;
    for(size_t i = 0; i < (size_t)(residuals.size() / 3); i++)
      dis_vec.insert(fabs(residuals[3 * i + 0]) +
                     fabs(residuals[3 * i + 1]) + fabs(residuals[3 * i + 2]));

    return *(std::next(dis_vec.begin(), (int)((ratio) * dis_vec.size())));
  }

  void write_pose(std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "pose.json", std::ofstream::trunc);
    file.close();
    Eigen::Quaterniond q0(pose_vec[0].q.w(), pose_vec[0].q.x(), pose_vec[0].q.y(), pose_vec[0].q.z());
    Eigen::Vector3d t0(pose_vec[0].t(0), pose_vec[0].t(1), pose_vec[0].t(2));
    file.open(path + "pose.json", std::ofstream::app);

    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      pose_vec[i].t << q0.inverse()*(pose_vec[i].t-t0);
      pose_vec[i].q.w() = (q0.inverse()*pose_vec[i].q).w();
      pose_vec[i].q.x() = (q0.inverse()*pose_vec[i].q).x();
      pose_vec[i].q.y() = (q0.inverse()*pose_vec[i].q).y();
      pose_vec[i].q.z() = (q0.inverse()*pose_vec[i].q).z();
      file << pose_vec[i].t(0) << " "
           << pose_vec[i].t(1) << " "
           << pose_vec[i].t(2) << " "
           << pose_vec[i].q.w() << " " << pose_vec[i].q.x() << " "
           << pose_vec[i].q.y() << " " << pose_vec[i].q.z();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }

  inline void write_pose_file(const std::vector<pose>& pose_vec, const std::string& filepath)
  {
    std::ofstream file(filepath, std::ofstream::trunc);
    if (!file.is_open()) return;
    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      file << pose_vec[i].t(0) << " "
           << pose_vec[i].t(1) << " "
           << pose_vec[i].t(2) << " "
           << pose_vec[i].q.w() << " " << pose_vec[i].q.x() << " "
           << pose_vec[i].q.y() << " " << pose_vec[i].q.z();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }

  void writeEVOPose(std::vector<double>& lidar_times, std::vector<pose>& pose_vec, std::string path)
  {
    std::ofstream file;
    file.open(path + "evo_pose.txt", std::ofstream::trunc);
    for(size_t i = 0; i < pose_vec.size(); i++)
    {
      file << std::setprecision(18) << lidar_times[i] << " " << std::setprecision(6)
           << pose_vec[i].t(0) << " " << pose_vec[i].t(1) << " " << pose_vec[i].t(2) << " "
           << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " "
           << pose_vec[i].q.z() << " " << pose_vec[i].q.w();
      if(i < pose_vec.size()-1) file << "\n";
    }
    file.close();
  }

  inline bool load_glim_submap(const std::string& submap_dir,
                               pcl::PointCloud<PointType>& cloud,
                               pose& submap_pose,
                               double& timestamp)
  {
    std::string data_path = submap_dir + "/data.txt";
    std::string pts_path = submap_dir + "/points_compact.bin";

    std::ifstream df(data_path);
    if (!df.is_open()) return false;

    std::string line;
    bool found_matrix = false;
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    timestamp = 0.0;

    while (std::getline(df, line)) {
      if (line.find("stamp:") != std::string::npos && timestamp == 0.0) {
        std::stringstream ss(line.substr(line.find(":") + 1));
        ss >> timestamp;
      }
      if (line.find("T_world_origin:") != std::string::npos) {
        found_matrix = true;
        for (int r = 0; r < 4; ++r) {
          if (!std::getline(df, line)) break;
          std::stringstream ss(line);
          for (int c = 0; c < 4; ++c) {
            ss >> T(r, c);
          }
        }
      }
    }
    df.close();

    if (!found_matrix) return false;

    Eigen::Matrix3d R = T.block<3, 3>(0, 0);
    submap_pose.q = Eigen::Quaterniond(R).normalized();
    submap_pose.t = T.block<3, 1>(0, 3);

    std::ifstream pf(pts_path, std::ios::binary);
    if (!pf.is_open()) return false;

    pf.seekg(0, std::ios::end);
    size_t file_size = pf.tellg();
    pf.seekg(0, std::ios::beg);

    size_t num_points = file_size / (3 * sizeof(float));
    std::vector<float> buffer(num_points * 3);
    pf.read(reinterpret_cast<char*>(buffer.data()), file_size);
    pf.close();

    cloud.resize(num_points);
    for (size_t i = 0; i < num_points; ++i) {
      cloud.points[i].x = buffer[i * 3 + 0];
      cloud.points[i].y = buffer[i * 3 + 1];
      cloud.points[i].z = buffer[i * 3 + 2];
    }
    return true;
  }

  inline void write_tum_trajectory(const std::vector<pose>& pose_vec,
                                  const std::vector<double>& stamps,
                                  const std::string& filepath)
  {
    std::ofstream f(filepath, std::ofstream::trunc);
    f << std::fixed;
    for (size_t i = 0; i < pose_vec.size(); ++i) {
      double t = (i < stamps.size() && stamps[i] > 0) ? stamps[i] : static_cast<double>(i) * 0.1;
      f << std::setprecision(6) << t << " "
        << std::setprecision(6)
        << pose_vec[i].t.x() << " " << pose_vec[i].t.y() << " " << pose_vec[i].t.z() << " "
        << pose_vec[i].q.x() << " " << pose_vec[i].q.y() << " " << pose_vec[i].q.z() << " "
        << pose_vec[i].q.w() << "\n";
    }
    f.close();
  }

  struct tum_pose {
    double stamp;
    Eigen::Vector3d t;
    Eigen::Quaterniond q;
  };

  inline std::vector<tum_pose> read_tum_trajectory(const std::string& filepath) {
    std::vector<tum_pose> poses;
    std::ifstream f(filepath);
    if (!f.is_open()) return poses;

    std::string line;
    while (std::getline(f, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::stringstream ss(line);
      tum_pose p;
      double qx, qy, qz, qw;
      if (ss >> p.stamp >> p.t.x() >> p.t.y() >> p.t.z() >> qx >> qy >> qz >> qw) {
        p.q = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
        poses.push_back(p);
      }
    }
    f.close();
    return poses;
  }

  inline void refine_dense_trajectory(const std::vector<tum_pose>& dense_poses,
                                     const std::vector<pose>& keyframes_input,
                                     const std::vector<pose>& keyframes_refined,
                                     const std::vector<double>& keyframe_stamps,
                                     const std::string& output_path)
  {
    if (dense_poses.empty() || keyframes_input.empty() || keyframes_input.size() != keyframes_refined.size()) {
      return;
    }

    size_t num_kf = keyframes_input.size();
    // Precompute delta transforms for each keyframe: Delta_T = T_refined * T_input^{-1}
    std::vector<Eigen::Quaterniond> delta_q(num_kf);
    std::vector<Eigen::Vector3d> delta_t(num_kf);

    for (size_t k = 0; k < num_kf; ++k) {
      Eigen::Matrix3d R_in = keyframes_input[k].q.toRotationMatrix();
      Eigen::Vector3d t_in = keyframes_input[k].t;
      Eigen::Matrix3d R_ref = keyframes_refined[k].q.toRotationMatrix();
      Eigen::Vector3d t_ref = keyframes_refined[k].t;

      Eigen::Matrix3d delta_R = R_ref * R_in.transpose();
      delta_q[k] = Eigen::Quaterniond(delta_R).normalized();
      delta_t[k] = t_ref - delta_R * t_in;
    }

    std::ofstream ofs(output_path, std::ofstream::trunc);
    ofs << std::fixed;

    size_t k_cursor = 0;
    for (size_t i = 0; i < dense_poses.size(); ++i) {
      double t = dense_poses[i].stamp;
      Eigen::Quaterniond cur_delta_q;
      Eigen::Vector3d cur_delta_t;

      if (t <= keyframe_stamps.front()) {
        cur_delta_q = delta_q.front();
        cur_delta_t = delta_t.front();
      } else if (t >= keyframe_stamps.back()) {
        cur_delta_q = delta_q.back();
        cur_delta_t = delta_t.back();
      } else {
        // Advance cursor until keyframe_stamps[k_cursor+1] > t
        while (k_cursor + 1 < num_kf && keyframe_stamps[k_cursor + 1] <= t) {
          k_cursor++;
        }
        size_t k0 = k_cursor;
        size_t k1 = std::min(k0 + 1, num_kf - 1);
        double t0 = keyframe_stamps[k0];
        double t1 = keyframe_stamps[k1];
        double dt = t1 - t0;
        double alpha = (dt > 1e-6) ? (t - t0) / dt : 0.0;
        alpha = std::clamp(alpha, 0.0, 1.0);

        cur_delta_q = delta_q[k0].slerp(alpha, delta_q[k1]).normalized();
        cur_delta_t = (1.0 - alpha) * delta_t[k0] + alpha * delta_t[k1];
      }

      // Apply delta transform to raw dense pose: T_ref = Delta_T * T_raw
      Eigen::Quaterniond q_refined = (cur_delta_q * dense_poses[i].q).normalized();
      Eigen::Vector3d t_refined = cur_delta_q * dense_poses[i].t + cur_delta_t;

      ofs << std::setprecision(6) << t << " "
          << t_refined.x() << " " << t_refined.y() << " " << t_refined.z() << " "
          << q_refined.x() << " " << q_refined.y() << " " << q_refined.z() << " "
          << q_refined.w() << "\n";
    }
    ofs.close();
  }
}

#endif