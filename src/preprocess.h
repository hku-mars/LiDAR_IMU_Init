#ifndef PREPROCESS_H
#define PREPROCESS_H

#include "common_lib.h"
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

const bool time_list_cut_frame(PointType &x, PointType &y);

struct orgtype
{
  double range;
  double dista; 
  double angle[2];
  double intersect;
  E_jump edj[2];
  Feature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (float, time, time)
        (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)
// namespace pandar_ros
namespace pandar_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        double timestamp;
        uint16_t  ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (double, timestamp, timestamp)
                                          (std::uint16_t, ring, ring)
)

//ANCHOR robosense modify
namespace robosense_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        std::uint8_t intensity;
        std::uint16_t ring;
        double timestamp;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

// namespace robosense_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint8_t, intensity, intensity)
                                          (std::uint16_t, ring, ring)
                                          (double, timestamp, timestamp)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process_cut_frame_livox(const livox_ros_driver::CustomMsg::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process_cut_frame_pcl2(const sensor_msgs::PointCloud2::ConstPtr &msg, deque<PointCloudXYZI::Ptr> &pcl_out, deque<double> &time_lidar, const int required_frame_num, int scan_count);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS;;
  double blind;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler_kitti(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
#endif