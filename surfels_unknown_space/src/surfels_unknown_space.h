/*
 * Copyright (c) 2019, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SURFELS_UNKNOWN_SPACE_H
#define SURFELS_UNKNOWN_SPACE_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>

// STL
#include <stdint.h>
#include <vector>
#include <string>
#include <sstream>
#include <set>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/console/time.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/StdVector>

// custom
#include <surfels_unknown_space_msgs/FrameState.h>
#include <surfels_unknown_space_msgs/SetSurfelCloudAction.h>
#include <surfels_unknown_space_msgs/GetSurfelCloudAction.h>
#include <surfels_unknown_space_msgs/GetTimersAction.h>

// Boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// OpenCL
#include <CL/cl2.hpp>

class SurfelsUnknownSpace
{
  public:
  typedef uint64_t uint64;
  typedef uint32_t uint32;
  typedef int32_t int32;
  typedef int64_t int64;
  typedef uint16_t uint16;
  typedef uint8_t uint8;
  typedef std::vector<bool> BoolVector;
  typedef std::vector<float> FloatVector;
  typedef std::vector<uint64> Uint64Vector;
  typedef std::set<uint64> Uint64Set;
  typedef std::vector<uint8> Uint8Vector;
  typedef std::vector<int64> Int64Vector;
  typedef std::vector<uint32> Uint32Vector;
  typedef boost::shared_ptr<Uint32Vector> Uint32VectorPtr;
  typedef std::vector<int32> Int32Vector;
  typedef std::vector<uint16> Uint16Vector;
  typedef std::vector<Uint64Vector> Uint64VectorVector;
  typedef std::vector<FloatVector> FloatVectorVector;
  typedef std::vector<unsigned char> UCharVector;
  typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;
  typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > Vector4fVector;
  typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
  typedef std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i> > Vector3iVector;
  typedef boost::shared_ptr<boost::thread> ThreadPtr;
  typedef pcl::PointXYZ PointXYZ;
  typedef pcl::PointNormal PointXYZNormal;
  typedef pcl::PointCloud<PointXYZ> PointXYZCloud;
  typedef pcl::PointCloud<PointXYZNormal> PointXYZNormalCloud;
  typedef pcl::PointXYZRGBA PointXYZRGBA;
  typedef pcl::PointCloud<PointXYZRGBA> PointXYZRGBACloud;
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;

  typedef std::shared_ptr<cl::Context> CLContextPtr;
  typedef std::shared_ptr<cl::CommandQueue> CLCommandQueuePtr;
  typedef std::shared_ptr<cl::Buffer> CLBufferPtr;
  typedef std::shared_ptr<cl::Device> CLDevicePtr;
  typedef std::shared_ptr<cl::Program> CLProgramPtr;
  typedef std::shared_ptr<cl::Kernel> CLKernelPtr;
  typedef std::vector<cl_float> CLFloatVector;
  typedef std::vector<cl_float3> CLFloat3Vector;
  typedef std::vector<cl_int> CLInt32Vector;
  typedef std::vector<cl_uint> CLUInt32Vector;
  typedef std::vector<cl_ushort2> CLUShort2Vector;

  template <class T>
    inline static T SQR(const T & a) {return a * a; }

  struct Surfel
  {
    Eigen::Vector3f position;
    float radius;
    Eigen::Vector3f normal;
    bool erased;
    bool is_surfel; // if false, this is a frontel
    uint8 cr,cg,cb;

    std::string ToString() const
    {
      std::ostringstream oss;
      oss << "(" << position.transpose() << ") (ø " << radius << ") (⟂ " << normal.transpose() << ")";
      return oss.str();
    }

    Surfel(const Eigen::Vector3f & p,const float r,const Eigen::Vector3f & n,const bool surfel)
    {
      position = p;
      radius = r;
      normal = n;
      erased = false;
      is_surfel = surfel;
      cr = cg = cb = 0;
    }

    Surfel()
    {
      normal = position = Eigen::Vector3f::Zero();
      radius = 0.0;
      erased = false;
      is_surfel = false;
      cr = cg = cb = 0;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };
  typedef std::vector<Surfel, Eigen::aligned_allocator<Surfel> > SurfelVector;

  struct OpenCLSurfel
  {
    cl_float position[3];
    cl_float radius;
    cl_float normal[3];
    cl_uchar erased;
    cl_uchar is_surfel;

    OpenCLSurfel()
    {
      for (uint64 i = 0; i < 3; i++)
        position[i] = 0;
      for (uint64 i = 0; i < 3; i++)
        normal[i] = 0;
      radius = 0.0;
      erased = false;
      is_surfel = false;
    }

    explicit OpenCLSurfel(const Surfel & other)
    {
      for (uint64 i = 0; i < 3; i++)
        position[i] = other.position[i];
      radius = other.radius;
      for (uint64 i = 0; i < 3; i++)
        normal[i] = other.normal[i];
      erased = other.erased;
      is_surfel = other.is_surfel;
    }

    Surfel toSurfel()
    {
      Surfel result;
      for (uint64 i = 0; i < 3; i++)
        result.position[i] = position[i];
      result.radius = radius;
      for (uint64 i = 0; i < 3; i++)
        result.normal[i] = normal[i];
      result.erased = erased;
      result.is_surfel = is_surfel;
      return result;
    }
  } __attribute__((packed));
  typedef std::vector<OpenCLSurfel> OpenCLSurfelVector;

  SurfelsUnknownSpace(ros::NodeHandle & nh);

  virtual ~SurfelsUnknownSpace();

  void initOpenCL();

  float getVoxelSideAtDistance(const float distance) const;
  float getVoxelSideAtDistance(const float distance, float min_range) const;

  int64 getVoxelCountAtDistance(const float distance) const;
  float getVoxelCountAtDistanceF(const float distance) const;

  float getDistanceFromVoxelCount(const uint64 count);

  Eigen::Vector3f getBearingForPixel(const uint64 x,const uint64 y);
  Vector3fVector getAllBearings(const uint64 width,const uint64 height);

  CLBufferPtr OpenCLCreateBuffer(const CLContextPtr context,
                                 const size_t size,
                                 const std::string name) const;
  void OpenCLCreateSurfels(const FloatVector & colors, const uint64 count_at_max_range);
  void OpenCLGetAllBearings(Vector3fVector & bearings);
  void OpenCLGenerateObservedSpaceField(const uint64 count_at_max_range,
                                       const FloatVector & depths);
  void OpenCLUpdateCurrentPoseAndIntrinsics(const Eigen::Affine3f & pose,
                                            const float min_range,
                                            const float max_range,
                                            const float dot_field_valid_th,
                                            const float dot_field_safety_th,
                                            const uint64 back_ignore_margin);
  void OpenCLProjectAndDeleteSurfels(const Eigen::Affine3f & pose,
                            const SurfelVector & surfels,
                            const uint64 count_at_max_range,
                            const FloatVector & colors);

  void OpenCLBuildKnownSpaceField(const uint64 count_at_max_range);

  void OpenCLFilterKnownSpaceField(const uint64 count_at_max_range);

  void OpenCLFilterKnownStateHullPN(const uint64 count_at_max_range);

  void ComputeApproximateFrustumBoundingBox(const Eigen::Affine3f & pose,
                                            Eigen::Vector3f & bounding_box_min,
                                            Eigen::Vector3f & bounding_box_max);

  void SplitNearSurfels(const Eigen::Affine3f & pose);

  template <typename T>
  std::vector<T> DownSample(const uint64 width,
                            const uint64 height,
                            const uint64 towidth,
                            const uint64 toheight,
                            const uint64 step,
                            const T mult,
                            const std::vector<T> & vec
                            );

  uint64 CreateSurfel(const Surfel & surfel);
  void DeleteSurfel(const uint64 id);
  void ClearFrontels() {m_surfels.clear(); m_deleted_surfels.clear(); }

  void onFrameState(const surfels_unknown_space_msgs::FrameStateConstPtr & state_ptr);

  void ProcessFrameWorker();
  void ProcessFrame(surfels_unknown_space_msgs::FrameStateConstPtr state_ptr);

  void onGetSurfelCloudAction(const surfels_unknown_space_msgs::GetSurfelCloudGoalConstPtr & goal);
  void onSetSurfelCloudAction(const surfels_unknown_space_msgs::SetSurfelCloudGoalConstPtr & goal);
  void onGetTimersAction(const surfels_unknown_space_msgs::GetTimersGoalConstPtr & goal);

  void ShowNUVG(const Eigen::Affine3f &pose,
                  const uint64 count_at_max_range,
                  const uint64 count_at_min_range,
                  const uint64 count_at_zero);
  void ShowSurfelCloud();
  void ShowStableImage(const uint64 width, const uint64 height,
                       const FloatVector &depths, const FloatVector &colors,
                       const Vector3fVector &bearings, const Eigen::Affine3f &pose);
  void ShowKnownStateHull(const uint64 width,
                               const uint64 height,
                               const uint64 special_color_width,
                               const uint64 special_color_height,
                               const bool transpose,
                               ros::Publisher & pub,
                               CLBufferPtr buf
                               );
  void ShowCamera(const Eigen::Affine3f &pose,
                  const float padding_x,
                  const float padding_y,
                  const float size_z);

  private:
  ros::NodeHandle & m_nh;

  SurfelVector m_surfels;
  Uint64Set m_deleted_surfels;

  ros::Subscriber m_frame_state_sub;

  ros::Publisher m_stable_image_pub;
  ros::Publisher m_surfel_cloud_pub;
  ros::Publisher m_known_space_frustum_pub;
  ros::Publisher m_nuvg_display_pub;
  ros::Publisher m_camera_display_pub;
  ros::Publisher m_known_state_hull_xp_pub;
  ros::Publisher m_known_state_hull_xn_pub;
  ros::Publisher m_known_state_hull_yp_pub;
  ros::Publisher m_known_state_hull_yn_pub;
  ros::Publisher m_known_state_hull_zp_pub;
  ros::Publisher m_known_state_hull_zn_pub;
  ros::Publisher m_known_state_hull_xp_f_pub;
  ros::Publisher m_known_state_hull_xn_f_pub;
  ros::Publisher m_known_state_hull_yp_f_pub;
  ros::Publisher m_known_state_hull_yn_f_pub;
  ros::Publisher m_known_state_hull_zp_f_pub;
  ros::Publisher m_known_state_hull_zn_f_pub;

  bool m_frontel_normal_as_color;

  bool m_enable_known_space_filter;
  float m_dot_field_safety_th;
  float m_dot_field_valid_th;

  surfels_unknown_space_msgs::FrameStateConstPtr m_frame_state;

  boost::mutex m_mutex;
  boost::condition_variable m_cond_var;
  uint32 m_requests_pending;
  bool m_processing;
  ThreadPtr m_thread;

  ros::Publisher m_ack_publisher;

  typedef actionlib::SimpleActionServer<surfels_unknown_space_msgs::SetSurfelCloudAction> SetSurfelCloudActionServer;
  typedef std::shared_ptr<SetSurfelCloudActionServer> SetSurfelCloudActionServerPtr;
  typedef actionlib::SimpleActionServer<surfels_unknown_space_msgs::GetSurfelCloudAction> GetSurfelCloudActionServer;
  typedef std::shared_ptr<GetSurfelCloudActionServer> GetSurfelCloudActionServerPtr;
  typedef actionlib::SimpleActionServer<surfels_unknown_space_msgs::GetTimersAction> GetTimersActionServer;
  typedef std::shared_ptr<GetTimersActionServer> GetTimersActionServerPtr;

  SetSurfelCloudActionServerPtr m_set_surfel_cloud_action_server;
  GetSurfelCloudActionServerPtr m_get_surfel_cloud_action_server;
  GetTimersActionServerPtr m_get_timers_action_server;

  float m_surfel_thickness;
  float m_surfel_radius_mult;
  float m_unknown_surfels_radius_mult_pn;
  uint64 m_side_padding;
  uint64 m_back_padding;
  double m_max_range;
  double m_min_range;
  uint64 m_downsample_factor;

  uint64 m_opencl_max_surfels_in_mem;
  uint64 m_surfels_projection_threads;

  // OpenCL
  CLContextPtr m_opencl_context;
  CLCommandQueuePtr m_opencl_command_queue;
  CLDevicePtr m_opencl_device;
  CLProgramPtr m_opencl_program;

  CLKernelPtr m_opencl_projectsurfels_kernel;
  CLKernelPtr m_opencl_project_internal_surfels_kernel;
  CLKernelPtr m_opencl_project_external_surfels_kernel;
  CLKernelPtr m_opencl_get_all_bearings_kernel;
  CLKernelPtr m_opencl_zero_dot_field_kernel;
  CLKernelPtr m_opencl_path_knownxf_kernel;
  CLKernelPtr m_opencl_path_knownxb_kernel;
  CLKernelPtr m_opencl_path_knownyf_kernel;
  CLKernelPtr m_opencl_path_knownyb_kernel;
  CLKernelPtr m_opencl_path_knownzf_kernel;
  CLKernelPtr m_opencl_path_knownzb_kernel;
  CLKernelPtr m_opencl_zero_known_space_field_kernel;
  CLKernelPtr m_opencl_filter_known_space_field_median_kernel;
  CLKernelPtr m_opencl_filter_known_space_field_passthrough_kernel;
  CLKernelPtr m_opencl_simple_fill_uint_kernel;
  CLKernelPtr m_opencl_filter_known_state_hull_pn_kernel;
  CLKernelPtr m_opencl_generate_observed_space_field_kernel;
  CLKernelPtr m_opencl_zero_observed_space_field_kernel;
  CLKernelPtr m_opencl_zero_occupancy_ids_kernel;
  CLKernelPtr m_opencl_create_new_surfels_kernel;

  CLBufferPtr m_opencl_surfels;
  CLBufferPtr m_opencl_pov_matrix;
  CLBufferPtr m_opencl_inverse_pov_matrix;
  CLBufferPtr m_opencl_intrinsics;
  CLBufferPtr m_opencl_bearings;
  CLBufferPtr m_opencl_projected_positions;
  CLBufferPtr m_opencl_projected_normals;
  CLBufferPtr m_opencl_projected_image_coords;
  CLBufferPtr m_opencl_projected_internal_ids;
  CLBufferPtr m_opencl_projected_external_ids;
  CLBufferPtr m_opencl_dot_field;
  CLBufferPtr m_opencl_invalid_dot_field;
  CLBufferPtr m_opencl_known_state_hull_xp;
  CLBufferPtr m_opencl_known_state_hull_xn;
  CLBufferPtr m_opencl_known_state_hull_yp;
  CLBufferPtr m_opencl_known_state_hull_yn;
  CLBufferPtr m_opencl_known_state_hull_zp;
  CLBufferPtr m_opencl_known_state_hull_zn;
  CLBufferPtr m_opencl_known_state_hull_xp_filtered;
  CLBufferPtr m_opencl_known_state_hull_xn_filtered;
  CLBufferPtr m_opencl_known_state_hull_yp_filtered;
  CLBufferPtr m_opencl_known_state_hull_yn_filtered;
  CLBufferPtr m_opencl_known_state_hull_zp_filtered;
  CLBufferPtr m_opencl_known_state_hull_zn_filtered;
  CLBufferPtr m_opencl_known_space_field;
  CLBufferPtr m_opencl_known_space_field_filtered;
  CLBufferPtr m_opencl_filter_known_space_field_counter;
  CLBufferPtr m_opencl_observed_space_field;
  CLBufferPtr m_opencl_depth_image;
  CLBufferPtr m_opencl_ids_to_be_deleted;
  CLBufferPtr m_opencl_ids_to_be_recolored; // these ids frontels->surfels
  CLBufferPtr m_opencl_occupancy_ids; // for each cell, (id + 1) of the surfel in it, 0 if none
  CLBufferPtr m_opencl_creation_counters;
  CLBufferPtr m_opencl_creation_unfinished_grid; // if creation buffer fills up, the grid saves where we stopped
  CLBufferPtr m_opencl_creation_surfel_color_source;
  CLBufferPtr m_opencl_creation_surfel_replace_global_id;

  struct Intrinsics
  {
    float center_x,center_y;
    float focal_x,focal_y,focal_avg;
    uint64 width,height;
    float min_range,max_range;

    Intrinsics(const float cx,const float cy,const float fx,const float fy,const uint64 w,const uint64 h,
               const float mrange,const float Mrange)
    {
      center_x = cx;
      center_y = cy;
      focal_x = fx;
      focal_y = fy;
      focal_avg = (fx + fy) / 2.0;
      width = w;
      height = h;
      min_range = mrange;
      max_range = Mrange;
    }

    Intrinsics RescaleInt(const uint64 scale) const
    {
      return Intrinsics(center_x / scale,center_y / scale,
                        focal_x / scale,focal_y / scale,
                        width / scale,height / scale,
                        min_range, max_range);
    }
  };
  typedef boost::shared_ptr<Intrinsics> IntrinsicsPtr;

  IntrinsicsPtr m_intrinsics;

  struct OpenCLIntrinsics
  {
    cl_float center_x, center_y;
    cl_float focal_x, focal_y, focal_avg;
    cl_int width,height;

    cl_float min_range, max_range, back_range;
    cl_uint count_at_max_range, count_at_min_range, count_at_zero;
    cl_uint back_ignore_margin;

    cl_float dot_field_valid_th, dot_field_safety_th;
    cl_float surfel_thickness;
    cl_float surfel_radius_mult;
    cl_float unknown_surfels_radius_mult_pn;
    cl_uint side_padding;

    OpenCLIntrinsics(const Intrinsics & other, const float o_min_range, const float o_max_range, const float o_back_range,
                     const uint64 o_count_at_max_range, const uint64 o_count_at_min_range, const uint64 o_count_at_zero,
                     const float o_dot_field_valid_th, const float o_dot_field_safety_th,
                     const uint64 o_back_ignore_margin, const float o_surfel_thickness, const float o_surfel_radius_mult,
                     const float o_unknown_surfels_radius_mult_pn, const uint64 o_side_padding)
    {
      center_x = other.center_x;
      center_y = other.center_y;
      focal_x = other.focal_x;
      focal_y = other.focal_y;
      focal_avg = other.focal_avg;
      width = other.width;
      height = other.height;
      min_range = o_min_range;
      max_range = o_max_range;
      back_range = o_back_range;

      count_at_max_range = o_count_at_max_range;
      count_at_min_range = o_count_at_min_range;
      count_at_zero = o_count_at_zero;

      dot_field_safety_th = o_dot_field_safety_th;
      dot_field_valid_th = o_dot_field_valid_th;
      back_ignore_margin = o_back_ignore_margin;
      surfel_thickness = o_surfel_thickness;
      surfel_radius_mult = o_surfel_radius_mult;
      unknown_surfels_radius_mult_pn = o_unknown_surfels_radius_mult_pn;
      side_padding = o_side_padding;
    }
  } __attribute__((packed));

  struct OpenCLRecolor
  {
    cl_uint surfel_id;
    cl_ushort2 coords; // image coordinates where to get the color
  } __attribute__((packed));
  typedef std::vector<OpenCLRecolor> OpenCLRecolorVector;

  struct OpenCLCreationCounters
  {
    cl_uint created;
    cl_uint creation_failed;
    cl_uint max_created;
    cl_uchar reinit;
    cl_uchar unfinished;
  } __attribute__((packed));
};


#endif // SURFELS_UNKNOWN_SPACE_H
