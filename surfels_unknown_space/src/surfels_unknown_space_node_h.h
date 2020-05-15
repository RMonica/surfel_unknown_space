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

#ifndef SURFELS_UNKNOWN_SPACE_NODE_H_H
#define SURFELS_UNKNOWN_SPACE_NODE_H_H

#include "surfels_unknown_space_node.h"
#include "timer_stub.h"
#include <surfels_unknown_space/surfels_unknown_space.h>

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

class SurfelsUnknownSpaceNode
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

  template <class T>
    inline static T SQR(const T & a) {return a * a; }

  SurfelsUnknownSpaceNode(ros::NodeHandle & nh);

  virtual ~SurfelsUnknownSpaceNode();

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
  void ShowSurfelCloud(const SurfelsUnknownSpace::SurfelVector &surfels);
  void ShowStableImage(const uint64 width, const uint64 height,
                       const FloatVector &depths, const FloatVector &colors,
                       const Vector3fVector &bearings, const Eigen::Affine3f &pose);
  bool HasShowKnownStateHull(const SurfelsUnknownSpace::IVisualListener::TKnownStateHullIndex index);
  void ShowKnownStateHull(const uint64 width,
                          const uint64 height,
                          const uint64 special_color_width,
                          const uint64 special_color_height,
                          ros::Publisher & pub,
                          const Uint32Vector & data
                          );
  ros::Publisher & GetKnownStateHullPublisherFromIndex(const SurfelsUnknownSpace::IVisualListener::TKnownStateHullIndex
                                                       index);
  void ShowCamera(const Eigen::Affine3f &pose,
                  const float padding_x,
                  const float padding_y,
                  const float size_z);
  void NewFrame();
  void StartTimer(const SurfelsUnknownSpace::ITimerListener::TPhase phase, const uint64 index);
  void StopTimer(const SurfelsUnknownSpace::ITimerListener::TPhase phase, const uint64 index);

  private:
  ros::NodeHandle & m_nh;

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

  surfels_unknown_space_msgs::FrameStateConstPtr m_frame_state;

  std::shared_ptr<SurfelsUnknownSpace> m_surfels_unknown_space;

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
};

#endif // SURFELS_UNKNOWN_SPACE_NODE_H_H
