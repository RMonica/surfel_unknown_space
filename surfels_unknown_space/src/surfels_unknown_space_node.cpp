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

#include "surfels_unknown_space_node.h"
#include "surfels_unknown_space_node_h.h"
#include "timer_stub.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

class TimerListener : public SurfelsUnknownSpace::ITimerListener
{
  public:
  typedef uint64_t uint64;

  /*
  enum class TPhase
  {
    SUBSAMPLE,
    INIT,
    BEARINGS,
    SPLITTING,
    OBSERVED_FIELD,
    DOT_FIELD,
      DOT_FIELD_INIT,
      DOT_FIELD_UPLOAD,
      DOT_FIELD_PROJECT,
      DOT_FIELD_INTERNAL,
      DOT_FIELD_HULL,
      DOT_FIELD_DELETE,
      DOT_FIELD_RECOLOR,
    DOT_HULL,
    KNOWN_SPACE,
    KNOWN_SPACE_FILTERING,
    CREATION,
    TOTAL,
  };
  */

  void NewFrame() override {TIMER_NEWFRAME; }
  void StartTimer(const TPhase phase, const uint64 index = 0) override
  {
    switch (phase)
    {
      case TPhase::SUBSAMPLE: TIMER_START(TIMERDEF_SUBSAMPLE); break;
      case TPhase::INIT: TIMER_START(TIMERDEF_INIT); break;
      case TPhase::BEARINGS: TIMER_START(TIMERDEF_BEARINGS); break;
      case TPhase::OBSERVED_FIELD: TIMER_START(TIMERDEF_OBSERVED_FIELD); break;
      case TPhase::DOT_FIELD: TIMER_START(TIMERDEF_DOT_FIELD); break;
      case TPhase::DOT_FIELD_INIT: TIMER_START(TIMERDEF_DOT_FIELD_INIT); break;
      case TPhase::DOT_FIELD_UPLOAD: TIMER_START(TIMERDEF_DOT_FIELD_UPLOAD(index)); break;
      case TPhase::DOT_FIELD_PROJECT: TIMER_START(TIMERDEF_DOT_FIELD_PROJECT(index)); break;
      case TPhase::DOT_FIELD_INTERNAL: TIMER_START(TIMERDEF_DOT_FIELD_INTERNAL(index)); break;
      case TPhase::DOT_FIELD_HULL: TIMER_START(TIMERDEF_DOT_FIELD_HULL(index)); break;
      case TPhase::DOT_FIELD_DELETE: TIMER_START(TIMERDEF_DOT_FIELD_DELETE(index)); break;
      case TPhase::DOT_FIELD_RECOLOR: TIMER_START(TIMERDEF_DOT_FIELD_RECOLOR(index)); break;
      case TPhase::DOT_HULL: TIMER_START(TIMERDEF_DOT_HULL); break;
      case TPhase::KNOWN_SPACE: TIMER_START(TIMERDEF_KNOWN_SPACE); break;
      case TPhase::KNOWN_SPACE_FILTERING: TIMER_START(TIMERDEF_KNOWN_SPACE_FILTERING); break;
      case TPhase::CREATION: TIMER_START(TIMERDEF_CREATION); break;
      //case TPhase::TOTAL: TIMER_START(TIMERDEF_TOTAL); break;
    }
  }
  void StopTimer(const TPhase phase, const uint64 index = 0) override
  {
    switch (phase)
    {
      case TPhase::SUBSAMPLE: TIMER_STOP(TIMERDEF_SUBSAMPLE); break;
      case TPhase::INIT: TIMER_STOP(TIMERDEF_INIT); break;
      case TPhase::BEARINGS: TIMER_STOP(TIMERDEF_BEARINGS); break;
      case TPhase::OBSERVED_FIELD: TIMER_STOP(TIMERDEF_OBSERVED_FIELD); break;
      case TPhase::DOT_FIELD: TIMER_STOP(TIMERDEF_DOT_FIELD); break;
      case TPhase::DOT_FIELD_INIT: TIMER_STOP(TIMERDEF_DOT_FIELD_INIT); break;
      case TPhase::DOT_FIELD_UPLOAD: TIMER_STOP(TIMERDEF_DOT_FIELD_UPLOAD(index)); break;
      case TPhase::DOT_FIELD_PROJECT: TIMER_STOP(TIMERDEF_DOT_FIELD_PROJECT(index)); break;
      case TPhase::DOT_FIELD_INTERNAL: TIMER_STOP(TIMERDEF_DOT_FIELD_INTERNAL(index)); break;
      case TPhase::DOT_FIELD_HULL: TIMER_STOP(TIMERDEF_DOT_FIELD_HULL(index)); break;
      case TPhase::DOT_FIELD_DELETE: TIMER_STOP(TIMERDEF_DOT_FIELD_DELETE(index)); break;
      case TPhase::DOT_FIELD_RECOLOR: TIMER_STOP(TIMERDEF_DOT_FIELD_RECOLOR(index)); break;
      case TPhase::DOT_HULL: TIMER_STOP(TIMERDEF_DOT_HULL); break;
      case TPhase::KNOWN_SPACE: TIMER_STOP(TIMERDEF_KNOWN_SPACE); break;
      case TPhase::KNOWN_SPACE_FILTERING: TIMER_STOP(TIMERDEF_KNOWN_SPACE_FILTERING); break;
      case TPhase::CREATION: TIMER_STOP(TIMERDEF_CREATION); break;
      //case TPhase::TOTAL: TIMER_STOP(TIMERDEF_TOTAL); break;
    }
  }
};

class ROSLogger : public SurfelsUnknownSpace::ILogger
{
  public:

  void LogInfo(const std::string & msg) override {ROS_INFO_STREAM(msg); }
  void LogWarn(const std::string & msg) override {ROS_WARN_STREAM(msg); }
  void LogError(const std::string & msg) override {ROS_ERROR_STREAM(msg); }
  void LogFatal(const std::string & msg) override {ROS_FATAL_STREAM(msg); }
};

class VisualListener : public SurfelsUnknownSpace::IVisualListener
{
  public:
  typedef uint64_t uint64;
  typedef std::vector<float> FloatVector;
  typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > Vector3fVector;

  VisualListener(SurfelsUnknownSpaceNode & parent): m_parent(parent) {}

  void ShowNUVG(const Eigen::Affine3f &pose,
                const uint64 count_at_max_range,
                const uint64 count_at_min_range,
                const uint64 count_at_zero) override
  {
    m_parent.ShowNUVG(pose, count_at_max_range, count_at_min_range, count_at_zero);
  }

  void ShowSurfelCloud(const SurfelsUnknownSpace::SurfelVector &surfels) override
  {
    m_parent.ShowSurfelCloud(surfels);
  }

  void ShowStableImage(const uint64 width, const uint64 height,
                       const FloatVector &depths, const FloatVector &colors,
                       const Vector3fVector &bearings, const Eigen::Affine3f &pose) override
  {
    m_parent.ShowStableImage(width, height, depths, colors, bearings, pose);
  }

  virtual bool HasShowKnownStateHull(const TKnownStateHullIndex index) override
  {
    ros::Publisher & pub = m_parent.GetKnownStateHullPublisherFromIndex(index);
    return pub.getNumSubscribers() > 0;
  }

  virtual void ShowKnownStateHull(const uint64 width,
                                  const uint64 height,
                                  const uint64 special_color_width,
                                  const uint64 special_color_height,
                                  const TKnownStateHullIndex index,
                                  const SurfelsUnknownSpace::Uint32Vector & data
                                  ) override
  {
    ros::Publisher & pub = m_parent.GetKnownStateHullPublisherFromIndex(index);
    m_parent.ShowKnownStateHull(width, height, special_color_width, special_color_height, pub, data);
  }

  ros::Publisher & GetKnownStateHullPublisherFromIndex(const TKnownStateHullIndex index)
  {
    return m_parent.GetKnownStateHullPublisherFromIndex(index);
  }

  void ShowCamera(const Eigen::Affine3f &pose,
                  const float padding_x,
                  const float padding_y,
                  const float size_z) override
  {
    m_parent.ShowCamera(pose, padding_x, padding_y, size_z);
  }

  SurfelsUnknownSpaceNode & m_parent;
};

SurfelsUnknownSpaceNode::SurfelsUnknownSpaceNode(ros::NodeHandle & nh): m_nh(nh)
{
  // init
  m_processing = false;
  m_requests_pending = 0;

  // parameters
  double param_double;
  std::string param_string;
  int param_int;

  m_nh.param<std::string>(PARAM_NAME_INPUT_TOPIC, param_string, PARAM_DEFAULT_INPUT_TOPIC);
  m_frame_state_sub = m_nh.subscribe(param_string, 1, &SurfelsUnknownSpaceNode::onFrameState, this);

  m_nh.param<std::string>(PARAM_NAME_SURFEL_UPLOAD_ACTION, param_string, PARAM_DEFAULT_SURFEL_UPLOAD_ACTION);
  m_set_surfel_cloud_action_server.reset(new SetSurfelCloudActionServer(m_nh, param_string,
                                         boost::bind(&SurfelsUnknownSpaceNode::onSetSurfelCloudAction, this, _1), false));

  m_nh.param<std::string>(PARAM_NAME_SURFEL_DOWNLOAD_ACTION, param_string, PARAM_DEFAULT_SURFEL_DOWNLOAD_ACTION);
  m_get_surfel_cloud_action_server.reset(new GetSurfelCloudActionServer(m_nh, param_string,
                                         boost::bind(&SurfelsUnknownSpaceNode::onGetSurfelCloudAction, this, _1), false));

  m_nh.param<std::string>(PARAM_NAME_GET_TIMERS_ACTION, param_string, PARAM_DEFAULT_GET_TIMERS_ACTION);
  m_get_timers_action_server.reset(new GetTimersActionServer(m_nh, param_string,
                                   boost::bind(&SurfelsUnknownSpaceNode::onGetTimersAction, this, _1), false));

  // debug publishers
  m_stable_image_pub = m_nh.advertise<sensor_msgs::Image>("stable_image",1);
  m_surfel_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("surfels_cloud",1);
  m_nuvg_display_pub = m_nh.advertise<visualization_msgs::MarkerArray>("nuvg",1);
  m_camera_display_pub = m_nh.advertise<visualization_msgs::MarkerArray>("camera",1);
  m_known_state_hull_xp_pub = m_nh.advertise<sensor_msgs::Image>("hull_xp",1);
  m_known_state_hull_xn_pub = m_nh.advertise<sensor_msgs::Image>("hull_xn",1);
  m_known_state_hull_yp_pub = m_nh.advertise<sensor_msgs::Image>("hull_yp",1);
  m_known_state_hull_yn_pub = m_nh.advertise<sensor_msgs::Image>("hull_yn",1);
  m_known_state_hull_zp_pub = m_nh.advertise<sensor_msgs::Image>("hull_zp",1);
  m_known_state_hull_zn_pub = m_nh.advertise<sensor_msgs::Image>("hull_zn",1);
  m_known_state_hull_xp_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_xp",1);
  m_known_state_hull_xn_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_xn",1);
  m_known_state_hull_yp_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_yp",1);
  m_known_state_hull_yn_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_yn",1);
  m_known_state_hull_zp_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_zp",1);
  m_known_state_hull_zn_f_pub = m_nh.advertise<sensor_msgs::Image>("hull_f_zn",1);

  m_nh.param<std::string>(PARAM_NAME_ACK_TOPIC, param_string, PARAM_DEFAULT_ACK_TOPIC);
  m_ack_publisher = m_nh.advertise<std_msgs::Empty>(param_string, 1);

  m_nh.param<bool>(PARAM_NAME_FRONTEL_NORMAL_AS_COLOR, m_frontel_normal_as_color, PARAM_DEFAULT_FRONTEL_NORMAL_AS_COLOR);

  SurfelsUnknownSpace::Config config;

  m_nh.param<double>(PARAM_NAME_MAX_RANGE, param_double, PARAM_DEFAULT_MAX_RANGE);
  config.max_range = param_double;

  m_nh.param<double>(PARAM_NAME_MIN_RANGE, param_double, PARAM_DEFAULT_MIN_RANGE);
  config.min_range = param_double;

  m_nh.param<double>(PARAM_NAME_HULL_UNKNOWN_SURFEL_MULT, param_double, PARAM_DEFAULT_HULL_UNKNOWN_SURFEL_MULT);
  config.unknown_surfels_radius_mult_pn = param_double;

  m_nh.param<double>(PARAM_NAME_SURFEL_THICKNESS, param_double, PARAM_DEFAULT_SURFEL_THICKNESS);
  config.surfel_thickness = param_double;

  // multiplier of the surfel radius
  m_nh.param<double>(PARAM_NAME_SURFEL_RADIUS_CREAT_MULT, param_double, PARAM_DEFAULT_SURFEL_RADIUS_CREAT_MULT);
  config.surfel_radius_mult = param_double;

  m_nh.param<bool>(PARAM_NAME_ENABLE_KNOWN_SPACE_FILTER, config.enable_known_space_filter, PARAM_DEFAULT_ENABLE_KNOWN_SPACE_FILTER);

  m_nh.param<double>(PARAM_NAME_CONFIDENCE_THRESHOLD, param_double, PARAM_DEFAULT_CONFIDENCE_THRESHOLD);
  config.dot_field_valid_th = param_double;

  m_nh.param<int>(PARAM_NAME_BACK_PADDING, param_int, PARAM_DEFAULT_BACK_PADDING);
  config.back_padding = param_int;

  m_nh.param<int>(PARAM_NAME_SIDE_PADDING, param_int, PARAM_DEFAULT_SIDE_PADDING);
  config.side_padding = param_int;

  m_nh.param<int>(PARAM_NAME_MAX_SURFELS_IN_MEM, param_int, PARAM_DEFAULT_MAX_SURFELS_IN_MEM);
  config.opencl_max_surfels_in_mem = param_int;

  m_nh.param<int>(PARAM_NAME_PROJECTION_THREADS, param_int, PARAM_DEFAULT_PROJECTION_THREADS);
  config.surfels_projection_threads = param_int;

  m_nh.param<int>(PARAM_NAME_DOWNSAMPLE_FACTOR, param_int, PARAM_DEFAULT_DOWNSAMPLE_FACTOR);
  config.downsample_factor = param_int;

  // start threads
  TIMER_INIT;

  SurfelsUnknownSpace::ILoggerPtr logger(new ROSLogger);
  SurfelsUnknownSpace::ITimerListenerPtr timer_listener(new TimerListener);
  SurfelsUnknownSpace::IVisualListenerPtr visual_listener(new VisualListener(*this));
  m_surfels_unknown_space.reset(new SurfelsUnknownSpace(config, logger, visual_listener, timer_listener));

  m_thread.reset(new boost::thread(&SurfelsUnknownSpaceNode::ProcessFrameWorker,this));

  m_get_surfel_cloud_action_server->start();
  m_set_surfel_cloud_action_server->start();
  m_get_timers_action_server->start();
}

SurfelsUnknownSpaceNode::~SurfelsUnknownSpaceNode()
{
}

ros::Publisher & SurfelsUnknownSpaceNode::GetKnownStateHullPublisherFromIndex(
  const SurfelsUnknownSpace::IVisualListener::TKnownStateHullIndex index)
{
  typedef SurfelsUnknownSpace::IVisualListener::TKnownStateHullIndex TI;
  switch (index)
  {
    case TI::XP: return m_known_state_hull_xp_pub;
    case TI::XN: return m_known_state_hull_xn_pub;
    case TI::YP: return m_known_state_hull_yp_pub;
    case TI::YN: return m_known_state_hull_yn_pub;
    case TI::ZP: return m_known_state_hull_zp_pub;
    case TI::ZN: return m_known_state_hull_zn_pub;
    case TI::XPF: return m_known_state_hull_xp_f_pub;
    case TI::XNF: return m_known_state_hull_xn_f_pub;
    case TI::YPF: return m_known_state_hull_yp_f_pub;
    case TI::YNF: return m_known_state_hull_yn_f_pub;
    case TI::ZPF: return m_known_state_hull_zp_f_pub;
    case TI::ZNF: return m_known_state_hull_zn_f_pub;
    default:
      ROS_ERROR("GetKnownStateHullPublisherFromIndex: invalid index: %d", int(index));
      return m_known_state_hull_xp_pub;
  }
}

void SurfelsUnknownSpaceNode::ProcessFrameWorker()
{
  ROS_INFO("surfels_unknown_space: process frame worker started.");

  while (true)
  {
    surfels_unknown_space_msgs::FrameStateConstPtr state_ptr;

    {
      boost::mutex::scoped_lock lock(m_mutex);

      while (!ros::isShuttingDown() && (m_requests_pending || !m_frame_state) )
        m_cond_var.timed_wait(lock,boost::posix_time::milliseconds(500));

      if (ros::isShuttingDown())
        return;

      state_ptr = m_frame_state;

      m_processing = true;

      m_frame_state.reset();
    }

    ROS_INFO("surfels_unknown_space: started processing.");

    ProcessFrame(state_ptr);

    m_ack_publisher.publish(std_msgs::Empty());

    ROS_INFO("surfels_unknown_space: end processing.");

    {
      boost::mutex::scoped_lock lock(m_mutex);
      m_processing = false;
      m_cond_var.notify_all();
    }
  }
}

void SurfelsUnknownSpaceNode::ProcessFrame(surfels_unknown_space_msgs::FrameStateConstPtr state_ptr)
{
  ROS_INFO("surfels_unknown_space: decoding message");
  const surfels_unknown_space_msgs::FrameState & state = *state_ptr;

  const uint64 input_width = state.width;
  const uint64 input_height = state.height;
  const SurfelsUnknownSpace::Intrinsics input_intrinsics(state.center_x, state.center_y, state.focal_x, state.focal_y,
                                                         state.width, state.height, 0.5, 3.0);

  Eigen::Affine3d pose;
  tf::poseMsgToEigen(state.pose, pose);

  FloatVector raw_depths;
  FloatVector raw_colors;
  // scope: decode depth and color
  {
    raw_depths.resize(input_width * input_height);
    for (uint64 y = 0; y < input_height; y++)
      for (uint64 x = 0; x < input_width; x++)
      {
        raw_depths[x + y * input_width] = state.input_depth[x + y * input_width] / 1000.0f;
      }

    raw_colors.resize(input_width * input_height * 3);
    for (uint64 y = 0; y < input_height; y++)
      for (uint64 x = 0; x < input_width; x++)
      {
        for (uint64 i = 0; i < 3; i++)
          raw_colors[(x + y * input_width) * 3 + i] = state.input_color[(x + y * input_width) * 3 + i] / 255.0f;
      }
  }

  ROS_INFO("surfels_unknown_space: processing.");
  m_surfels_unknown_space->ProcessFrame(input_width, input_height, raw_depths, raw_colors, pose.cast<float>(), input_intrinsics);
}

void SurfelsUnknownSpaceNode::onFrameState(const surfels_unknown_space_msgs::FrameStateConstPtr & state_ptr)
{
  boost::mutex::scoped_lock lock(m_mutex);
  ROS_INFO("surfels_unknown_space: Received frame state.");
  m_frame_state = state_ptr;
  m_cond_var.notify_all();
}

void SurfelsUnknownSpaceNode::onSetSurfelCloudAction(const surfels_unknown_space_msgs::SetSurfelCloudGoalConstPtr &goal)
{
  ROS_INFO("surfels_unknown_space: setting surfel cloud...");

  boost::mutex::scoped_lock lock(m_mutex);
  m_requests_pending++;
  {
    ROS_INFO("surfels_unknown_space: waiting for end of processing...");
    while (m_processing && !ros::isShuttingDown())
      m_cond_var.timed_wait(lock,boost::posix_time::milliseconds(500));

    if (ros::isShuttingDown())
      return;
  }

  PointSurfelCloud pcl_cloud;
  pcl::fromROSMsg(goal->pointcloud,pcl_cloud);

  const uint64 surfels_size = pcl_cloud.size();
  SurfelsUnknownSpace::SurfelVector surfels;
  surfels.reserve(surfels_size);

  for (uint64 i = 0; i < surfels_size; i++)
  {
    const pcl::PointSurfel & pcl_surfel = pcl_cloud[i];

    const Eigen::Vector3f position(pcl_surfel.x,pcl_surfel.y,pcl_surfel.z);
    const float radius = pcl_surfel.radius;
    const Eigen::Vector3f normal(pcl_surfel.normal_x,pcl_surfel.normal_y,pcl_surfel.normal_z);
    const bool is_surfel = (pcl_surfel.confidence > 0.5f);

    SurfelsUnknownSpace::Surfel surfel(position,radius,normal,is_surfel);

    if (is_surfel)
    {
      surfel.cr = pcl_surfel.r;
      surfel.cg = pcl_surfel.g;
      surfel.cb = pcl_surfel.b;
    }

    surfels.push_back(surfel);
  }

  m_surfels_unknown_space->SetSurfels(surfels);

  surfels_unknown_space_msgs::SetSurfelCloudResult result;
  result.ok = true;
  m_set_surfel_cloud_action_server->setSucceeded(result);
  m_requests_pending--;
  m_cond_var.notify_all();

  ROS_INFO("surfels_unknown_space: set surfel cloud (size %u)", (unsigned)surfels_size);
}

void SurfelsUnknownSpaceNode::onGetSurfelCloudAction(const surfels_unknown_space_msgs::GetSurfelCloudGoalConstPtr &goal)
{
  ROS_INFO("surfels_unknown_space: getting surfel cloud...");

  boost::mutex::scoped_lock lock(m_mutex);
  m_requests_pending++;
  {
    ROS_INFO("surfels_unknown_space: waiting for end of processing...");
    while (m_processing && !ros::isShuttingDown())
      m_cond_var.timed_wait(lock,boost::posix_time::milliseconds(500));

    if (ros::isShuttingDown())
      return;
  }

  surfels_unknown_space_msgs::GetSurfelCloudResult result;

  SurfelsUnknownSpace::SurfelVector surfels = m_surfels_unknown_space->GetSurfels();

  const uint64 surfels_size = surfels.size();
  PointSurfelCloud pcl_cloud;
  pcl_cloud.reserve(surfels_size);
  for (uint64 i = 0; i < surfels_size; i++)
  {
    const SurfelsUnknownSpace::Surfel & surfel = surfels[i];
    if (surfel.erased)
      continue;

    pcl::PointSurfel pcl_surfel;
    pcl_surfel.x = surfel.position.x();
    pcl_surfel.y = surfel.position.y();
    pcl_surfel.z = surfel.position.z();
    pcl_surfel.radius = surfel.radius;
    pcl_surfel.normal_x = surfel.normal.x();
    pcl_surfel.normal_y = surfel.normal.y();
    pcl_surfel.normal_z = surfel.normal.z();
    pcl_surfel.curvature = 0.0;

    if (surfel.is_surfel)
    {
      pcl_surfel.r = surfel.cr;
      pcl_surfel.g = surfel.cg;
      pcl_surfel.b = surfel.cb;
      pcl_surfel.confidence = 1.0;
    }
    else
    {
      pcl_surfel.r = 0;
      pcl_surfel.g = 0;
      pcl_surfel.b = 255;
      pcl_surfel.confidence = 0.0;
    }
    pcl_surfel.a = 255;

    pcl_cloud.push_back(pcl_surfel);
  }
  pcl_cloud.width = pcl_cloud.size();
  pcl_cloud.height = 1;

  pcl::toROSMsg(pcl_cloud,result.pointcloud);

  m_get_surfel_cloud_action_server->setSucceeded(result);
  m_requests_pending--;
  m_cond_var.notify_all();

  ROS_INFO("surfels_unknown_space: surfel cloud sent (size %u)", (unsigned)surfels_size);
}

void SurfelsUnknownSpaceNode::onGetTimersAction(const surfels_unknown_space_msgs::GetTimersGoalConstPtr &)
{
  ROS_INFO("surfels_unknown_space: timers requested.");

  boost::mutex::scoped_lock lock(m_mutex);
  m_requests_pending++;
  {
    ROS_INFO("surfels_unknown_space: waiting for end of processing...");
    while (m_processing && !ros::isShuttingDown())
      m_cond_var.timed_wait(lock,boost::posix_time::milliseconds(500));

    if (ros::isShuttingDown())
      return;
  }

  surfels_unknown_space_msgs::GetTimersResult result;
  result.timers = TIMER_GETSTRING;
  m_get_timers_action_server->setSucceeded(result);

  m_requests_pending--;
  m_cond_var.notify_all();
  ROS_INFO("surfels_unknown_space: timers sent.");
}

int main(int argc, char ** argv)
{
  ros::init(argc,argv,"surfels_unknown_space");

  ros::NodeHandle nh("~");

  SurfelsUnknownSpaceNode susn(nh);

  ros::spin();

  return 0;
}


