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

#include "surfels_unknown_space.h"
#include "surfels_unknown_space_node.h"

#include "timer_stub.h"

#include <visualization_msgs/MarkerArray.h>

SurfelsUnknownSpace::SurfelsUnknownSpace(ros::NodeHandle & nh): m_nh(nh)
{
  // init
  m_processing = false;
  m_requests_pending = 0;

  // parameters
  double param_double;
  std::string param_string;
  int param_int;

  m_nh.param<std::string>(PARAM_NAME_INPUT_TOPIC, param_string, PARAM_DEFAULT_INPUT_TOPIC);
  m_frame_state_sub = m_nh.subscribe(param_string, 1, &SurfelsUnknownSpace::onFrameState, this);

  m_nh.param<std::string>(PARAM_NAME_SURFEL_UPLOAD_ACTION, param_string, PARAM_DEFAULT_SURFEL_UPLOAD_ACTION);
  m_set_surfel_cloud_action_server.reset(new SetSurfelCloudActionServer(m_nh, param_string,
                                         boost::bind(&SurfelsUnknownSpace::onSetSurfelCloudAction, this, _1), false));

  m_nh.param<std::string>(PARAM_NAME_SURFEL_DOWNLOAD_ACTION, param_string, PARAM_DEFAULT_SURFEL_DOWNLOAD_ACTION);
  m_get_surfel_cloud_action_server.reset(new GetSurfelCloudActionServer(m_nh, param_string,
                                         boost::bind(&SurfelsUnknownSpace::onGetSurfelCloudAction, this, _1), false));

  m_nh.param<std::string>(PARAM_NAME_GET_TIMERS_ACTION, param_string, PARAM_DEFAULT_GET_TIMERS_ACTION);
  m_get_timers_action_server.reset(new GetTimersActionServer(m_nh, param_string,
                                   boost::bind(&SurfelsUnknownSpace::onGetTimersAction, this, _1), false));

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

  m_nh.param<double>(PARAM_NAME_MAX_RANGE, param_double, PARAM_DEFAULT_MAX_RANGE);
  m_max_range = param_double;

  m_nh.param<double>(PARAM_NAME_MIN_RANGE, param_double, PARAM_DEFAULT_MIN_RANGE);
  m_min_range = param_double;

  m_nh.param<double>(PARAM_NAME_HULL_UNKNOWN_SURFEL_MULT, param_double, PARAM_DEFAULT_HULL_UNKNOWN_SURFEL_MULT);
  m_unknown_surfels_radius_mult_pn = param_double;

  m_nh.param<double>(PARAM_NAME_SURFEL_THICKNESS, param_double, PARAM_DEFAULT_SURFEL_THICKNESS);
  m_surfel_thickness = param_double;

  // multiplier of the surfel radius
  m_nh.param<double>(PARAM_NAME_SURFEL_RADIUS_CREAT_MULT, param_double, PARAM_DEFAULT_SURFEL_RADIUS_CREAT_MULT);
  m_surfel_radius_mult = param_double;

  m_nh.param<bool>(PARAM_NAME_ENABLE_KNOWN_SPACE_FILTER, m_enable_known_space_filter, PARAM_DEFAULT_ENABLE_KNOWN_SPACE_FILTER);

  m_nh.param<double>(PARAM_NAME_CONFIDENCE_THRESHOLD, param_double, PARAM_DEFAULT_CONFIDENCE_THRESHOLD);
  m_dot_field_safety_th = param_double;
  m_dot_field_valid_th = param_double;

  m_nh.param<int>(PARAM_NAME_BACK_PADDING, param_int, PARAM_DEFAULT_BACK_PADDING);
  m_back_padding = param_int;

  m_nh.param<int>(PARAM_NAME_SIDE_PADDING, param_int, PARAM_DEFAULT_SIDE_PADDING);
  m_side_padding = param_int;

  m_nh.param<int>(PARAM_NAME_MAX_SURFELS_IN_MEM, param_int, PARAM_DEFAULT_MAX_SURFELS_IN_MEM);
  m_opencl_max_surfels_in_mem = param_int;

  m_nh.param<int>(PARAM_NAME_PROJECTION_THREADS, param_int, PARAM_DEFAULT_PROJECTION_THREADS);
  m_surfels_projection_threads = param_int;

  m_nh.param<int>(PARAM_NAME_DOWNSAMPLE_FACTOR, param_int, PARAM_DEFAULT_DOWNSAMPLE_FACTOR);
  m_downsample_factor = param_int;

  // start threads
  initOpenCL();

  TIMER_INIT;

  m_thread.reset(new boost::thread(&SurfelsUnknownSpace::ProcessFrameWorker,this));

  m_get_surfel_cloud_action_server->start();
  m_set_surfel_cloud_action_server->start();
  m_get_timers_action_server->start();
}

SurfelsUnknownSpace::~SurfelsUnknownSpace()
{
}

float SurfelsUnknownSpace::getVoxelSideAtDistance(const float distance, float min_range) const
{
  if (distance < min_range)
    return min_range * 1.0f / m_intrinsics->focal_avg;

  return distance * 1.0f / m_intrinsics->focal_avg;
}

float SurfelsUnknownSpace::getVoxelSideAtDistance(const float distance) const
{
  return getVoxelSideAtDistance(distance, m_min_range);
}

float SurfelsUnknownSpace::getVoxelCountAtDistanceF(const float distance) const
{
  if (distance < m_min_range)
    return (distance / getVoxelSideAtDistance(0)) + m_back_padding;

  const float result = (m_min_range / getVoxelSideAtDistance(0) +
      (std::log(distance) - std::log(m_min_range)) * m_intrinsics->focal_avg) + m_back_padding;

  return result;
}

SurfelsUnknownSpace::int64 SurfelsUnknownSpace::getVoxelCountAtDistance(const float distance) const
{
  const float rounded = std::round(getVoxelCountAtDistanceF(distance));
  if (rounded < 0)
    return -1;

  return rounded;
}

float SurfelsUnknownSpace::getDistanceFromVoxelCount(const uint64 count)
{
  const float zero_side = getVoxelSideAtDistance(0);
  const float diff = (float(count) - m_back_padding) * zero_side;
  if (diff < m_min_range)
    return diff;

  const float K = (1.0 / m_intrinsics->focal_avg);

  const float result = std::exp(K * (count - m_back_padding) + std::log(m_min_range) - K * (m_min_range / zero_side));
  return result;
}

Eigen::Vector3f SurfelsUnknownSpace::getBearingForPixel(const uint64 x,const uint64 y)
{
  const Eigen::Vector3f result(float(x) - m_intrinsics->center_x,float(y) - m_intrinsics->center_y,m_intrinsics->focal_avg);
  return result.normalized();
}

SurfelsUnknownSpace::Vector3fVector SurfelsUnknownSpace::getAllBearings(const uint64 width,const uint64 height)
{
  Vector3fVector result(width * height);
  for (uint64 y = 0; y < height; y++)
    for (uint64 x = 0; x < width; x++)
      result[x + y * width] = getBearingForPixel(x,y);
  return result;
}

void SurfelsUnknownSpace::SplitNearSurfels(const Eigen::Affine3f & pose)
{
  SurfelVector new_surfels;
  const uint64 surfels_size = m_surfels.size();
  uint64 counter = 0;

  for (uint64 i = 0; i < surfels_size; i++)
  {
    const Surfel & surfel = m_surfels[i];
    if (surfel.erased)
      continue;

    const float distance = (pose.translation() - surfel.position).norm();
    const float radius = surfel.radius;
    const Eigen::Vector3f normal = surfel.normal;
    const float new_radius = std::sqrt(3.0f) * getVoxelSideAtDistance(distance) / 2.0;
    if (radius > new_radius * 2.0)
    {
      // if surfel near enough
      // split the surfel into four new surfels

      uint64 lower_i;
      // build reference frame on surfel with normal = axis z
      for (uint64 ai = 0; ai < 3; ai++) // find lower normal coord
        if (ai == 0 || (std::abs(normal[ai]) < std::abs(normal[lower_i])))
          lower_i = ai;
      const Eigen::Vector3f best_axis = Eigen::Vector3f::Unit(lower_i);
      const Eigen::Vector3f axis_z = normal;
      const Eigen::Vector3f axis_x = best_axis - best_axis.dot(normal) * normal;
      const Eigen::Vector3f axis_y = axis_z.cross(axis_x);

      Surfel nf[4];
      for (int64 dy = 0; dy <= 1; dy++)
        for (int64 dx = 0; dx <= 1; dx++)
        {
          const float fdx = dx ? 1 : -1;
          const float fdy = dy ? 1 : -1;
          const Eigen::Vector3f transl = (axis_x * fdx + axis_y * fdy) * new_radius / sqrt(2.0f);
          const uint64 di = dy * 2 + dx;
          nf[di] = surfel;
          nf[di].position = surfel.position + transl;
          nf[di].radius = radius / 2.0f;
        }

      m_surfels[i] = nf[0];
      for (uint64 h = 1; h < 4; h++)
        new_surfels.push_back(nf[h]);

      counter++;
    }
  }
  for (const Surfel & nsurf : new_surfels)
    CreateSurfel(nsurf);

  std::cout << "Split " << counter << " surfels." << std::endl;
}

template <typename T>
std::vector<T> SurfelsUnknownSpace::DownSample(const uint64 width,
                                               const uint64 height,
                                               const uint64 towidth,
                                               const uint64 toheight,
                                               const uint64 step,
                                               const T mult,
                                               const std::vector<T> & vec
                                               )
{
  const uint64 tosize = towidth * toheight;
  std::vector<T> result(tosize * step);

  for (uint64 y = 0; y < toheight; y++)
    for (uint64 x = 0; x < towidth; x++)
    {
      const uint64 toi = x + y * towidth;
      const uint64 fromx = x * width / towidth;
      const uint64 fromy = y * height / toheight;
      const uint64 fromi = fromx + fromy * width;
      for (uint64 si = 0; si < step; si++)
        result[toi * step + si] = mult * vec[fromi * step + si];
    }
  return result;
}

SurfelsUnknownSpace::uint64 SurfelsUnknownSpace::CreateSurfel(const Surfel & surfel)
{
  uint64 pos;
  if (m_deleted_surfels.empty())
  {
    m_surfels.push_back(surfel);
    pos = m_surfels.size() - 1;
  }
  else
  {
    pos = *m_deleted_surfels.begin();
    m_surfels[pos] = surfel;
    m_deleted_surfels.erase(m_deleted_surfels.begin());
  }

  return pos;
}

void SurfelsUnknownSpace::DeleteSurfel(const uint64 index)
{
  if (m_surfels[index].erased)
    return;
  if (index + 1 == m_surfels.size())
  {
    m_surfels.pop_back();
    while (!m_surfels.empty() && m_surfels.back().erased)
    {
      m_deleted_surfels.erase(m_surfels.size() - 1);
      m_surfels.pop_back();
    }
  }
  else
  {
    m_surfels[index].erased = true;
    m_deleted_surfels.insert(index);
  }
}

void SurfelsUnknownSpace::ComputeApproximateFrustumBoundingBox(const Eigen::Affine3f & pose,
                                                              Eigen::Vector3f & bounding_box_min,
                                                              Eigen::Vector3f & bounding_box_max)
{
  const float center_dist = m_intrinsics->max_range / 2.0f;
  const float lateral_dist_xp = (m_intrinsics->width - m_intrinsics->center_x) *
                                (m_intrinsics->max_range / m_intrinsics->focal_x);
  const float lateral_dist_yp = (m_intrinsics->height - m_intrinsics->center_y) *
                                (m_intrinsics->max_range / m_intrinsics->focal_y);
  const float lateral_dist_xn = (m_intrinsics->center_x) *
                                (m_intrinsics->max_range / m_intrinsics->focal_x);
  const float lateral_dist_yn = (m_intrinsics->center_y) *
                                (m_intrinsics->max_range / m_intrinsics->focal_y);
  const Eigen::Vector3f center(0.0f, 0.0f, center_dist);
  bounding_box_max = bounding_box_min = (pose * center);
  // bounding box
  for (int64 x = -1; x <= 1; x++)
    for (int64 y = -1; y <= 1; y++)
      for (int64 z = -1; z <= 1; z++)
      {
        if (!x || !y || !z)
          continue;

        Eigen::Vector3f vertex;
        vertex.x() = (x > 0) ? lateral_dist_xp : -lateral_dist_xn;
        vertex.y() = (y > 0) ? lateral_dist_yp : -lateral_dist_yn;
        vertex.z() = center_dist + center_dist * z;

        const Eigen::Vector3f world_vertex = pose * vertex;
        bounding_box_max = bounding_box_max.array().max(world_vertex.array());
        bounding_box_min = bounding_box_min.array().min(world_vertex.array());
      }
  bounding_box_max += Eigen::Vector3f::Ones() * 0.1f;
  bounding_box_min -= Eigen::Vector3f::Ones() * 0.1f;
}

void SurfelsUnknownSpace::ProcessFrame(surfels_unknown_space_msgs::FrameStateConstPtr state_ptr)
{

  pcl::console::TicToc tictoc;
  pcl::console::TicToc total_tictoc;

  const surfels_unknown_space_msgs::FrameState & state = *state_ptr;

  TIMER_NEWFRAME;
  TIMER_START(TIMERDEF_SUBSAMPLE);

  const uint64 input_width = state.width;
  const uint64 input_height = state.height;
  const uint64 width = state.width / m_downsample_factor;
  const uint64 height = state.height / m_downsample_factor;
  const Intrinsics input_intrinsics(state.center_x,state.center_y,state.focal_x,state.focal_y,
                                    state.width,state.height,m_min_range,m_max_range);
  m_intrinsics = boost::make_shared<Intrinsics>(input_intrinsics.RescaleInt(m_downsample_factor));

  FloatVector raw_depths;
  FloatVector raw_colors;
  // scope: decode depth and color
  {
    raw_depths.resize(input_width * input_height);
    for (uint64 y = 0; y < input_height; y++)
      for (uint64 x = 0; x < input_width; x++)
      {
        raw_depths[x + y * input_width] = state.input_depth[x + y * input_width] / 1000.0f;
        if (raw_depths[x + y * input_width] > m_max_range - 0.1f)
          raw_depths[x + y * input_width] = 0.0f;
      }

    raw_colors.resize(input_width * input_height * 3);
    for (uint64 y = 0; y < input_height; y++)
      for (uint64 x = 0; x < input_width; x++)
      {
        for (uint64 i = 0; i < 3; i++)
          raw_colors[(x + y * input_width) * 3 + i] = state.input_color[(x + y * input_width) * 3 + i] / 255.0f;
      }
  }
  const FloatVector depths = DownSample<float>(input_width,input_height,width,height,1,1.0,raw_depths);
  const FloatVector colors = DownSample<float>(input_width,input_height,width,height,3,1.0,raw_colors);

  TIMER_STOP(TIMERDEF_SUBSAMPLE);
  TIMER_START(TIMERDEF_INIT);

  Eigen::Affine3f pose = Eigen::Affine3f::Identity();
  {
    Eigen::Affine3d posed;
    tf::poseMsgToEigen(state.pose,posed);
    pose = posed.cast<float>();
  }

  OpenCLUpdateCurrentPoseAndIntrinsics(pose, m_min_range, m_max_range,
                                       m_dot_field_valid_th, m_dot_field_safety_th, m_back_padding);

  total_tictoc.tic();

  const uint64 count_at_max_range = getVoxelCountAtDistance(m_max_range);
  const uint64 count_at_min_range = getVoxelCountAtDistance(m_min_range);
  const int64 count_at_zero = getVoxelCountAtDistance(0.0f);

  TIMER_STOP(TIMERDEF_INIT);

  std::cout << "Computing bearings." << std::endl;
  tictoc.tic();
  Vector3fVector bearings = getAllBearings(width,height);
  tictoc.toc_print();

  TIMER_START(TIMERDEF_BEARINGS);
  std::cout << "Computing bearings OpenCL." << std::endl;
  tictoc.tic();
  OpenCLGetAllBearings(bearings);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_BEARINGS);

  TIMER_START(TIMERDEF_SPLITTING);
  std::cout << "Splitting surfels." << std::endl;
  tictoc.tic();
  SplitNearSurfels(pose);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_SPLITTING);

  TIMER_START(TIMERDEF_OBSERVED_FIELD);
  std::cout << "Generating observed space field OpenCL." << std::endl;
  tictoc.tic();
  OpenCLGenerateObservedSpaceField(count_at_max_range, depths);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_OBSERVED_FIELD);

  TIMER_START(TIMERDEF_DOT_FIELD);
  std::cout << "Projecting surfels OpenCL." << std::endl;
  tictoc.tic();
  OpenCLProjectAndDeleteSurfels(pose,m_surfels,count_at_max_range,colors);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_DOT_FIELD);

  TIMER_START(TIMERDEF_DOT_HULL);
  std::cout << "Filtering known state hull OpenCL." << std::endl;
  tictoc.tic();
  OpenCLFilterKnownStateHullPN(count_at_max_range);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_DOT_HULL);

  TIMER_START(TIMERDEF_KNOWN_SPACE);
  std::cout << "Building known space field OpenCL." << std::endl;
  tictoc.tic();
  OpenCLBuildKnownSpaceField(count_at_max_range);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_KNOWN_SPACE);

  TIMER_START(TIMERDEF_KNOWN_SPACE_FILTERING);
  std::cout << "Filtering known space field OpenCL." << std::endl;
  tictoc.tic();
  OpenCLFilterKnownSpaceField(count_at_max_range);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_KNOWN_SPACE_FILTERING);

  TIMER_START(TIMERDEF_CREATION);
  std::cout << "Creating new surfels OpenCL." << std::endl;
  tictoc.tic();
  OpenCLCreateSurfels(colors, count_at_max_range);
  tictoc.toc_print();
  TIMER_STOP(TIMERDEF_CREATION);

  std::cout << "Total processing: ";
  total_tictoc.toc_print();

  std::cout << "Publishing: ";
  tictoc.tic();

  ShowStableImage(width, height, depths, colors, bearings, pose);
  ShowNUVG(pose, count_at_max_range, count_at_min_range, count_at_zero);
  ShowCamera(pose, m_side_padding, m_side_padding, m_min_range);
  ShowKnownStateHull(count_at_max_range,height,count_at_min_range,-1,false,
                          m_known_state_hull_xn_pub,m_opencl_known_state_hull_xn);
  ShowKnownStateHull(count_at_max_range,height,count_at_min_range,-1,false,
                          m_known_state_hull_xp_pub,m_opencl_known_state_hull_xp);
  ShowKnownStateHull(count_at_max_range,width,count_at_min_range,-1,true,
                          m_known_state_hull_yn_pub,m_opencl_known_state_hull_yn);
  ShowKnownStateHull(count_at_max_range,width,count_at_min_range,-1,true,
                          m_known_state_hull_yp_pub,m_opencl_known_state_hull_yp);
  ShowKnownStateHull(width,height,-1,-1,true,
                          m_known_state_hull_zn_pub,m_opencl_known_state_hull_zn);
  ShowKnownStateHull(width,height,-1,-1,true,
                          m_known_state_hull_zp_pub,m_opencl_known_state_hull_zp);
  ShowKnownStateHull(count_at_max_range,height,count_at_min_range,-1,false,
                          m_known_state_hull_xn_f_pub,m_opencl_known_state_hull_xn_filtered);
  ShowKnownStateHull(count_at_max_range,height,count_at_min_range,-1,false,
                          m_known_state_hull_xp_f_pub,m_opencl_known_state_hull_xp_filtered);
  ShowKnownStateHull(count_at_max_range,width,count_at_min_range,-1,true,
                          m_known_state_hull_yn_f_pub,m_opencl_known_state_hull_yn_filtered);
  ShowKnownStateHull(count_at_max_range,width,count_at_min_range,-1,true,
                          m_known_state_hull_yp_f_pub,m_opencl_known_state_hull_yp_filtered);
  ShowKnownStateHull(width,height,-1,-1,true,
                          m_known_state_hull_zn_f_pub,m_opencl_known_state_hull_zn_filtered);
  ShowKnownStateHull(width,height,-1,-1,true,
                          m_known_state_hull_zp_f_pub,m_opencl_known_state_hull_zp_filtered);
  ShowSurfelCloud();
  tictoc.toc_print();

  const uint64 mem_usage = m_surfels.size() * sizeof(Surfel);
  std::cout << "Surfels memory usage: " << mem_usage << " (" << mem_usage / 1000000 << " MB)" << std::endl;
}

