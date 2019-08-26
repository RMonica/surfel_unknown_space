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
#include "surfels_unknown_space.h"
#include "timer_stub.h"

void SurfelsUnknownSpace::ProcessFrameWorker()
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

void SurfelsUnknownSpace::onFrameState(const surfels_unknown_space_msgs::FrameStateConstPtr & state_ptr)
{
  boost::mutex::scoped_lock lock(m_mutex);
  ROS_INFO("surfels_unknown_space: Received frame state.");
  m_frame_state = state_ptr;
  m_cond_var.notify_all();
}

void SurfelsUnknownSpace::onSetSurfelCloudAction(const surfels_unknown_space_msgs::SetSurfelCloudGoalConstPtr &goal)
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
  m_surfels.clear();
  m_surfels.reserve(surfels_size);

  for (uint64 i = 0; i < surfels_size; i++)
  {
    const pcl::PointSurfel & pcl_surfel = pcl_cloud[i];

    const Eigen::Vector3f position(pcl_surfel.x,pcl_surfel.y,pcl_surfel.z);
    const float radius = pcl_surfel.radius;
    const Eigen::Vector3f normal(pcl_surfel.normal_x,pcl_surfel.normal_y,pcl_surfel.normal_z);
    const bool is_surfel = (pcl_surfel.confidence > 0.5f);

    Surfel surfel(position,radius,normal,is_surfel);

    if (is_surfel)
    {
      surfel.cr = pcl_surfel.r;
      surfel.cg = pcl_surfel.g;
      surfel.cb = pcl_surfel.b;
    }

    m_surfels.push_back(surfel);
  }

  surfels_unknown_space_msgs::SetSurfelCloudResult result;
  result.ok = true;
  m_set_surfel_cloud_action_server->setSucceeded(result);
  m_requests_pending--;
  m_cond_var.notify_all();

  ROS_INFO("surfels_unknown_space: surfel cloud set (size %u)", (unsigned)surfels_size);
}

void SurfelsUnknownSpace::onGetSurfelCloudAction(const surfels_unknown_space_msgs::GetSurfelCloudGoalConstPtr &goal)
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

  const uint64 surfels_size = m_surfels.size();
  PointSurfelCloud pcl_cloud;
  pcl_cloud.reserve(surfels_size);
  for (uint64 i = 0; i < surfels_size; i++)
  {
    const Surfel & surfel = m_surfels[i];
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

  ROS_INFO("surfels_unknown_space: surfel cloud get (size %u)", (unsigned)surfels_size);
}

void SurfelsUnknownSpace::onGetTimersAction(const surfels_unknown_space_msgs::GetTimersGoalConstPtr &)
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

  SurfelsUnknownSpace sus(nh);

  ros::spin();

  return 0;
}


