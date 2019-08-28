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

#include "state_image_reader.h"

// ROS
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>

// STL
#include <string>
#include <vector>
#include <stdint.h>
#include <fstream>

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// Boost
#include <boost/lexical_cast.hpp>

#include <surfels_unknown_space_msgs/FrameState.h>
#include <surfels_unknown_space_msgs/GetSurfelCloudAction.h>

#include "state_point_type.h"

class StateImageReader
{
  public:
  typedef std::vector<std::string> StringVector;
  typedef uint64_t uint64;
  typedef int64_t int64;
  typedef pcl::PointCloud<pcl::PointSurfel> PointSurfelCloud;

  StateImageReader(ros::NodeHandle & nh): m_nh(nh),
    m_elastic_ui_download_ac("/elastic_ui_download", true)
  {
    std::string param_string;
    int param_int;
    double param_double;

    m_nh.param<std::string>(PARAM_NAME_ACK_TOPIC, param_string, PARAM_DEFAULT_ACK_TOPIC);
    m_ack_subscriber = m_nh.subscribe(param_string, 1, &StateImageReader::onAck, this);

    m_nh.param<std::string>(PARAM_NAME_FRAME_STATE_TOPIC, param_string, PARAM_DEFAULT_FRAME_STATE_TOPIC);
    m_state_publisher = m_nh.advertise<surfels_unknown_space_msgs::FrameState>(param_string, 1);

    m_nh.param<std::string>(PARAM_NAME_INPUT_FILE_PREFIX, param_string, PARAM_DEFAULT_INPUT_FILE_PREFIX);
    m_save_prefix = param_string;

    m_nh.param<double>(PARAM_NAME_WAIT_INITIAL, param_double, PARAM_DEFAULT_WAIT_INITIAL);
    m_initial_timer = m_nh.createTimer(ros::Duration(param_double), &StateImageReader::onInitialTimer, this, true);

    m_nh.param<std::string>(PARAM_NAME_SAVE_CLOUD_NAME, param_string, PARAM_DEFAULT_SAVE_CLOUD_NAME);
    m_save_cloud_name = param_string;

    m_nh.param<std::string>(PARAM_NAME_TF_REFERENCE_FRAME, param_string, PARAM_DEFAULT_TF_REFERENCE_FRAME);
    m_tf_reference_frame_name = param_string;

    m_nh.param<std::string>(PARAM_NAME_TF_CAMERA_FRAME, param_string, PARAM_DEFAULT_TF_CAMERA_FRAME);
    m_tf_camera_frame_name = param_string;

    m_counter = 0;
  }

  void onInitialTimer(const ros::TimerEvent&)
  {
    PublishNext();
  }

  void onAck(const std_msgs::Empty &)
  {
    PublishNext();
  }

  void SaveCloud()
  {
    ROS_INFO("state_image_reader: downloading and saving point cloud.");
    if (!m_elastic_ui_download_ac.waitForServer(ros::Duration(10.0f)))
    {
      ROS_ERROR("state_image_reader: could not connect to server!");
      return;
    }

    ROS_INFO("state_image_reader: sending goal.");
    surfels_unknown_space_msgs::GetSurfelCloudGoal goal;
    m_elastic_ui_download_ac.sendGoal(goal);

    const bool finished_before_timeout = m_elastic_ui_download_ac.waitForResult();
    if (!finished_before_timeout)
    {
      ROS_ERROR("state_image_reader: result not received!");
      return;
    }

    actionlib::SimpleClientGoalState state = m_elastic_ui_download_ac.getState();

    if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR("state_image_reader: action did not succeed, state is %s.",state.toString().c_str());
      return;
    }

    const std::string counter_string = boost::lexical_cast<std::string>(m_counter);
    const std::string filename = m_save_prefix + counter_string + "_" + m_save_cloud_name + ".pcd";
    ROS_INFO("state_image_reader: saving pointcloud: %s",filename.c_str());
    surfels_unknown_space_msgs::GetSurfelCloudResult result = *m_elastic_ui_download_ac.getResult();

    PointSurfelCloud cloud;
    pcl::fromROSMsg(result.pointcloud, cloud);
    if (cloud.empty())
    {
      ROS_INFO("state_image_reader: point cloud not saved, it is empty.");
    }
    else
    {
      if (!pcl::io::savePCDFileBinary(filename, cloud))
        ROS_INFO("state_image_reader: saved file %s",filename.c_str());
      else
        ROS_ERROR("state_image_reader: could not save file %s",filename.c_str());
    }
  }

  void PublishNext()
  {
    const std::string counter_string = boost::lexical_cast<std::string>(m_counter);
    const std::string full_prefix = m_save_prefix + counter_string + "_";

    surfels_unknown_space_msgs::FrameState state_msg;

    std::ifstream mfile(full_prefix + "intrinsics.txt");
    mfile >> state_msg.seq;
    mfile >> state_msg.width >> state_msg.height;
    mfile >> state_msg.focal_x >> state_msg.focal_y;
    mfile >> state_msg.center_x >> state_msg.center_y;

    const uint64 width = state_msg.width;
    const uint64 height = state_msg.height;

    if (!mfile)
    {
      ROS_ERROR_STREAM("state_image_reader: Error while loading " << full_prefix + "intrinsics.txt");
      SaveCloud();
      return;
    }

    std::ifstream pfile(full_prefix + "pose.matrix");
    Eigen::Affine3d mat;
    for (uint64 y = 0; y < 3; y++)
    {
      float v;
      for (uint64 x = 0; x < 3; x++)
      {
        pfile >> v;
        mat.linear()(y, x) = v;
      }
      pfile >> v;
      mat.translation()[y] = v;
    }
    tf::poseEigenToMsg(mat, state_msg.pose);

    if (!pfile)
    {
      ROS_ERROR_STREAM("state_image_reader: Error while loading " << full_prefix + "pose.matrix");
      SaveCloud();
      return;
    }

    pcl::PointCloud<StatePointType> cloud;
    const std::string filename = full_prefix + "cloud.pcd";
    if (pcl::io::loadPCDFile(filename, cloud))
    {
      ROS_ERROR("state_image_reader:  could not save file %s",filename.c_str());
      SaveCloud();
      return;
    }

    state_msg.input_depth.resize(height * width);
    state_msg.input_color.resize((height * width) * 3);

    for (uint64 y = 0; y < height; y++)
      for (uint64 x = 0; x < width; x++)
      {
        const uint64 i2 = y * width + x;
        const StatePointType & point = cloud[i2];

        state_msg.input_color[i2 * 3 + 0] = point.input_r;
        state_msg.input_color[i2 * 3 + 1] = point.input_g;
        state_msg.input_color[i2 * 3 + 2] = point.input_b;

        state_msg.input_depth[i2] = point.input_depth;

        cloud.push_back(point);
      }

    ROS_INFO("state_image_reader: publishing state %u", (unsigned)m_counter);
    m_state_publisher.publish(state_msg);

    {
      tf::Transform transform;
      tf::poseEigenToTF(mat, transform);
      m_tf_broadcaster.sendTransform(
        tf::StampedTransform(transform, ros::Time::now(), m_tf_reference_frame_name, m_tf_camera_frame_name));
    }

    m_counter++;
  }

  private:
  ros::NodeHandle & m_nh;

  ros::Subscriber m_ack_subscriber;
  ros::Publisher m_state_publisher;

  actionlib::SimpleActionClient<surfels_unknown_space_msgs::GetSurfelCloudAction> m_elastic_ui_download_ac;

  ros::Timer m_initial_timer;

  std::string m_save_prefix;
  std::string m_save_cloud_name;
  uint64 m_counter;

  std::string m_tf_reference_frame_name;
  std::string m_tf_camera_frame_name;

  tf::TransformBroadcaster m_tf_broadcaster;
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "state_image_reader");
  ros::NodeHandle nh("~");

  StateImageReader sir(nh);

  ros::spin();

  return 0;
}
