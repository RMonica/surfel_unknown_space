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

#include "surfels_unknown_space_node_h.h"

// ROS
#include <visualization_msgs/MarkerArray.h>

// STL
#include <fstream>

static std_msgs::ColorRGBA BuildColorMsg(const float r, const float g, const float b)
{
  std_msgs::ColorRGBA result;
  result.r = r;
  result.g = g;
  result.b = b;
  result.a = 1.0f;
  return result;
}

static void CylinderFromLine(const Eigen::Vector3f & start, const Eigen::Vector3f & end,
                             visualization_msgs::Marker & cyl, int i)
{
  const Eigen::Vector3f center = (start + end) / 2.0f;
  const float height = (start - end).norm();

  const Eigen::Vector3f dir = (start - end).normalized();
  Eigen::Vector3f perp = Eigen::Vector3f::UnitX();
  for (int i = 1; i < 3; i++)
    if (std::abs(dir.dot(perp)) < std::abs(dir.dot(Eigen::Vector3f::Unit(i))))
      perp = Eigen::Vector3f::Unit(i);
  const Eigen::Vector3f a = perp.cross(dir).normalized();
  const Eigen::Vector3f b = dir.cross(a);

  Eigen::Matrix3f mat;
  mat.col(0) = a;
  mat.col(1) = b;
  mat.col(2) = dir;

  Eigen::Affine3f pose;
  pose.translation() = center;
  pose.linear() = mat;

  tf::poseEigenToMsg(pose.cast<double>(),cyl.pose);
  cyl.scale.z = height;

  cyl.id = i;
}

void SurfelsUnknownSpaceNode::ShowCamera(const Eigen::Affine3f &pose,
                                         const float padding_x,
                                         const float padding_y,
                                         const float size_z)
{
  if (!m_camera_display_pub.getNumSubscribers())
    return;

  visualization_msgs::MarkerArray funnel_msg;
  auto & markers = funnel_msg.markers;

  SurfelsUnknownSpace::IntrinsicsConstPtr intrinsics = m_surfels_unknown_space->GetIntrinsics();
  if (!intrinsics)
    return;

  const int64 width = intrinsics->width;
  const int64 height = intrinsics->height;

  visualization_msgs::Marker delete_marker;
  delete_marker.header.stamp = ros::Time::now();
  delete_marker.header.frame_id = "map";
  delete_marker.action = delete_marker.DELETEALL;
  delete_marker.ns = "";
  markers.push_back(delete_marker);

  const float grid_radius = 0.02f;

  visualization_msgs::Marker cylinder_grid_marker;
  cylinder_grid_marker.header.stamp = ros::Time::now();
  cylinder_grid_marker.header.frame_id = "map";
  cylinder_grid_marker.action = cylinder_grid_marker.ADD;
  cylinder_grid_marker.ns = "";
  cylinder_grid_marker.type = cylinder_grid_marker.CYLINDER;
  cylinder_grid_marker.scale.x = grid_radius;
  cylinder_grid_marker.scale.y = grid_radius;
  cylinder_grid_marker.color = BuildColorMsg(1.0f,1.0f,0.0);

  Vector3fVector ends;

  const Eigen::Vector2i vec[4] = {Eigen::Vector2i(0, 0), Eigen::Vector2i(0, 1), Eigen::Vector2i(1, 1), Eigen::Vector2i(1, 0)};

  for (int64 i = 0; i < 4; i++)
  {
    const int x = padding_x + vec[i].x() * (width - 2 * padding_x);
    const int y = padding_y + vec[i].y() * (height - 2 * padding_y);
    const Eigen::Vector3f bearing((x - intrinsics->center_x) / intrinsics->focal_x,
                                  (y - intrinsics->center_y) / intrinsics->focal_y,
                                  1.0f);
    const Eigen::Vector3f start = bearing * 0.0f;
    const Eigen::Vector3f end = bearing * size_z;
    const Eigen::Vector3f world_start = pose * start;
    const Eigen::Vector3f world_end = pose * end;
    ends.push_back(world_end);

    CylinderFromLine(world_start, world_end, cylinder_grid_marker, markers.size());
    markers.push_back(cylinder_grid_marker);
  }

  for (uint64 i = 0; i < 4; i++)
  {
    CylinderFromLine(ends[i], ends[(i + 1) % 4], cylinder_grid_marker, markers.size());
    markers.push_back(cylinder_grid_marker);
  }

  m_camera_display_pub.publish(funnel_msg);
}

void SurfelsUnknownSpaceNode::ShowNUVG(const Eigen::Affine3f & pose,
                                    const uint64 count_at_max_range,
                                    const uint64 count_at_min_range,
                                    const uint64 count_at_zero)
{
  if (!m_nuvg_display_pub.getNumSubscribers())
    return;

  visualization_msgs::MarkerArray nuvg_msg;
  auto & markers = nuvg_msg.markers;

  SurfelsUnknownSpace::IntrinsicsConstPtr intrinsics = m_surfels_unknown_space->GetIntrinsics();
  if (!intrinsics)
    return;

  const int64 width = intrinsics->width;
  const int64 height = intrinsics->height;

  const int64 SAMPLING = 20;

  const float diameter_at_zero = m_surfels_unknown_space->getVoxelSideAtDistance(0);

  visualization_msgs::Marker delete_marker;
  delete_marker.header.stamp = ros::Time::now();
  delete_marker.header.frame_id = "map";
  delete_marker.action = delete_marker.DELETEALL;
  delete_marker.ns = "";
  markers.push_back(delete_marker);

  const float grid_radius = 0.0025f;
  const float path_radius = 0.0050f;

  visualization_msgs::Marker cylinder_grid_marker;
  cylinder_grid_marker.header.stamp = ros::Time::now();
  cylinder_grid_marker.header.frame_id = "map";
  cylinder_grid_marker.action = cylinder_grid_marker.ADD;
  cylinder_grid_marker.ns = "";
  cylinder_grid_marker.type = cylinder_grid_marker.CYLINDER;
  cylinder_grid_marker.scale.x = grid_radius;
  cylinder_grid_marker.scale.y = grid_radius;
  cylinder_grid_marker.color = BuildColorMsg(0.5,0.5,0.5);

  visualization_msgs::Marker cylinder_path_x_marker;
  cylinder_path_x_marker.header.stamp = ros::Time::now();
  cylinder_path_x_marker.header.frame_id = "map";
  cylinder_path_x_marker.action = cylinder_grid_marker.ADD;
  cylinder_path_x_marker.ns = "";
  cylinder_path_x_marker.type = cylinder_grid_marker.CYLINDER;
  cylinder_path_x_marker.scale.x = path_radius;
  cylinder_path_x_marker.scale.y = path_radius;
  cylinder_path_x_marker.color = BuildColorMsg(1,0,0);

  visualization_msgs::Marker cylinder_path_y_marker;
  cylinder_path_y_marker.header.stamp = ros::Time::now();
  cylinder_path_y_marker.header.frame_id = "map";
  cylinder_path_y_marker.action = cylinder_grid_marker.ADD;
  cylinder_path_y_marker.ns = "";
  cylinder_path_y_marker.type = cylinder_grid_marker.CYLINDER;
  cylinder_path_y_marker.scale.x = path_radius;
  cylinder_path_y_marker.scale.y = path_radius;
  cylinder_path_y_marker.color = BuildColorMsg(0,1,0);

  visualization_msgs::Marker cylinder_path_z_marker;
  cylinder_path_z_marker.header.stamp = ros::Time::now();
  cylinder_path_z_marker.header.frame_id = "map";
  cylinder_path_z_marker.action = cylinder_grid_marker.ADD;
  cylinder_path_z_marker.ns = "";
  cylinder_path_z_marker.type = cylinder_grid_marker.CYLINDER;
  cylinder_path_z_marker.scale.x = path_radius;
  cylinder_path_z_marker.scale.y = path_radius;
  cylinder_path_z_marker.color = BuildColorMsg(0,0,1);

  for (int64 i = 0; i <= width; i += SAMPLING)
    for (int64 h = 0; h <= height; h += SAMPLING)
    {
      const Eigen::Vector3f bearing((i - intrinsics->center_x) / intrinsics->focal_x,
                                    (h - intrinsics->center_y) / intrinsics->focal_y,
                                    1.0f);
      const Eigen::Vector3f start = bearing * intrinsics->min_range;
      const Eigen::Vector3f end = bearing * intrinsics->max_range;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_grid_marker, markers.size());
      markers.push_back(cylinder_grid_marker);
    }

  for (int64 i = SAMPLING / 2; i < width; i += SAMPLING)
    for (int64 h = SAMPLING / 2; h < height; h += SAMPLING)
    {
      const Eigen::Vector3f bearing((i - intrinsics->center_x) / intrinsics->focal_x,
                                    (h - intrinsics->center_y) / intrinsics->focal_y,
                                    1.0f);
      const Eigen::Vector3f start = bearing * intrinsics->min_range;
      const Eigen::Vector3f end = bearing * intrinsics->max_range;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_path_z_marker, markers.size());
      markers.push_back(cylinder_path_z_marker);
    }

  for (int64 i = 0; i <= width; i += SAMPLING)
    for (int64 h = 0; h <= height; h += SAMPLING)
    {
      const Eigen::Vector3f xy((i - intrinsics->center_x) * diameter_at_zero,
                               (h - intrinsics->center_y) * diameter_at_zero,
                               0.0f);
      const Eigen::Vector3f z0(0.0f, 0.0f, 0.0f);
      const Eigen::Vector3f z1(0.0f, 0.0f, intrinsics->min_range);
      const Eigen::Vector3f start = xy + z0;
      const Eigen::Vector3f end = xy + z1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_grid_marker, markers.size());
      markers.push_back(cylinder_grid_marker);
    }

  for (int64 i = SAMPLING / 2; i < width; i += SAMPLING)
    for (int64 h = SAMPLING / 2; h < height; h += SAMPLING)
    {
      const Eigen::Vector3f xy((i - intrinsics->center_x) * diameter_at_zero,
                               (h - intrinsics->center_y) * diameter_at_zero,
                               0.0f);
      const Eigen::Vector3f z0(0.0f, 0.0f, 0.0f);
      const Eigen::Vector3f z1(0.0f, 0.0f, intrinsics->min_range);
      const Eigen::Vector3f start = xy + z0;
      const Eigen::Vector3f end = xy + z1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_path_z_marker, markers.size());
      markers.push_back(cylinder_path_z_marker);
    }

  for (int64 z = count_at_zero; z <= int64(count_at_max_range); z += SAMPLING)
    for (int64 x = 0; x <= width; x += SAMPLING)
    {
      Eigen::Vector3f xz = (z > int64(count_at_min_range))
        ? Eigen::Vector3f((x - intrinsics->center_x) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z),
                          0.0f,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z))
        : Eigen::Vector3f((x - intrinsics->center_x) * diameter_at_zero,
                          0.0f,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z));
      Eigen::Vector3f y0 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(0.0f, (- intrinsics->center_y) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f)
          : Eigen::Vector3f(0.0f, (- intrinsics->center_y) * diameter_at_zero, 0.0f);
      Eigen::Vector3f y1 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(0.0f,(- intrinsics->center_y + height) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f)
          : Eigen::Vector3f(0.0f,(- intrinsics->center_y + height) * diameter_at_zero, 0.0f);

      const Eigen::Vector3f start = xz + y0;
      const Eigen::Vector3f end = xz + y1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_grid_marker, markers.size());
      markers.push_back(cylinder_grid_marker);
    }

  for (int64 z = count_at_zero + SAMPLING / 2; z < int64(count_at_max_range); z += SAMPLING)
    for (int64 x = SAMPLING / 2; x < width; x += SAMPLING)
    {
      Eigen::Vector3f xz = (z > int64(count_at_min_range))
        ? Eigen::Vector3f((x - intrinsics->center_x) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z),
                          0.0f,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z))
        : Eigen::Vector3f((x - intrinsics->center_x) * diameter_at_zero,
                          0.0f,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z));
      Eigen::Vector3f y0 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(0.0f, (- intrinsics->center_y) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f)
          : Eigen::Vector3f(0.0f, (- intrinsics->center_y) * diameter_at_zero, 0.0f);
      Eigen::Vector3f y1 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(0.0f,(- intrinsics->center_y + height) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f)
          : Eigen::Vector3f(0.0f,(- intrinsics->center_y + height) * diameter_at_zero, 0.0f);

      const Eigen::Vector3f start = xz + y0;
      const Eigen::Vector3f end = xz + y1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_path_y_marker, markers.size());
      markers.push_back(cylinder_path_y_marker);
    }

  for (int64 z = count_at_zero; z <= int64(count_at_max_range); z += SAMPLING)
    for (int64 y = 0; y <= height; y += SAMPLING)
    {
      Eigen::Vector3f yz = (z > int64(count_at_min_range))
        ? Eigen::Vector3f(0.0f,
                          (y - intrinsics->center_y) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z),
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z))
        : Eigen::Vector3f(0.0f,
                          (y - intrinsics->center_y) * diameter_at_zero,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z));
      Eigen::Vector3f x0 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(( - intrinsics->center_x) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f, 0.0f)
          : Eigen::Vector3f(( - intrinsics->center_x) * diameter_at_zero, 0.0f, 0.0f);
      Eigen::Vector3f x1 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(( - intrinsics->center_x + width) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f, 0.0f)
          : Eigen::Vector3f(( - intrinsics->center_x + width) * diameter_at_zero, 0.0f, 0.0f);

      const Eigen::Vector3f start = yz + x0;
      const Eigen::Vector3f end = yz + x1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_grid_marker, markers.size());
      markers.push_back(cylinder_grid_marker);
    }

  for (int64 z = count_at_zero + SAMPLING / 2; z < int64(count_at_max_range); z += SAMPLING)
    for (int64 y = SAMPLING / 2; y < height; y += SAMPLING)
    {
      Eigen::Vector3f yz = (z > int64(count_at_min_range))
        ? Eigen::Vector3f(0.0f,
                          (y - intrinsics->center_y) / intrinsics->focal_y * m_surfels_unknown_space->getDistanceFromVoxelCount(z),
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z))
        : Eigen::Vector3f(0.0f,
                          (y - intrinsics->center_y) * diameter_at_zero,
                          m_surfels_unknown_space->getDistanceFromVoxelCount(z));
      Eigen::Vector3f x0 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(( - intrinsics->center_x) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f, 0.0f)
          : Eigen::Vector3f(( - intrinsics->center_x) * diameter_at_zero, 0.0f, 0.0f);
      Eigen::Vector3f x1 = (z > int64(count_at_min_range))
          ? Eigen::Vector3f(( - intrinsics->center_x + width) / intrinsics->focal_x * m_surfels_unknown_space->getDistanceFromVoxelCount(z), 0.0f, 0.0f)
          : Eigen::Vector3f(( - intrinsics->center_x + width) * diameter_at_zero, 0.0f, 0.0f);

      const Eigen::Vector3f start = yz + x0;
      const Eigen::Vector3f end = yz + x1;
      const Eigen::Vector3f world_start = pose * start;
      const Eigen::Vector3f world_end = pose * end;

      CylinderFromLine(world_start, world_end, cylinder_path_x_marker, markers.size());
      markers.push_back(cylinder_path_x_marker);
    }

  m_nuvg_display_pub.publish(nuvg_msg);
}

void SurfelsUnknownSpaceNode::ShowKnownStateHull(const uint64 width,
                                                 const uint64 height,
                                                 const uint64 special_color_width,
                                                 const uint64 special_color_height,
                                                 ros::Publisher & pub,
                                                 const Uint32Vector & data
                                                 )
{
  if (!pub.getNumSubscribers())
    return;

  sensor_msgs::Image image;
  image.width = width;
  image.height = height;
  image.step = width * 4;
  image.is_bigendian = false;
  image.encoding = "rgba8";

  image.data.resize(image.width * image.height * 4,0);
  for (uint64 y = 0; y < height; y++)
    for (uint64 x = 0; x < width; x++)
    {
      const uint64 i = y * width + x;
      const uint64 iin = i;
      const bool v = (data[iin] % 2);

      const bool sc = (x == special_color_width) || (y == special_color_height);
      const Eigen::Vector3i color_on = sc ? Eigen::Vector3i(0,0,255) : Eigen::Vector3i(0,255,0);
      const Eigen::Vector3i color_off = sc ? Eigen::Vector3i(255,255,0) : Eigen::Vector3i(255,0,0);

      if (v)
      {
        image.data[i * 4 + 0] = color_on.x();
        image.data[i * 4 + 1] = color_on.y();
        image.data[i * 4 + 2] = color_on.z();
      }
      else
      {
        image.data[i * 4 + 0] = color_off.x();
        image.data[i * 4 + 1] = color_off.y();
        image.data[i * 4 + 2] = color_off.z();
      }

      image.data[i * 4 + 3] = 255;
    }

  pub.publish(image);
}

void SurfelsUnknownSpaceNode::ShowSurfelCloud(const SurfelsUnknownSpace::SurfelVector & surfels)
{
  if (!m_surfel_cloud_pub.getNumSubscribers())
    return;

  sensor_msgs::PointCloud2 cloud2;

  pcl::PointCloud<pcl::PointSurfel> cloud;
  const uint64 cloud_size = surfels.size();
  cloud.resize(cloud_size);

  uint64 counter = 0;
  for (uint64 i = 0; i < cloud_size; i++)
  {
    const SurfelsUnknownSpace::Surfel & fr = surfels[i];
    if (fr.erased)
      continue;

    pcl::PointSurfel & pt = cloud[counter];

    pt.x = fr.position.x();
    pt.y = fr.position.y();
    pt.z = fr.position.z();

    pt.normal_x = fr.normal.x();
    pt.normal_y = fr.normal.y();
    pt.normal_z = fr.normal.z();

    pt.radius = fr.radius;

    pt.confidence = 1.0f;
    pt.curvature = 0.0f;

    if (!fr.is_surfel)
    {
      if (m_frontel_normal_as_color)
      {
        pt.r = fr.normal.x() * 127 + 128;
        pt.g = fr.normal.y() * 127 + 128;
        pt.b = fr.normal.z() * 127 + 128;
      }
      else
      {
        pt.r = 0;
        pt.g = 0;
        pt.b = (0.3 + 0.7 * (0.5 + (-fr.normal.z()) / 2.0)) * 255;
      }
    }
    else
    {
      pt.r = fr.cr;
      pt.g = fr.cg;
      pt.b = fr.cb;
    }

    pt.a = 255;
    counter++;
  }
  cloud.resize(counter);
  cloud.width = counter;
  cloud.height = 1;

  pcl::toROSMsg(cloud,cloud2);
  cloud2.header.frame_id = "/map";
  m_surfel_cloud_pub.publish(cloud2);
}

void SurfelsUnknownSpaceNode::ShowStableImage(const uint64 width, const uint64 height,
                                              const FloatVector &depths, const FloatVector &colors,
                                              const Vector3fVector &bearings, const Eigen::Affine3f &pose)
{
  if (!m_stable_image_pub.getNumSubscribers())
    return;

  sensor_msgs::Image image;
  image.width = width;
  image.height = height;
  image.is_bigendian = false;
  image.encoding = "rgba8";

  image.data.resize(image.width * image.height * 4,0);
  for (uint64 y = 0; y < height; y++)
    for (uint64 x = 0; x < width; x++)
    {
      const uint64 i = y * width + x;

      if (depths[i])
      {
        image.data[i * 4 + 0] = colors[i * 3 + 0] * 255;
        image.data[i * 4 + 1] = colors[i * 3 + 1] * 255;
        image.data[i * 4 + 2] = colors[i * 3 + 2] * 255;
      }
      else
      {
        image.data[i * 4 + 0] = 0;
        image.data[i * 4 + 1] = 255;
        image.data[i * 4 + 2] = 0;
      }

      image.data[i * 4 + 3] = 255;
    }

  m_stable_image_pub.publish(image);
}
