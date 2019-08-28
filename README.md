Surfel-based Reconstruction of the Known Volume
===============================================

This repository contains a C++/OpenCL implementation of a surfel-based method to incrementally reconstruct known (empty) volume observed by a moving 3D camera.
As the 3D camera observes the environment, two kinds of measurements are generated: occupied measurements, when a view ray hits a solid surface, and empty (miss) measurements, when the view ray traverses a region of space to observe a surface beyond it.

The goal of this method is the incremental reconstruction of the empty volume, i.e., of the volume traversed by the view rays, using a surfel-based representation. In particular, surfels are assembled to represent a closed surface, bounding the empty volume. The surface separates the empty volume from unknown or occupied space.

For surfels separating empty from occupied space, the implementation also tracks the color. The others are classified as frontels (frontier elements), as they represent a frontier of known space, that can be explored.

Related publications
--------------------

To appear.

Dependencies
------------

- ROS (Robot Operating System) Kinetic or Melodic
- OpenCL C++ headers, version 2.0
- OpenCL runtime, minimum version 1.1
- PCL (Point Cloud Library)
- Eigen3

The package `rmonica_multi_timer` was used for profiling, but it is currently unpublished. The dependency is optional.

Usage
-----

The implementation of the method is contained in the `surfels_unknown_space` node, in the `surfels_unknown_space` package.
The node starts with an empty surfel cloud, representing completely unknown space, and it waits for sensor observations.

Sensor observations should be provided to the topic `/elastic_frame_state_stable` using the message type `surfels_unknown_space_msgs/FrameState`.
The message type is a copy of `elastic_bridge/FrameState`, so that it is compatible with [elastic_bridge](https://github.com/RMonica/elastic_bridge), even if not all fields are used.
In particular, these fields are required:

- width: the image width
- height: the image height
- focal_x: focal length (horizontal)
- focal_y: focal length (vertical)
- pose: sensor pose from an external source (the node does not perform egomotion tracking)
- input_color: color image, 3 bytes per pixel
- input_depth: depth image, in mm

When the node receives a message from topic `/elastic_frame_state_stable`, it updates the surfel-based representation to include the new data. At the end of the update, the node sends a `std_msgs/Empty` acknowledgment message to the `ack` topic, and it becomes ready for a new message.

The current surfel cloud may be obtained at any time by calling action `/elastic_ui_download`, of type `surfels_unknown_space_msgs/GetSurfelCloud`. The surfel cloud is also published at each iteration to topic `surfels_cloud` as `sensor_msgs/PointCloud2`. The current surfel cloud can also be loaded from an external source by calling action `/elastic_ui_upload` of type `surfels_unknown_space_msgs/SetSurfelCloud`.

The surfel cloud uses the standard PCL `PointSurfel` point type (compatible with [surfel_cloud_rviz_plugin](https://github.com/RMonica/surfel_cloud_rviz_plugin)). Frontels can be distinguished from occupied surfels as they have `confidence` field set to 0.

Test
----

The package `surfels_unknown_space_state_image_reader` is provided as an example to test the `surfels_unknown_space` node. Further information is provided in the README of that package.

Parameters
----------

General configuration:

- `input_frame_state_topic` (string): input topic for sensor observations (default `/elastic_frame_state_stable`)
- `ack_topic` (string): output topic for the acknowledgment (default: `ack`)
- `frontel_normal_as_color` (bool): if true, the frontel color is computed from their normal. Otherwise, frontels have an uniform blue color. (default: false)
- `max_range` (double): maximum sensor range (default: 5 meters)
- `min_range` (double): minimum sensor range (default: 0.5 meters)
- `downsampling_factor` (int): input image downsampling factor (default: 4)
- `surfel_download_action` (string): action name for surfel download (default: `/elastic_ui_download`)
- `surfel_upload_action` (string): action name for surfel upload (default: `/elastic_ui_upload`)
- `get_timers_action` (string): currently not available (requires `rmonica_multi_timer`).

OpenCL configuration:

- `opencl_platform_name` (string): part of the platform name for OpenCL, first matching platform will be used (default: empty)
- `opencl_device_name` (string): part of the device name for OpenCL, first matching device will be used (default: empty)
- `opencl_use_intel` (bool): activates a workaround to prevent a crash in the Intel compiler
- `opencl_device_type` (string): restrict OpenCL to a specific device type, it can be `GPU`, `CPU` or `ALL` (default: `ALL`)
- `opencl_subdevice_size` (int): if non-zero, only `opencl_subdevice_size` cores of the device are used (requires device fission support) (default: 0)
- `max_surfels_in_gpu_memory` (int): number of surfels which are loaded simultaneously in OpenCL memory, reduce to decrease OpenCL memory usage at the expense of performance (default: 1 000 000)
- `projection_threads` (int): number of simultaneous thread in the projection phase, tweak to improve performance (default: 8192)

Internal parameters configuration:

- `confidence_threshold`: minimum angle path/surfel (default: 0.173)
- `enable_known_space_filter`: enable known space median filter (default: true)
- `side_padding`: padding of the input image (default: 5)
- `back_padding`: padding behind the sensor (default: 2)
- `hull_unknown_surfel_mult`: unknown surfels are multiplied by this when computing the known state hull (default: 1.5)
- `surfel_thickness`: surfel ellipsoid thickness, in voxels (default: 1.732)
- `surfel_radius_creation_mult`: surfel radius is multiplied by this (default: 1.2)

2019-08-26
