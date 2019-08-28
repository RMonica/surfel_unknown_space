Surfels Unknown Space State Image Reader
========================================

This package is an example to test the `surfels_unknown_space` node. The `state_image_reader` node, contained in this package, feeds sensor frames to `surfels_unknown_space` in sequence. The node waits for an `ack` message after each frame, so that sensor frames are not skipped.

A sample launch file is provided in the `launch` folder. The launch file expects a dataset in the `data` folder. For each frame, numbered *`n`* starting from 0, the following files must be present in `data`:

- *`n`*`_intrinsics.txt`: a text file containing the sensor intrinsic parameters, in the format:

```
  sequence_number
  width height
  
  focal_x focal_y
  center_x center_y
```
**Note**: the `surfels_unknown_space` node requires that all frames have the same intrinsic parameters.

- *`n`*`_pose.matrix`: a text file containing the sensor pose (z forward, y down), a roto-translation matrix in the format:
```
  rxx rxy rxz tx
  ryx ryy rzy ty
  rzx rzy rzz tz
  0   0   0   1
```

- *`n`*`_cloud.pcd`: the sensor data, an organized `width x height` PCL point cloud with (at least) these custom fields for each point:
```
  uint32 input_rgba    RGBA color
  uint16 input_depth   depth in mm
```
**Note**: zero depth represents an invalid measurement.

When all frames have been processed, the `state_image_reader` saves the current surfel cloud to *`n`*`_surfels.pcd`.

A compatible dataset can be downloaded from <http://rimlab.ce.unipr.it/RMonica.html>, under "surfels_unknown_space_dataset".