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

#ifndef SURFELS_UNKNOWN_SPACE_NODE_H
#define SURFELS_UNKNOWN_SPACE_NODE_H

// -- general --

#define PARAM_NAME_INPUT_TOPIC                 "input_frame_state_topic"
#define PARAM_DEFAULT_INPUT_TOPIC              "/elastic_frame_state_stable" // FrameState

#define PARAM_NAME_ACK_TOPIC                   "ack_topic"
#define PARAM_DEFAULT_ACK_TOPIC                "ack"

#define PARAM_NAME_FRONTEL_NORMAL_AS_COLOR     "frontel_normal_as_color"
#define PARAM_DEFAULT_FRONTEL_NORMAL_AS_COLOR  (false) // if true, frontels are in false colors, computed from the normal
                                                       // if false, all frontels are in blue

#define PARAM_NAME_MAX_RANGE                   "max_range"
#define PARAM_DEFAULT_MAX_RANGE                (5.0) // sensor max range, meters

#define PARAM_NAME_MIN_RANGE                   "min_range"
#define PARAM_DEFAULT_MIN_RANGE                (0.5) // sensor min range, meters

#define PARAM_NAME_DOWNSAMPLE_FACTOR           "downsampling_factor"
#define PARAM_DEFAULT_DOWNSAMPLE_FACTOR        (int(4))

#define PARAM_NAME_SURFEL_DOWNLOAD_ACTION      "surfel_download_action"
#define PARAM_DEFAULT_SURFEL_DOWNLOAD_ACTION   "/elastic_ui_download"

#define PARAM_NAME_SURFEL_UPLOAD_ACTION        "surfel_upload_action"
#define PARAM_DEFAULT_SURFEL_UPLOAD_ACTION     "/elastic_ui_upload"

#define PARAM_NAME_GET_TIMERS_ACTION           "get_timers_action"
#define PARAM_DEFAULT_GET_TIMERS_ACTION        "/elastic_ui_get_timers"

// -- OpenCL configuration --

#define PARAM_NAME_OPENCL_PLATFORM_NAME        "opencl_platform_name"
#define PARAM_DEFAULT_OPENCL_PLATFORM_NAME     ""           // a part of the name is enough, empty = default

#define PARAM_NAME_OPENCL_DEVICE_NAME          "opencl_device_name"
#define PARAM_DEFAULT_OPENCL_DEVICE_NAME       ""           // a part of the name is enough, empty = default

#define PARAM_NAME_OPENCL_USE_INTEL            "opencl_use_intel"
#define PARAM_DEFAULT_OPENCL_USE_INTEL         (bool(false))  // if true, activates a workaround for the Intel OpenCL compiler

#define PARAM_NAME_OPENCL_DEVICE_TYPE          "opencl_device_type"
#define PARAM_VALUE_OPENCL_DEVICE_TYPE_GPU     "GPU"
#define PARAM_VALUE_OPENCL_DEVICE_TYPE_CPU     "CPU"
#define PARAM_VALUE_OPENCL_DEVICE_TYPE_ALL     "ALL"
#define PARAM_DEFAULT_OPENCL_DEVICE_TYPE       PARAM_VALUE_OPENCL_DEVICE_TYPE_ALL

#define PARAM_NAME_OPENCL_SUBDEVICE_SIZE       "opencl_subdevice_size"
#define PARAM_DEFAULT_OPENCL_SUBDEVICE_SIZE    (int(0)) // the number of cores to be used, zero = all
                                                        // uses device fission

#define PARAM_NAME_MAX_SURFELS_IN_MEM          "max_surfels_in_gpu_memory"
#define PARAM_DEFAULT_MAX_SURFELS_IN_MEM       (int(1000 * 1000)) // decrease to trade speed for GPU ram

#define PARAM_NAME_PROJECTION_THREADS          "projection_threads"
#define PARAM_DEFAULT_PROJECTION_THREADS       (int(1024 * 8)) // number of OpenCL threads

// -- internal configuration --

#define PARAM_NAME_CONFIDENCE_THRESHOLD        "confidence_threshold"
#define PARAM_DEFAULT_CONFIDENCE_THRESHOLD     (0.173)

#define PARAM_NAME_ENABLE_KNOWN_SPACE_FILTER  "enable_known_space_filter"
#define PARAM_DEFAULT_ENABLE_KNOWN_SPACE_FILTER (true) // enable median filter in known space

#define PARAM_NAME_SIDE_PADDING                "side_padding"
#define PARAM_DEFAULT_SIDE_PADDING             (int(5))

#define PARAM_NAME_BACK_PADDING                "back_padding"
#define PARAM_DEFAULT_BACK_PADDING             (int(2))

#define PARAM_NAME_HULL_UNKNOWN_SURFEL_MULT    "hull_unknown_surfel_mult"
#define PARAM_DEFAULT_HULL_UNKNOWN_SURFEL_MULT (1.5) // radius multiplier for unknown surfels when computing known space hull

#define PARAM_NAME_SURFEL_THICKNESS           "surfel_thickness"
#define PARAM_DEFAULT_SURFEL_THICKNESS        (std::sqrt(3.0))

#define PARAM_NAME_SURFEL_RADIUS_CREAT_MULT   "surfel_radius_creation_mult"
#define PARAM_DEFAULT_SURFEL_RADIUS_CREAT_MULT (1.2) // multiplier for surfel radius at creation

#endif // SURFELS_UNKNOWN_SPACE_H
