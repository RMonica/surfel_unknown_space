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

#ifndef STATE_IMAGE_READER_H
#define STATE_IMAGE_READER_H

#define PARAM_NAME_INPUT_FILE_PREFIX         "input_file_prefix"
#define PARAM_DEFAULT_INPUT_FILE_PREFIX      ""

#define PARAM_NAME_ACK_TOPIC                 "ack_topic"
#define PARAM_DEFAULT_ACK_TOPIC              "ack"

#define PARAM_NAME_FRAME_STATE_TOPIC         "frame_state_topic"
#define PARAM_DEFAULT_FRAME_STATE_TOPIC      "frame_state"

#define PARAM_NAME_WAIT_INITIAL              "wait_initial"
#define PARAM_DEFAULT_WAIT_INITIAL           float(2.0) // seconds

#define PARAM_NAME_SAVE_CLOUD_NAME           "save_cloud_name"
#define PARAM_DEFAULT_SAVE_CLOUD_NAME        "surfels"

#define PARAM_NAME_TF_REFERENCE_FRAME        "tf_reference_frame"
#define PARAM_DEFAULT_TF_REFERENCE_FRAME     "/map"

#define PARAM_NAME_TF_CAMERA_FRAME           "tf_camera_frame"
#define PARAM_DEFAULT_TF_CAMERA_FRAME        "/camera_frame"

#endif // STATE_IMAGE_READER_H
