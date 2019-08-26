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

#ifndef TIMER_STUB_H
#define TIMER_STUB_H

#ifdef RMONICA_MULTI_TIMER_FOUND
#include "timer_def.h"
#else

#define TIMERDEF_SUBSAMPLE        ""
#define TIMERDEF_INIT             ""
#define TIMERDEF_BEARINGS         ""
#define TIMERDEF_SPLITTING        ""
#define TIMERDEF_OBSERVED_FIELD   ""
#define TIMERDEF_DOT_FIELD        ""
#define TIMERDEF_DOT_FIELD_INIT   ""
#define TIMERDEF_DOT_FIELD_UPLOAD(n)   ""
#define TIMERDEF_DOT_FIELD_PROJECT(n)  ""
#define TIMERDEF_DOT_FIELD_INTERNAL(n) ""
#define TIMERDEF_DOT_FIELD_HULL(n)     ""
#define TIMERDEF_DOT_FIELD_RECOLOR(n)  ""
#define TIMERDEF_DOT_FIELD_DELETE(n)   ""
#define TIMERDEF_DOT_HULL         ""
#define TIMERDEF_KNOWN_SPACE      ""
#define TIMERDEF_KNOWN_SPACE_FILTERING ""
#define TIMERDEF_CREATION         ""

#define TIMER_INIT
#define TIMER_START(x)
#define TIMER_STOP(x)
#define TIMER_NEWFRAME
#define TIMER_GETSTRING ""

#endif // RMONICA_MULTI_TIMER_FOUND

#endif // TIMER_STUB_H
