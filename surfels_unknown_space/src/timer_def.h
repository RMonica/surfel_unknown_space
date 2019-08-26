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

#ifndef TIMER_DEF_H
#define TIMER_DEF_H

#include <rmonica_multi_timer/multi_timer_table.h>

#define TIMERDEF_SUBSAMPLE        "Subsample"
#define TIMERDEF_INIT             "Init"
#define TIMERDEF_BEARINGS         "Bearings"
#define TIMERDEF_SPLITTING        "Splitting"
#define TIMERDEF_OBSERVED_FIELD   "ObservedField"
#define TIMERDEF_DOT_FIELD        "DotField"
#define TIMERDEF_DOT_FIELD_INIT   "DotFieldInit"
#define TIMERDEF_DOT_FIELD_UPLOAD(n)   (std::string("DotFieldUpload") + std::to_string(n))
#define TIMERDEF_DOT_FIELD_PROJECT(n)  (std::string("DotFieldProject") + std::to_string(n))
#define TIMERDEF_DOT_FIELD_INTERNAL(n) (std::string("DotFieldInternal") + std::to_string(n))
#define TIMERDEF_DOT_FIELD_HULL(n)     (std::string("DotFieldHull") + std::to_string(n))
#define TIMERDEF_DOT_FIELD_RECOLOR(n)  (std::string("DotFieldRecolor") + std::to_string(n))
#define TIMERDEF_DOT_FIELD_DELETE(n)   (std::string("DotFieldDelete") + std::to_string(n))
#define TIMERDEF_DOT_HULL         "FilterDotHull"
#define TIMERDEF_KNOWN_SPACE      "KnownSpace"
#define TIMERDEF_KNOWN_SPACE_FILTERING "KnownSpaceFiltering"
#define TIMERDEF_CREATION         "Creation"

class TimerSingleton
{
public:
  static MultiTimerTable & Get()
  {
    static MultiTimerTable timers;
    return timers;
  }
};

#define TIMER_INIT (TimerSingleton::Get().SetNames({TIMERDEF_SUBSAMPLE, TIMERDEF_INIT, TIMERDEF_BEARINGS, \
                                                    TIMERDEF_SPLITTING, TIMERDEF_OBSERVED_FIELD, TIMERDEF_DOT_FIELD, \
                                                    TIMERDEF_DOT_FIELD_INIT, \
  TIMERDEF_DOT_FIELD_UPLOAD(0), TIMERDEF_DOT_FIELD_PROJECT(0), \
  TIMERDEF_DOT_FIELD_INTERNAL(0), TIMERDEF_DOT_FIELD_HULL(0), \
  TIMERDEF_DOT_FIELD_RECOLOR(0), TIMERDEF_DOT_FIELD_DELETE(0), \
  TIMERDEF_DOT_FIELD_UPLOAD(1), TIMERDEF_DOT_FIELD_PROJECT(1), \
  TIMERDEF_DOT_FIELD_INTERNAL(1), TIMERDEF_DOT_FIELD_HULL(1), \
  TIMERDEF_DOT_FIELD_RECOLOR(1), TIMERDEF_DOT_FIELD_DELETE(1), \
  TIMERDEF_DOT_FIELD_UPLOAD(2), TIMERDEF_DOT_FIELD_PROJECT(2), \
  TIMERDEF_DOT_FIELD_INTERNAL(2), TIMERDEF_DOT_FIELD_HULL(2), \
  TIMERDEF_DOT_FIELD_RECOLOR(2), TIMERDEF_DOT_FIELD_DELETE(2), \
  TIMERDEF_DOT_FIELD_UPLOAD(3), TIMERDEF_DOT_FIELD_PROJECT(3), \
  TIMERDEF_DOT_FIELD_INTERNAL(3), TIMERDEF_DOT_FIELD_HULL(3), \
  TIMERDEF_DOT_FIELD_RECOLOR(3), TIMERDEF_DOT_FIELD_DELETE(3), \
  TIMERDEF_DOT_FIELD_UPLOAD(4), TIMERDEF_DOT_FIELD_PROJECT(4), \
  TIMERDEF_DOT_FIELD_INTERNAL(4), TIMERDEF_DOT_FIELD_HULL(4), \
  TIMERDEF_DOT_FIELD_RECOLOR(4), TIMERDEF_DOT_FIELD_DELETE(4), \
  TIMERDEF_DOT_FIELD_UPLOAD(5), TIMERDEF_DOT_FIELD_PROJECT(5), \
  TIMERDEF_DOT_FIELD_INTERNAL(5), TIMERDEF_DOT_FIELD_HULL(5), \
  TIMERDEF_DOT_FIELD_RECOLOR(5), TIMERDEF_DOT_FIELD_DELETE(5), \
                                                    TIMERDEF_DOT_HULL, TIMERDEF_KNOWN_SPACE, TIMERDEF_KNOWN_SPACE_FILTERING, \
                                                    TIMERDEF_CREATION}));
#define TIMER_START(x) (TimerSingleton::Get().Tic((x)));
#define TIMER_STOP(x) (TimerSingleton::Get().Toc((x)));
#define TIMER_NEWFRAME (TimerSingleton::Get().NewFrame());
#define TIMER_GETSTRING (TimerSingleton::Get().ToString())

#endif // TIMER_DEF_H
