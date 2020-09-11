/*
 * Copyright 2019 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include "../types.h"

namespace sensors {

struct AccelerometerData {
    // System wall time.
    uint64_t system_timestamp;

    // Sensor clock time in nanoseconds.
    uint64_t sensor_timestamp_ns;

    // Acceleration force along the x,y,z axes in m/s^2. This follows android
    // specification
    // (https://developer.android.com/guide/topics/sensors/sensors_overview.html#sensors-coords).
    vector<3> data;
};

struct GyroscopeData {
    // System wall time.
    uint64_t system_timestamp;

    // Sensor clock time in nanoseconds.
    uint64_t sensor_timestamp_ns;

    // Rate of rotation around the x,y,z axes in rad/s. This follows android
    // specification
    // (https://developer.android.com/guide/topics/sensors/sensors_overview.html#sensors-coords).
    vector<3> data;
};

} // namespace sensors
