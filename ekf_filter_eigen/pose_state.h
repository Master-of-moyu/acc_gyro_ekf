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

enum {
    kPoseStateFlagInvalid = 1U << 0,
    kPoseStateFlagInitializing = 1U << 1,
    kPoseStateFlagHas6DoF = 1U << 2,
};

// Stores a head pose pose plus derivatives. This can be used for prediction.
struct PoseState {
    // System wall time.
    int64_t timestamp;

    // Rotation from Sensor Space to Start Space.
    quaternion sensor_from_start_rotation;

    // First derivative of the rotation.
    vector<3> sensor_from_start_rotation_velocity;

    // Current gyroscope bias in rad/s.
    vector<3> bias;

    // The position of the headset.
    vector<3> position = vector<3>::Zero();

    // In the same coordinate frame as the position.
    vector<3> velocity = vector<3>::Zero();

    // Flags indicating the status of the pose.
    uint64_t flags = 0;
};

} // namespace sensors
