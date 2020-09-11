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
#include "pose_prediction.h"

namespace sensors {

namespace {

    constexpr double kEpsilon = 1e-15;

}

quaternion GetRotationFromGyroscope(const vector<3>& gyroscope_value,
                                    double timestep_s) {
    const double velocity = gyroscope_value.norm();

    // When there is no rotation data return an identity rotation.
    if (velocity < kEpsilon) {
        return quaternion::Identity();
    }
    // Since the gyroscope_value is a start from sensor transformation we need to
    // invert it to have a sensor from start transformation, hence the minus sign.
    // For more info:
    // http://developer.android.com/guide/topics/sensors/sensors_motion.html#sensors-motion-gyro
    return quaternion(Eigen::AngleAxisd(-timestep_s * velocity, gyroscope_value / velocity)).normalized();
}

quaternion PredictPose(int64_t requested_pose_timestamp,
                       const PoseState& current_state) {
    // Subtracting unsigned numbers is bad when the result is negative.
    const int64_t diff = requested_pose_timestamp - current_state.timestamp;
    const double timestep_s = static_cast<double>(diff) * 1e-9;

    const quaternion update = GetRotationFromGyroscope(
        current_state.sensor_from_start_rotation_velocity, timestep_s);
    return update * current_state.sensor_from_start_rotation;
}

quaternion PredictPoseInv(int64_t requested_pose_timestamp,
                          const PoseState& current_state) {
    // Subtracting unsigned numbers is bad when the result is negative.
    const int64_t diff = requested_pose_timestamp - current_state.timestamp;
    const double timestep_s = static_cast<double>(diff) * 1e-9;

    const quaternion update = GetRotationFromGyroscope(
        current_state.sensor_from_start_rotation_velocity, timestep_s);
    return current_state.sensor_from_start_rotation * update.conjugate();
}

} // namespace sensors
