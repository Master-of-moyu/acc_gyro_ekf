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
#include "median_filter.h"

namespace sensors {

MedianFilter::MedianFilter(size_t filter_size) :
    filter_size_(filter_size) {
}

void MedianFilter::AddSample(const vector<3>& sample) {
    buffer_.push_back(sample);
    norms_.push_back(sample.norm());
    if (buffer_.size() > filter_size_) {
        buffer_.pop_front();
        norms_.pop_front();
    }
}

bool MedianFilter::IsValid() const {
    return buffer_.size() == filter_size_;
}

vector<3> MedianFilter::GetFilteredData() const {
    std::vector<double> norms(norms_.begin(), norms_.end());

    // Get median of value of the norms.
    std::nth_element(norms.begin(), norms.begin() + filter_size_ / 2,
                     norms.end());
    const double median_norm = norms[filter_size_ / 2];

    // Get median value based on their norm.
    auto median_it = buffer_.begin();
    for (const auto norm : norms_) {
        if (norm == median_norm) {
            break;
        }
        ++median_it;
    }

    return *median_it;
}

void MedianFilter::Reset() {
    buffer_.clear();
    norms_.clear();
}

} // namespace sensors
