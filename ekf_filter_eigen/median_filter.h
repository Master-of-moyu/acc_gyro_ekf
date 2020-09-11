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

#include <deque>
#include "../types.h"

namespace sensors {

// Fixed window FIFO median filter for vectors of the given dimension = 3.
class MedianFilter {
  public:
    // Creates a median filter of size filter_size.
    // @param filter_size size of the internal filter.
    explicit MedianFilter(size_t filter_size);

    // Adds sample to buffer_ if buffer_ is full it drops the oldest sample.
    void AddSample(const vector<3>& sample);

    // Returns true if buffer has filter_size_ sample, false otherwise.
    bool IsValid() const;

    // Returns the median of values store in the internal buffer.
    vector<3> GetFilteredData() const;

    // Resets the filter, removing all samples that have been added.
    void Reset();

  private:
    const size_t filter_size_;
    std::deque<vector<3>> buffer_;
    // Contains norms of the elements stored in buffer_.
    std::deque<double> norms_;
};

} // namespace sensors
