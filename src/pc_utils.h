// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef MYNTEYE_UTIL_POINTS_H_
#define MYNTEYE_UTIL_POINTS_H_
#pragma once

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mynteyed/filter/base_filter.h"

#include "mynteyed/stubs/global.h"
#include "mynteyed/camera.h"

MYNTEYE_BEGIN_NAMESPACE

namespace pcutil {
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor,
    std::vector<std::shared_ptr<BaseFilter>> filters);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor = 1000);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    const cv::Mat &rgb, const cv::Mat& depth,
    const CameraIntrinsics& cam_in, float cam_factor);

}  // namespace util

MYNTEYE_END_NAMESPACE

#endif  // MYNTEYE_UTIL_POINTS_H_
