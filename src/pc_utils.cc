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
#include "pc_utils.h"

#include <string>
#include <vector>

#include <pcl/io/ply_io.h>

MYNTEYE_BEGIN_NAMESPACE

namespace pcutil {

inline
CameraIntrinsics get_camera_intrinsics(const Camera &camera) {
  auto stream_mode = camera.GetOpenParams().stream_mode;
  return camera.GetStreamIntrinsics(stream_mode).left;
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor,
    std::vector<std::shared_ptr<BaseFilter>> filters) {
  static auto cam_in = get_camera_intrinsics(*camera);

  cv::Mat color;
  auto image_color = camera->GetStreamData(ImageType::IMAGE_LEFT_COLOR);
  if (image_color.img) {
    color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
  }

  cv::Mat depth;
  auto image_depth = camera->GetStreamData(ImageType::IMAGE_DEPTH);
  if (image_depth.img) {
    depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
  }

  if (color.empty() || depth.empty()) { return nullptr; }
  for (size_t i=0; i< filters.size(); i++) {
    filters[i]->ProcessFrame(image_depth.img, image_depth.img);
  }

  return get_point_cloud(color, depth, cam_in, cam_factor);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    Camera *camera, float cam_factor) {
  static auto cam_in = get_camera_intrinsics(*camera);

  cv::Mat color;
  auto image_color = camera->GetStreamData(ImageType::IMAGE_LEFT_COLOR);
  if (image_color.img) {
    color = image_color.img->To(ImageFormat::COLOR_BGR)->ToMat();
  }

  cv::Mat depth;
  auto image_depth = camera->GetStreamData(ImageType::IMAGE_DEPTH);
  if (image_depth.img) {
    depth = image_depth.img->To(ImageFormat::DEPTH_RAW)->ToMat();
  }

  if (color.empty() || depth.empty()) { return nullptr; }

  return get_point_cloud(color, depth, cam_in, cam_factor);
}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr get_point_cloud(
    const cv::Mat &rgb, const cv::Mat& depth,
    const CameraIntrinsics& cam_in, float cam_factor) {
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  for (int m = 0; m < depth.rows; m++) {
    for (int n = 0; n < depth.cols; n++) {
      std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
      if (d == 0) continue;
      pcl::PointXYZRGBA p;
      p.x = static_cast<float>(d) / cam_factor;
      p.y = (n - cam_in.cx) * p.x / cam_in.fx;
      p.z = (m - cam_in.cy) * p.x / cam_in.fy;
      p.b = rgb.ptr<uchar>(m)[n * 3];
      p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
      p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
      cloud->points.push_back(p);
    }
  }
  return cloud;
}

}  // namespace util

MYNTEYE_END_NAMESPACE
