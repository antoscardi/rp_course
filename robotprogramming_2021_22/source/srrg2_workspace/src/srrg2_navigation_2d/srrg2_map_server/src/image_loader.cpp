/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * This file contains helper functions for loading images as maps.
 *
 * Author: Brian Gerkey
 * 2nd author: Giorgio Grisetti
 */

#include "map_server/image_loader.h"
#include <Eigen/Geometry>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <stdio.h>
#include <stdlib.h>

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

// IASTIMIA: gg: non ho mai visto tanta merda cosi' concentrata in un unico file

namespace map_server {

  template <typename PixelType>
  void getAvgAlpha(uint8_t& avg, uint8_t& alpha, const PixelType& p) {
    float accumulator = 0;
    static constexpr int avg_channels =
      PixelType::channels < 3 ? PixelType::channels : PixelType::channels - 1;
    static constexpr int alpha_channel = PixelType::channels > 3 ? 3 : -1;
    alpha                              = alpha_channel > -1 ? p(alpha_channel) : 0;

    for (int i = 0; i < avg_channels; ++i) {
      accumulator += (float) p(i);
    }
    avg = (uint8_t)(accumulator / avg_channels);
  }

  template <int num_channels>
  bool fillMapFromImage(nav_msgs::GetMap::Response& dest,
                        const cv::Mat src,
                        float occ_th,
                        float free_th,
                        bool negate  = false,
                        MapMode mode = TRINARY) {
    if (src.type() != CV_8UC1 && src.type() != CV_8UC3 && src.type() != CV_8UC4) {
      std::cerr << "cannot handle image of type [:" << src.type() << "]" << std::endl;
      std::cerr << "known types are :" << CV_8UC1 << " " << CV_8UC3 << " " << CV_8UC4 << std::endl;
      return false;
    }
    if (num_channels != src.channels()) {
      std::cerr << "cannot handle image with [:" << src.channels() << "] channels" << std::endl;
    }
    dest.map.info.width  = src.cols;
    dest.map.info.height = src.rows;

    using PixelType = cv::Vec<uint8_t, num_channels>;
    dest.map.data.resize(src.rows * src.cols);
    for (int r = 0; r < src.rows; ++r) {
      int8_t* dptr = &dest.map.data[src.cols * (src.rows - r - 1)]; // this is sick
      for (int c = 0; c < src.cols; ++c, ++dptr) {
        uint8_t avg, alpha;
        getAvgAlpha(avg, alpha, src.at<PixelType>(r, c));
        int8_t out = -1; // default output occupancy (unknown)
        // if alpha channel >0 process the occupancy
        if (alpha > 0) {
          if (!negate) {
            avg = 255 - avg;
          }
          float avg_f = (float) avg / (float) 255.f;
          float occ_f = std::max(0.f, (float) (avg_f - free_th) / (float) (occ_th - free_th));
          occ_f       = std::min(1.f, occ_f);
          switch (mode) {
            case RAW:
              out = avg;
              break;
            case SCALE:
              out = occ_f * 100;
              break;
            case TRINARY:
              if (occ_f > occ_th)
                out = 100;
              else if (occ_f <= free_th)
                out = 0;
              break;
          }
        }
        *dptr = out;
      }
    }
    return true;
  }

#if CV_VERSION_MAJOR < 4
#define IMAGE_READ_MODE CV_LOAD_IMAGE_UNCHANGED
#else
#define IMAGE_READ_MODE cv::IMREAD_UNCHANGED
#endif

  void loadMapFromFile(nav_msgs::GetMap::Response* resp,
                       const char* fname,
                       double res,
                       bool negate,
                       double occ_th,
                       double free_th,
                       double* origin,
                       MapMode mode) {
    cv::Mat img = cv::imread(fname, IMAGE_READ_MODE);

    bool result = false;
    switch (img.channels()) {
      case 1:
        result = fillMapFromImage<1>(*resp, img, occ_th, free_th, negate, mode);
        break;
      case 3:
        result = fillMapFromImage<3>(*resp, img, occ_th, free_th, negate, mode);
        break;
      case 4:
        result = fillMapFromImage<4>(*resp, img, occ_th, free_th, negate, mode);
        break;
      default:
        std::cerr << "cannot handle image of type [:" << img.type() << "]" << std::endl;
    }
    if (!result) {
      std::cerr << "issue in converting map" << std::endl;
      return;
    }
    // Copy the image data into the map structure
    resp->map.info.resolution        = res;
    resp->map.info.origin.position.x = origin[0];
    resp->map.info.origin.position.y = origin[1];
    resp->map.info.origin.position.z = 0.0;
    Eigen::AngleAxisf rot(origin[2], Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf q(rot.toRotationMatrix());
    resp->map.info.origin.orientation.w = q.w();
    resp->map.info.origin.orientation.x = q.x();
    resp->map.info.origin.orientation.y = q.y();
    resp->map.info.origin.orientation.z = q.z();
  }
} // namespace map_server
