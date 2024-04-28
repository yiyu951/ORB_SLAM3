/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>

#include <opencv2/core/core.hpp>

#include "depthai/depthai.hpp"
#include <System.h>
#include <csignal>
#include <memory>

#include <fmt/color.h>
#include <fmt/format.h>

using namespace std;
bool mainLoopFlag = true;
void signal_handler(int signum) {

  mainLoopFlag = false; // 设置标志表示收到了信号
  fmt::print("Exit Program\n");
}

std::shared_ptr<dai::Device> createDevice() {
  dai::Pipeline pipeline;

  // Define sources and outputs
  auto monoRight = pipeline.create<dai::node::MonoCamera>();
  auto monoLeft = pipeline.create<dai::node::MonoCamera>();
  auto xoutRight = pipeline.create<dai::node::XLinkOut>();
  auto xoutLeft = pipeline.create<dai::node::XLinkOut>();

  auto imu = pipeline.create<dai::node::IMU>();
  auto imuLinkOut = pipeline.create<dai::node::XLinkOut>();

  xoutRight->setStreamName("right");
  xoutLeft->setStreamName("left");
  imuLinkOut->setStreamName("imu");

  monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

  monoLeft->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoRight->setResolution(
      dai::MonoCameraProperties::SensorResolution::THE_720_P);

  // enable ACCELEROMETER_RAW at 500 hz rate
  imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 500);
  // enable GYROSCOPE_RAW at 400 hz rate
  imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 400);
  // it's recommended to set both setBatchReportThreshold and setMaxBatchReports
  // to 20 when integrating in a pipeline with a lot of input/output connections
  // above this threshold packets will be sent in batch of X, if the host is not
  // blocked and USB bandwidth is available
  imu->setBatchReportThreshold(1);
  // maximum number of IMU packets in a batch, if it's reached device will block
  // sending until host can receive it if lower or equal to batchReportThreshold
  // then the sending is always blocking on device useful to reduce device's CPU
  // load  and number of lost packets, if CPU load is high on device side due to
  // multiple nodes
  imu->setMaxBatchReports(10);

  // Linking
  monoLeft->out.link(xoutLeft->input);
  monoRight->out.link(xoutRight->input);
  imu->out.link(imuLinkOut->input);

  return std::make_shared<dai::Device>(pipeline);
}

void ImageThread(std::shared_ptr<dai::Device> device) {}

void ImuThread(std::shared_ptr<dai::Device> device,
               std::vector<ORB_SLAM3::IMU::Point> &vImuMeans,
               std::mutex &mutex) {
  auto imuQueue = device->getOutputQueue("imu", 50, false);

  while (mainLoopFlag) {
    auto imuData = imuQueue->get<dai::IMUData>();

    auto imuPackets = imuData->packets;
    for (auto &imuPacket : imuPackets) {

      auto &acceleroValues = imuPacket.acceleroMeter;
      auto &gyroValues = imuPacket.gyroscope;

      auto acceleroTs1 = acceleroValues.getTimestamp().time_since_epoch();
      double timestamp =
          chrono::duration_cast<chrono::duration<double>>(acceleroTs1).count();
      // auto gyroTs1 = gyroValues.getTimestampDevice();
      {
        std::lock_guard<std::mutex> l{mutex};
        vImuMeans.emplace_back(ORB_SLAM3::IMU::Point{
            acceleroValues.x, acceleroValues.y, acceleroValues.z, gyroValues.x,
            gyroValues.y, gyroValues.z, timestamp});
      }
    }
  }
}

void LeftImageThread(std::shared_ptr<dai::Device> device,
                     std::deque<std::shared_ptr<dai::ImgFrame>> &qImages) {
  auto qLeft = device->getOutputQueue("left", 4, false);
  while (mainLoopFlag) {
    auto inLeft = qLeft->get<dai::ImgFrame>();
    if (inLeft) {
      qImages.emplace_back(inLeft);
    }
  }
}

void RightImageThread(std::shared_ptr<dai::Device> device,
                      std::deque<std::shared_ptr<dai::ImgFrame>> &qImages) {
  auto qRight = device->getOutputQueue("right", 4, false);
  while (mainLoopFlag) {
    auto inRight = qRight->get<dai::ImgFrame>();
    if (inRight) {
      qImages.emplace_back(inRight);
    }
  }
}

int main(int argc, char **argv) {
  // if (argc != 4) {
  //   cerr << endl
  //        << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings "
  //           "path_to_sequence"
  //        << endl;
  //   return 1;
  // }

  // 注册信号处理器
  signal(SIGINT, signal_handler);
  auto device = createDevice();

  auto qLeft = device->getOutputQueue("left", 4, false);
  auto qRight = device->getOutputQueue("right", 4, false);

  double t_track = 0.f;
  double t_resize = 0.f;

  // Main loop
  cv::Mat imLeft, imRight;

  std::mutex imuMutex;
  std::vector<ORB_SLAM3::IMU::Point> vImuMeans;
  std::thread imuThread{ImuThread, device, std::ref(vImuMeans),
                        std::ref(imuMutex)};
  std::deque<std::shared_ptr<dai::ImgFrame>> qLeftImages, qRightImages;
  std::thread leftImageThread(&LeftImageThread, device, std::ref(qLeftImages));
  std::thread rightImageThread{RightImageThread, device,
                               std::ref(qRightImages)};

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;

  const int nImages = vstrImageLeft.size();

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, true);
  float imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  int all = 0;
  int lon = 0;
  while (mainLoopFlag) {
    while (qLeftImages.empty() || qRightImages.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    auto inLeft = qLeftImages.front();
    qLeftImages.pop_front();
    auto inRight = qRightImages.front();
    qRightImages.pop_front();

    using namespace std::chrono_literals;
    auto durationLeft = inRight->getTimestamp().time_since_epoch();
    auto durationRight = inLeft->getTimestamp().time_since_epoch();
/*
    while (durationLeft - durationRight > 20ms ||
           durationRight - durationLeft > 20ms) {
      if (durationLeft > durationRight) {
        if (!qRightImages.empty()) {
          inRight = qRightImages.front();
          qRightImages.pop_front();
        } else {
          break;
        }
      } else if (durationLeft < durationRight) {
        if (!qLeftImages.empty()) {
          inLeft = qLeftImages.front();
          qLeftImages.pop_front();
        } else {
          break;
        }
      }
      durationLeft = inRight->getTimestamp().time_since_epoch();
      durationRight = inLeft->getTimestamp().time_since_epoch();
    }
    if (durationLeft - durationRight > 20ms ||
        durationRight - durationLeft > 20ms) {
      continue;
    }
*/
    double timestampLeft =
        chrono::duration_cast<chrono::duration<double>>(durationLeft).count();
    double timestampRight =
        chrono::duration_cast<chrono::duration<double>>(durationRight).count();
    double interal = (timestampLeft - timestampRight) * 1e3;

    imLeft = inLeft->getCvFrame();
    imRight = inRight->getCvFrame();

    // get IMU
    vector<ORB_SLAM3::IMU::Point> vImuTemp{};
    {
      std::lock_guard<std::mutex> l{imuMutex};
      for (auto imu : vImuMeans) {
        vImuTemp.emplace_back(imu);
      }
      vImuMeans.clear();
    }

    fmt::print("[imu num]: {}\n", vImuTemp.size());

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, timestampLeft);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();


  }


  if (imuThread.joinable())
    imuThread.join();
  if (leftImageThread.joinable())
    leftImageThread.join();
  if (rightImageThread.joinable())
    rightImageThread.join();


  // Stop all threads
  SLAM.Shutdown();

  return 0;
}