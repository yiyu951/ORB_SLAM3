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

#include <System.h>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

int main(int argc, char **argv) {
  if (argc != 4) {
    cerr << endl
         << "Usage: ./stereo_kitti path_to_vocabulary path_to_settings "
            "path_to_sequence"
         << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageLeft;
  vector<string> vstrImageRight;
  vector<double> vTimestamps;
  LoadImages(string(argv[3]), vstrImageLeft, vstrImageRight, vTimestamps);
  const int nImages = vstrImageLeft.size();

  vector<cv::Point3f> vAcc, vGyro;
  vector<double> vTimestampsImu;
  LoadIMU(argv[3], vTimestampsImu, vAcc, vGyro);
  int nImu = vTimestampsImu.size();
  int first_imu = 0;
  while (vTimestampsImu[first_imu] <= vTimestamps[0])
    first_imu++;
  first_imu--; // first imu measurement to be considered

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, false);
  float imageScale = SLAM.GetImageScale();

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  double t_track = 0.f;
  double t_resize = 0.f;

  // Main loop
  cv::Mat imLeft, imRight;
  // Seq loop
  vector<ORB_SLAM3::IMU::Point> vImuMeas;
  double t_rect = 0.f;
  int num_rect = 0;
  int proccIm = 0;

  for (int ni = 0; ni < nImages; ni++) {
    // Read left and right images from file
    imLeft = cv::imread(vstrImageLeft[ni],
                        cv::IMREAD_GRAYSCALE); //,cv::IMREAD_UNCHANGED);
    imRight = cv::imread(vstrImageRight[ni],
                         cv::IMREAD_GRAYSCALE); //,cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (imLeft.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(vstrImageLeft[ni]) << endl;
      return 1;
    }

    if (imageScale != 1.f) {

      int width = imLeft.cols * imageScale;
      int height = imLeft.rows * imageScale;
      cv::resize(imLeft, imLeft, cv::Size(width, height));
      cv::resize(imRight, imRight, cv::Size(width, height));
    }

    // Load imu measurements from previous frame
    vImuMeas.clear();

    if (ni > 0)
      while (vTimestampsImu[first_imu] <= vTimestamps[ni] && first_imu < nImu) {
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(
            vAcc[first_imu].x, vAcc[first_imu].y, vAcc[first_imu].z,
            vGyro[first_imu].x, vGyro[first_imu].y, vGyro[first_imu].z,
            vTimestampsImu[first_imu]));
        first_imu++;
      }

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    // Pass the images to the SLAM system
    SLAM.TrackStereo(imLeft, imRight, tframe, vImuMeas);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    vTimesTrack[ni] = ttrack;

    // Wait to load the next frame
    double T = 0;
    if (ni < nImages - 1)
      T = vTimestamps[ni + 1] - tframe;
    else if (ni > 0)
      T = tframe - vTimestamps[ni - 1];

    if (ttrack < T)
      usleep((T - ttrack) * 1e6);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps) {
  ifstream fTimes;
  string strPathTimeFile = strPathToSequence + "/times.txt";
  fTimes.open(strPathTimeFile.c_str());
  while (!fTimes.eof()) {
    string s;
    getline(fTimes, s);
    if (!s.empty()) {
      stringstream ss;
      ss << s;
      double t;
      ss >> t;
      vTimestamps.push_back(t);
    }
  }

  string strPrefixLeft = strPathToSequence + "/left/";
  string strPrefixRight = strPathToSequence + "/right/";

  const int nTimes = vTimestamps.size();
  vstrImageLeft.resize(nTimes);
  vstrImageRight.resize(nTimes);

  for (int i = 0; i < nTimes; i++) {
    // stringstream ss;
    // ss << setfill('0') << setw(6) << i;
    vstrImageLeft[i] = strPrefixLeft + std::to_string(i) + ".png";
    vstrImageRight[i] = strPrefixRight + std::to_string(i) + ".png";
  }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps,
             vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro) {
  ifstream fImu;
  fImu.open((strImuPath + "/imu.txt").c_str());
  vTimeStamps.reserve(5000);
  vAcc.reserve(5000);
  vGyro.reserve(5000);

  while (!fImu.eof()) {
    string s;
    getline(fImu, s);
    if (s[0] == '#')
      continue;

    if (!s.empty()) {
      string item;
      size_t pos = 0;
      double data[7];
      int count = 0;
      while ((pos = s.find(',')) != string::npos) {
        item = s.substr(0, pos);
        data[count++] = stod(item);
        s.erase(0, pos + 1);
      }
      item = s.substr(0, pos);
      data[6] = stod(item);

      vTimeStamps.push_back(data[0]);
      vAcc.push_back(cv::Point3f(data[1], data[2], data[3]));
      vGyro.push_back(cv::Point3f(data[4], data[5], data[6]));
    }
  }
}
