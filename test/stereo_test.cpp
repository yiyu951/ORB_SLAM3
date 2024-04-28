#include <signal.h>
#include <stdlib.h>
#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>

#include <condition_variable>

#include <opencv2/core/core.hpp>

#include <librealsense2/rs.hpp>
#include "librealsense2/rsutil.h"

#include <System.h>

bool b_continue_session;

void exit_loop_handler(int /*s*/)
{
    std::cout << "Finishing session" << '\n';
    b_continue_session = false;
}

int main()
{
    // 信号量注册
    struct sigaction sigIntHandler
    {
    };

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    double offset = 0;  // ms

    // realsense 初始化

    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device" << '\n';
        return 0;
    }
    // Get Device
    selected_device = devices[0];

    // Get Sensors
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();
    std::cout << "sensor size"
              << " : " << sensors.size() << '\n';
    int index = 0;

    for (rs2::sensor sensor : sensors) {
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) {
            ++index;
            if (index == 1) {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                // sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_LIMIT, 8000);
                sensor.set_option(RS2_OPTION_EXPOSURE, 8000);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0);  // switch off emitter
            }
            std::cout << "  " << index << " : " << sensor.get_info(RS2_CAMERA_INFO_NAME)
                      << std::endl;
            //   get_sensor_option(sensor);
            if (index == 2) {
                // RGB camera (not used here...)
                // sensor.set_option(RS2_OPTION_EXPOSURE, 100.f);
            }

            if (index == 3) {
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
            }
        }
    }

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default
    // profile
    rs2::config cfg;
    int width  = 640;
    int height = 480;
    cfg.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // IMU callback
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    vector<double> v_accel_timestamp;
    vector<rs2_vector> v_accel_data;
    vector<double> v_gyro_timestamp;
    vector<rs2_vector> v_gyro_data;

    double prev_accel_timestamp = 0;
    rs2_vector prev_accel_data;
    double current_accel_timestamp = 0;
    rs2_vector current_accel_data;
    vector<double> v_accel_timestamp_sync;
    vector<rs2_vector> v_accel_data_sync;

    cv::Mat imCV;
    cv::Mat imRightCV;
    int width_img          = 0;
    int height_img         = 0;
    double timestamp_image = -1.0;
    bool image_ready{false};
    int count_im_buffer = 0;  // count dropped frames

    auto imu_callback = [&](const rs2::frame & frame) {
        std::unique_lock<std::mutex> lock(imu_mutex);

        if (rs2::frameset fs = frame.as<rs2::frameset>()) {

            std::cout << "enter realsense callbakc\n";
            count_im_buffer++;

            double new_timestamp_image = fs.get_timestamp() * 1e-3;
            if (abs(timestamp_image - new_timestamp_image) < 0.001) {
                // cout << "Two frames with the same timeStamp!!!\n";
                count_im_buffer--;
                return;
            }

            rs2::video_frame ir_frameL = fs.get_infrared_frame(1);
            rs2::video_frame ir_frameR = fs.get_infrared_frame(2);

            imCV = cv::Mat(
                cv::Size(width_img, height_img), CV_8U, (void *)(ir_frameL.get_data()),
                cv::Mat::AUTO_STEP);
            imRightCV = cv::Mat(
                cv::Size(width_img, height_img), CV_8U, (void *)(ir_frameR.get_data()),
                cv::Mat::AUTO_STEP);

            timestamp_image = fs.get_timestamp() * 1e-3;
            image_ready     = true;

            // while (v_gyro_timestamp.size() > v_accel_timestamp_sync.size()) {
            //     int index          = v_accel_timestamp_sync.size();
            //     double target_time = v_gyro_timestamp[index];

            //     v_accel_data_sync.push_back(current_accel_data);
            //     v_accel_timestamp_sync.push_back(target_time);
            // }

            lock.unlock();
            cond_image_rec.notify_all();
        } else if (rs2::motion_frame m_frame = frame.as<rs2::motion_frame>()) {
            /*
            
            if (m_frame.get_profile().stream_name() == "Gyro") {
                // It runs at 200Hz
                v_gyro_data.push_back(m_frame.get_motion_data());
                v_gyro_timestamp.push_back((m_frame.get_timestamp() + offset) * 1e-3);
                rs2_vector gyro_sample = m_frame.get_motion_data();
                std::cout << "Gyro:" << gyro_sample.x << ", " << gyro_sample.y << ", "
                          << gyro_sample.z << std::endl;
            } else if (m_frame.get_profile().stream_name() == "Accel") {
                // It runs at 60Hz
                prev_accel_timestamp = current_accel_timestamp;
                prev_accel_data      = current_accel_data;

                current_accel_data      = m_frame.get_motion_data();
                current_accel_timestamp = (m_frame.get_timestamp() + offset) * 1e-3;

                std::cout << "Accel:" << current_accel_data.x << ", " << current_accel_data.y
                          << ", " << current_accel_data.z << std::endl;
            }

            */
        }
    };

    rs2::pipeline_profile pipe_profile = pipe.start(cfg, imu_callback);

    rs2::stream_profile cam_left  = pipe_profile.get_stream(RS2_STREAM_INFRARED, 1);
    rs2::stream_profile cam_right = pipe_profile.get_stream(RS2_STREAM_INFRARED, 2);

    rs2::stream_profile imu_stream = pipe_profile.get_stream(RS2_STREAM_GYRO);

    // 外参
    float * Rbc = cam_left.get_extrinsics_to(imu_stream).rotation;
    float * tbc = cam_left.get_extrinsics_to(imu_stream).translation;
    //   std::cout << "Tbc (left) = " << std::endl;
    //   for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++)
    //       std::cout << Rbc[i * 3 + j] << ", ";
    //     std::cout << tbc[i] << "\n";
    //   }

    float * Rlr = cam_right.get_extrinsics_to(cam_left).rotation;
    float * tlr = cam_right.get_extrinsics_to(cam_left).translation;
    //   std::cout << "Tlr  = " << std::endl;
    //   for (int i = 0; i < 3; i++) {
    //     for (int j = 0; j < 3; j++)
    //       std::cout << Rlr[i * 3 + j] << ", ";
    //     std::cout << tlr[i] << "\n";
    //   }

    // 内参
    rs2_intrinsics intrinsics_left = cam_left.as<rs2::video_stream_profile>().get_intrinsics();
    width_img                      = intrinsics_left.width;
    height_img                     = intrinsics_left.height;

    rs2_intrinsics intrinsics_right = cam_right.as<rs2::video_stream_profile>().get_intrinsics();
    width_img                       = intrinsics_right.width;
    height_img                      = intrinsics_right.height;
    std::chrono::steady_clock::time_point prev_time;
    cv::Mat im;
    cv::Mat imRight;
    while (b_continue_session) {

        std::chrono::steady_clock::time_point now_time = std::chrono::steady_clock::now();

        auto t = std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(
                     now_time - prev_time)
                     .count();
        std::cout << t << "ms\n";
        prev_time = now_time;

        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if (!image_ready) {
                std::cout << "wait image\n";
                cond_image_rec.wait(lk);
            }

            im          = imCV.clone();
            imRight     = imRightCV.clone();
            image_ready = false;
        }
        std::cout << "get Image\n";
        cv::imshow("left", im);
        cv::imshow("right", imRight);
        int k = cv::waitKey(10);
        if (k == 27) {
            std::cout << "exit\n";
            return 0;
        }
    }

    return 0;
}