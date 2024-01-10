# ORB-SLAM3 学习

## 1. mono-inertial(单目-惯性)

### 1.1 通用启动流程

```
// 运行参数
path_to_vocabulary
path_to_settings
path_to_sequence_folder_1
path_to_times_file_1
```



```C++
// System类
System(const string &strVocFile=argv[1], const string &strSettingsFile=argv[2], const eSensor sensor, const bool bUseViewer=True)
```

```C++
// 进入 (image, timestamp, imu_means)
SLAM.TrackMonocular(im,tframe,vImuMeas);
```

### 1.2 System::TrackMonocular

1. mbShutDown 判断是否返回空
2. settings_->needToResize() 图像resize
3. 检查两个参数，判断是否切换模式
   1. 用到类LocalMapping、Tracking
   2. 两个参数mbActivateLocalizationMode、mbDeactivateLocalizationMode
4. 判断是否reset
   1. mbReset ---> mpTracker->Reset()
   2. mbResetActiveMap ---> mpTracker->ResetActiveMap()
5. 添加IMU数据: mpTracker->GrabImuData()
6. 开始图像处理: mpTracker->GrabImageMonocular()

### 1.3 Tracking::GrabImageMonocular



