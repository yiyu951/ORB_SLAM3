cmake_minimum_required(VERSION 3.0 )
project(ORB_SLAM3)

SET(CMAKE_BUILD_TYPE Debug)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})
set(CMAKE_CXX_STANDARD  14)
set(CMAKE_CXX_STANDARD_REQUIRED  ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS  ON)
#set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -w -Wall   -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -w -Wall   -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

add_definitions(-DCOMPILEDWITHC11)

# Check C++11 or C++0x support
# include(CheckCXXCompilerFlag)
# CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
# CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
# if(COMPILER_SUPPORTS_CXX11)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#    add_definitions(-DCOMPILEDWITHC11)
#    message(STATUS "Using flag -std=c++11.")
# elseif(COMPILER_SUPPORTS_CXX0X)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#    add_definitions(-DCOMPILEDWITHC0X)
#    message(STATUS "Using flag -std=c++0x.")
# else()
#    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
# endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(OpenCV)
message("OpenCV VIRSION = ${OpenCV_VERSION}")
message("OpenCV INCLUDE = ${OpenCV_INCLUDE_DIRS}")

find_package(fmt REQUIRED)


# MESSAGE("OPENCV VERSION:")
# MESSAGE(${OpenCV_VERSION})

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(realsense2 REQUIRED)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/include
        ${PROJECT_SOURCE_DIR}/include/CameraModels
        ${PROJECT_SOURCE_DIR}/Thirdparty/Sophus
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)   

include_directories(${PROJECT_NAME}
        ${realsense_INCLUDE_DIR}
)


set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
        src/System.cc
        src/Tracking.cc
        src/LocalMapping.cc
        src/LoopClosing.cc
        src/ORBextractor.cc
        src/ORBmatcher.cc
        src/FrameDrawer.cc
        src/Converter.cc
        src/MapPoint.cc
        src/KeyFrame.cc
        src/Atlas.cc
        src/Map.cc
        src/MapDrawer.cc
        src/Optimizer.cc
        src/Frame.cc
        src/KeyFrameDatabase.cc
        src/Sim3Solver.cc
        src/Viewer.cc
        src/ImuTypes.cc
        src/G2oTypes.cc
        src/CameraModels/Pinhole.cpp
        src/CameraModels/KannalaBrandt8.cpp
        src/OptimizableTypes.cpp
        src/MLPnPsolver.cpp
        src/GeometricTools.cc
        src/TwoViewReconstruction.cc
        src/Config.cc
        src/Settings.cc
        include/System.h
        include/Tracking.h
        include/LocalMapping.h
        include/LoopClosing.h
        include/ORBextractor.h
        include/ORBmatcher.h
        include/FrameDrawer.h
        include/Converter.h
        include/MapPoint.h
        include/KeyFrame.h
        include/Atlas.h
        include/Map.h
        include/MapDrawer.h
        include/Optimizer.h
        include/Frame.h
        include/KeyFrameDatabase.h
        include/Sim3Solver.h
        include/Viewer.h
        include/ImuTypes.h
        include/G2oTypes.h
        include/CameraModels/GeometricCamera.h
        include/CameraModels/Pinhole.h
        include/CameraModels/KannalaBrandt8.h
        include/OptimizableTypes.h
        include/MLPnPsolver.h
        include/GeometricTools.h
        include/TwoViewReconstruction.h
        include/SerializationUtils.h
        include/Config.h
        include/Settings.h
)

add_subdirectory(Thirdparty/g2o)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        -lboost_serialization
        -lcrypto
        fmt::fmt

        ${realsense2_LIBRARY}
)


find_package(depthai CONFIG REQUIRED)

#Stereo examples
add_executable(oak
        Examples/Stereo/oak.cc)
target_link_libraries(oak 
        ${PROJECT_NAME}
        depthai::opencv
)



#Stereo Inertial examples
add_executable(oak_inertial
        Examples/Stereo-Inertial/oak.cc)
target_link_libraries(oak_inertial
        ${PROJECT_NAME}
        depthai::opencv
)



#Stereo Inertial examples
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/Examples)

add_executable(oak_realtime
        Examples/oak_realtime.cc)
target_link_libraries(oak_realtime
        ${PROJECT_NAME}
        depthai::opencv
        fmt::fmt
)


## D435i
# d435i Calibration
add_executable(recorder_realsense_D435i
Examples/Calibration/recorder_realsense_D435i.cc)
target_link_libraries(recorder_realsense_D435i ${PROJECT_NAME})


# stereo_inertial_realsense_D435i
add_executable(stereo_inertial_realsense_D435i
Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc)
target_link_libraries(stereo_inertial_realsense_D435i ${PROJECT_NAME})



# RGB-D examples
    add_executable(rgbd_realsense_D435i
            Examples/RGB-D/rgbd_realsense_D435i.cc)
    target_link_libraries(rgbd_realsense_D435i ${PROJECT_NAME})



# RGB-D inertial examples

    add_executable(rgbd_inertial_realsense_D435i
            Examples/RGB-D-Inertial/rgbd_inertial_realsense_D435i.cc)
    target_link_libraries(rgbd_inertial_realsense_D435i ${PROJECT_NAME})


#Stereo examples

    add_executable(stereo_realsense_D435i
            Examples/Stereo/stereo_realsense_D435i.cc)
    target_link_libraries(stereo_realsense_D435i ${PROJECT_NAME})


#Monocular examples
   add_executable(mono_realsense_D435i
            Examples/Monocular/mono_realsense_D435i.cc)
    target_link_libraries(mono_realsense_D435i ${PROJECT_NAME})

#Monocular inertial examples

    add_executable(mono_inertial_realsense_D435i
            Examples/Monocular-Inertial/mono_inertial_realsense_D435i.cc)
    target_link_libraries(mono_inertial_realsense_D435i ${PROJECT_NAME})



add_subdirectory(test)