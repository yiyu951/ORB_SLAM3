add_executable(stereo_test stereo_test.cpp)

target_link_libraries(stereo_test
    boost_serialization
    crypto
    fmt::fmt
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    ${realsense2_LIBRARY}
)