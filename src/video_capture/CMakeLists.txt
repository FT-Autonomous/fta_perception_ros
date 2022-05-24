cmake_minimum_required(VERSION 3.8)
project(video_capture)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(perception_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(video_capture src/video_capture.cpp)
target_include_directories(video_capture PRIVATE "/usr/include/opencv4/")
target_link_options(video_capture PUBLIC "-I/usr/include/opencv4" "-lopencv_gapi" "-lopencv_stitching" "-lopencv_alphamat" "-lopencv_aruco" "-lopencv_barcode" "-lopencv_bgsegm" "-lopencv_bioinspired" "-lopencv_ccalib" "-lopencv_cvv" "-lopencv_dnn_objdetect" "-lopencv_dnn_superres" "-lopencv_dpm" "-lopencv_face" "-lopencv_freetype" "-lopencv_fuzzy" "-lopencv_hdf" "-lopencv_hfs" "-lopencv_img_hash" "-lopencv_intensity_transform" "-lopencv_line_descriptor" "-lopencv_mcc" "-lopencv_quality" "-lopencv_rapid" "-lopencv_reg" "-lopencv_rgbd" "-lopencv_saliency" "-lopencv_stereo" "-lopencv_structured_light" "-lopencv_phase_unwrapping" "-lopencv_superres" "-lopencv_optflow" "-lopencv_surface_matching" "-lopencv_tracking" "-lopencv_highgui" "-lopencv_datasets" "-lopencv_text" "-lopencv_plot" "-lopencv_videostab" "-lopencv_videoio" "-lopencv_viz" "-lopencv_wechat_qrcode" "-lopencv_xfeatures2d" "-lopencv_shape" "-lopencv_ml" "-lopencv_ximgproc" "-lopencv_video" "-lopencv_xobjdetect" "-lopencv_objdetect" "-lopencv_calib3d" "-lopencv_imgcodecs" "-lopencv_features2d" "-lopencv_dnn" "-lopencv_flann" "-lopencv_xphoto" "-lopencv_photo" "-lopencv_imgproc" "-lopencv_core")
ament_target_dependencies(video_capture rclcpp sensor_msgs perception_msgs)

install(TARGETS video_capture
        DESTINATION lib/${PROJECT_NAME})

file(GLOB dirs LIST_DIRECTORIES true build/${PROJECT_NAME}/*_cpp)
list(APPEND dirs "/opt/ros2/galactic/include/"
                 "/usr/include/opencv4/")
list(TRANSFORM dirs PREPEND "-I")
list(JOIN dirs "," to_output)
file(WRITE .clangd "CompileFlags:\n\tAdd: [${to_output}]")

ament_package()
