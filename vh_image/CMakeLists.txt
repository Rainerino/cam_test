cmake_minimum_required(VERSION 3.5)
project(vh_image)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(PkgConfig)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(cv)
pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.14
                               gstreamer-sdp-1.0>=1.14
                               gstreamer-video-1.0>=1.14
                               gstreamer-app-1.0>=1.14
                               gstreamer-codecparsers-1.0>=1.14)
find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies  "rclcpp"
                  "sensor_msgs" 
                  "cv_bridge"
                  "OpenCV"
)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(vh_image 
        src/main.cpp
        src/vh_appsink.cpp
)

target_include_directories(vh_image PRIVATE 
          include/${PROJECT_NAME} 
          ${GST_INCLUDE_DIRS} 
          ${OpenCV_INCLUDE_DIRS})

ament_target_dependencies(vh_image ${dependencies})

target_link_libraries(vh_image 
        gstreamer-1.0
        gobject-2.0
        glib-2.0
        pthread 
        ${OpenCV_LIBS})

install(TARGETS vh_image 
  DESTINATION lib/${PROJECT_NAME})

# install the launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

  
ament_package()
