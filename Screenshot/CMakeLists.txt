cmake_minimum_required(VERSION 3.0)

# Create a project
project( capture_image )

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCV packages
find_package( OpenCV REQUIRED )

# Define the executable helloCV
add_executable( capture_image capture_image.cpp )
target_link_libraries( capture_image ${OpenCV_LIBS} )
