cmake_minimum_required(VERSION 3.14)
project(hazmat_detector LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio dnn imgcodecs)

add_executable(hazmat_detector hazmat_detector.cpp)
target_link_libraries(hazmat_detector PRIVATE ${OpenCV_LIBS})
