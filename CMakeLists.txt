cmake_minimum_required(VERSION 3.31)
project(Gravy3D)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp:llvm")

find_package(OpenMP REQUIRED)

set(OpenCV_DIR "D:/Downloads/opencv/build")

        # Locate OpenCV
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(untitled30 main.cpp)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Link OpenCV libraries
target_link_libraries(untitled30 PRIVATE OpenMP::OpenMP_CXX ${OpenCV_LIBS})
