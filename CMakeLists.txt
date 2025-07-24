cmake_minimum_required(VERSION 3.22)
project(Gravity3D)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -openmp:llvm")

find_package(OpenMP REQUIRED)

# Locate OpenCV
set(OpenCV_DIR "D:/Downloads/opencv/build")
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(Gravity3D
        data.cpp
        main.cpp
        simulation.cpp
        render.cpp
        viewData.cpp
)

# Include OpenCV headers
include_directories(${OpenCV_INCLUDE_DIRS})

# Link OpenCV libraries
target_link_libraries(Gravity3D PRIVATE OpenMP::OpenMP_CXX ${OpenCV_LIBS})
