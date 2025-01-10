cmake_minimum_required(VERSION 3.0)
set(CMAKE_CXX_STANDARD 11)

project(dwa_planner)

find_package(OpenCV REQUIRED )
find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(Eigen3 REQUIRED) # <-- "REQUIRED" ensures EIGEN3_FOUND is ON

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    include
)

# Link libraries
set(EXTERNAL_LIBS 
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
)

# Setup the target executable.
add_executable(
    dwa_planner 
    src/main.cpp 
    src/obstacles.cpp 
    src/utils.cpp
)

target_link_libraries(
    dwa_planner
    ${EXTERNAL_LIBS}
)
