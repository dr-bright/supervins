cmake_minimum_required(VERSION 3.14)
project(camera_models)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -fPIC")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    )

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)
include_directories(${Boost_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)

include(FetchContent)

# Eigen
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY  https://gitlab.com/libeigen/eigen.git
    GIT_TAG         3.4.0
    GIT_SHALLOW     TRUE
    GIT_PROGRESS    TRUE
)
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)
set(EIGEN_BUILD_DOC OFF)
set(EIGEN_BUILD_PKGCONFIG OFF)
set(EIGEN_TEST_CXX11 ON)
set(EIGEN_HAS_CXX11_MATH ON)
list(APPEND external Eigen)
list(APPEND libs Eigen3::Eigen)

# Ceres
find_package(Ceres REQUIRED)

FetchContent_MakeAvailable(${external})


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES camera_models
    CATKIN_DEPENDS roscpp std_msgs
#    DEPENDS system_lib
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )

include_directories("include")

add_executable(Calibrations 
    src/intrinsic_calib.cc
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

add_library(camera_models
    src/chessboard/Chessboard.cc
    src/calib/CameraCalibration.cc
    src/camera_models/Camera.cc
    src/camera_models/CameraFactory.cc
    src/camera_models/CostFunctionFactory.cc
    src/camera_models/PinholeCamera.cc
    src/camera_models/PinholeFullCamera.cc
    src/camera_models/CataCamera.cc
    src/camera_models/EquidistantCamera.cc
    src/camera_models/ScaramuzzaCamera.cc
    src/sparse_graph/Transform.cc
    src/gpl/gpl.cc
    src/gpl/EigenQuaternionParameterization.cc)

target_link_libraries(Calibrations ${Boost_LIBRARIES} ${OpenCV_LIBS} Ceres::ceres)
target_link_libraries(camera_models ${Boost_LIBRARIES} ${OpenCV_LIBS} Ceres::ceres)
