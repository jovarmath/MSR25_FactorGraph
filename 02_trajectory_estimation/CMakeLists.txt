# ----------------------------------------------------------------------------------
# Create Project
# ----------------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.5) # 3.0
project(TR76 
        VERSION 3.0
        DESCRIPTION "ISAM GNSS IMU TS trajectory optimization"
        LANGUAGES CXX
        )

# ----------------------------------------------------------------------------------
# 1. Search pkg and libraries
# ----------------------------------------------------------------------------------

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# add GoogleTest
find_package(GTest)
enable_testing()

# Find GTSAM, either from a local build, or from a Debian/Ubuntu package.
find_package(GTSAM REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development)

message(STATUS "GTSAM Version: ${GTSAM_VERSION}")

#find_package(ALGLIB REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

# add mathplot
#find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

# Add Matplotlib
find_package(PythonLibs 3.8.10 REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)

# ----------------------------------------------------------------------------------
# 2. Set some flags for compilation
# ----------------------------------------------------------------------------------

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

# ----------------------------------------------------------------------------------
# 3. Tell CMAKE to output binaries here
# ----------------------------------------------------------------------------------

set( CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set( CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PROJECT_SOURCE_DIR}/include)

# ----------------------------------------------------------------------------------
# 4. Crate/Add my libraries 
# ----------------------------------------------------------------------------------

add_library(Print src/Print.cpp)

add_library(TrajectoryReport src/TrajectoryReport.cpp)
target_link_libraries(TrajectoryReport PUBLIC gtsam Data)
                     
add_library(Data src/Data.cpp)

target_link_libraries(Data PUBLIC Python3::Python PUBLIC gtsam TrajectoryReport )

add_library(FactorGraph src/FactorGraph.cpp
                        src/GNSSHeadingFactor.cpp
                        src/GNSSFactor.cpp
                        src/TSFactor.cpp
                        src/TSBaselineFactor.cpp
                        src/IMUstochastics.cpp
                        src/GPSstochastics.cpp
                        src/GPSPitchHeadingstochastics.cpp
                        src/TSstochastics.cpp
                        src/SystemConfig.cpp
                        src/iSAMConfig.cpp
                        src/TSBaselineFactorstochastics.cpp)

target_link_libraries(FactorGraph PUBLIC gtsam Data Print TrajectoryReport )

# ----------------------------------------------------------------------------------
# 5. Create Executables and target libs
# ----------------------------------------------------------------------------------

set(main on)

if(main)
        add_executable(main
                       main.cpp
                       src/IMUstochastics.cpp
                       src/GPSstochastics.cpp
                       src/GPSPitchHeadingstochastics.cpp
                       src/SystemConfig.cpp
                       src/TSstochastics.cpp
                       src/TSBaselineFactorstochastics.cpp
                       src/iSAMConfig.cpp )
        # Link libs
        target_link_libraries(main FactorGraph)
endif()

          
