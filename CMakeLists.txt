cmake_minimum_required(VERSION 3.0)
project(imu_algorithms)

set(CMAKE_CXX_STANDARD 11)

# set(CMAKE_CXX_STANDARD 11)
# aux_source_directory(${PROJECT_SOURCE_DIR}/src SRC)
aux_source_directory(${PROJECT_SOURCE_DIR} SRC)
include_directories(${PROJECT_SOURCE_DIR}/include)

# add_library(imu_alg SHARED ${SRC})
# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# add_executable(app imu_alg)
# link_directories(${PROJECT_SOURCE_DIR}/bin)

link_directories(${PROJECT_SOURCE_DIR}/bin)
add_executable(app ${SRC})
target_link_libraries(app imu_alg)