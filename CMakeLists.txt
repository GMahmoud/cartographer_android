# Copyright 2016 The Cartographer Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 2.8.12)  # Ships with Ubuntu 14.04 (Trusty)

set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_ANDROID_ARM_MODE ON)
set(ANDROID_TOOLCHAIN /opt/toolchain)
set(ANDROID_SYSROOT ${ANDROID_TOOLCHAIN}/sysroot/usr/include)
set(CMAKE_ANDROID_STANDALONE_TOOLCHAIN gcc)
set(CMAKE_C_COMPILER ${ANDROID_TOOLCHAIN}/bin/arm-linux-androideabi-gcc)
set(CMAKE_CXX_COMPILER ${ANDROID_TOOLCHAIN}/bin/arm-linux-androideabi-g++)

project(cartographer)

set(CARTOGRAPHER_MAJOR_VERSION 1)
set(CARTOGRAPHER_MINOR_VERSION 0)
set(CARTOGRAPHER_PATCH_VERSION 0)
set(CARTOGRAPHER_VERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION}.${CARTOGRAPHER_PATCH_VERSION})
set(CARTOGRAPHER_SOVERSION ${CARTOGRAPHER_MAJOR_VERSION}.${CARTOGRAPHER_MINOR_VERSION})

set(BOOST_HDR ${ANDROID_SYSROOT}/boost-1_53)
set(LUA_HDR ${ANDROID_SYSROOT}/lua-5.2.4)
set(EIGEN_HDR ${ANDROID_SYSROOT}/eigen3)
set(PROTOBUF_PROTOC_EXECUTABLE /usr/bin/protoc) 

include("${PROJECT_SOURCE_DIR}/cmake/functions.cmake")
message(STATUS "-----------------------")
message(STATUS ${PROJECT_SOURCE_DIR}/cmake/functions.cmake)
message(STATUS "-----------------------")

include_directories(${LUA_HDR})
include_directories(${BOOST_HDR})
include_directories(${ANDROID_SYSROOT})

set(PROTOBUF_PROTOC_EXECUTABLE /usr/bin/protoc)

google_initialize_cartographer_project()
#google_enable_testing()

# Install catkin package.xml
install(FILES package.xml DESTINATION share/cartographer)

set(CARTOGRAPHER_CONFIGURATION_FILES_DIRECTORY ${CMAKE_INSTALL_PREFIX}/share/cartographer/configuration_files
  CACHE PATH ".lua configuration files directory")

install(DIRECTORY configuration_files DESTINATION share/cartographer/)

install(DIRECTORY cmake DESTINATION share/cartographer/)

file(GLOB_RECURSE ALL_SRCS "*.cc" "*.h")
file(GLOB_RECURSE ALL_TESTS "*_test.cc")
file(GLOB_RECURSE ALL_EXECUTABLES "*_main.cc")
list(REMOVE_ITEM ALL_SRCS ${ALL_TESTS})
list(REMOVE_ITEM ALL_SRCS ${ALL_EXECUTABLES})

file(GLOB_RECURSE ALL_PROTOS "*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)

  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")

  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_SRCS ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

add_library(${PROJECT_NAME} ${ALL_SRCS})
configure_file(
  ${PROJECT_SOURCE_DIR}/cartographer/common/config.h.cmake
  ${PROJECT_BINARY_DIR}/cartographer/common/config.h)

google_binary(cartographer_autogenerate_ground_truth
  SRCS
    cartographer/ground_truth/autogenerate_ground_truth_main.cc
)

google_binary(cartographer_compute_relations_metrics
  SRCS
    cartographer/ground_truth/compute_relations_metrics_main.cc
)

foreach(ABS_FIL ${ALL_TESTS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  # Replace slashes as required for CMP0037.
  string(REPLACE "/" "." TEST_TARGET_NAME "${DIR}/${FIL_WE}")
  google_test("${TEST_TARGET_NAME}" ${ABS_FIL})
endforeach()

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN_HDR}")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_HDR}")
find_library(
    CERES_LIB
    NAMES ceres
    PATHS ${PROJECT_SOURCE_DIR}/../libs
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${CERES_LIB})
message(STATUS "-----------------------")
message(STATUS ${CERES_LIB})
message(STATUS "-----------------------")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_HDR}")
#target_link_libraries(${PROJECT_NAME} PUBLIC llua)
find_library(
    LUA_LIB
    NAMES lua
    PATHS ${PROJECT_SOURCE_DIR}/../libs
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${LUA_LIB})
message(STATUS "-----------------------")
message(STATUS ${LUA_LIB})
message(STATUS "-----------------------")

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${BOOST_HDR}")
find_library(
    BOOST_LIB
    NAMES boost_iostreams-gcc-mt-1_53
    PATHS ${PROJECT_SOURCE_DIR}/../libs
    PATH_SUFFIXES lib
    NO_DEFAULT_PATH
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${BOOST_LIB})
message(STATUS "-----------------------")
message(STATUS ${BOOST_LIB})
message(STATUS "-----------------------")

# We expect find_package(Ceres) to have located these for us.
target_link_libraries(${PROJECT_NAME} PUBLIC glog)
target_link_libraries(${PROJECT_NAME} PUBLIC gflags)

#target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
#  "${CAIRO_INCLUDE_DIRS}")
#target_link_libraries(${PROJECT_NAME} PUBLIC ${CAIRO_LIBRARIES})

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  ${PROTOBUF_INCLUDE_DIR})
# TODO(hrapp): This should not explicitly list pthread and use
# PROTOBUF_LIBRARIES, but that failed on first try.
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY} pthread)

# Add the binary directory first, so that port.h is included after it has
# been generated.
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# TODO(damonkohler): Create a testing library.
# target_include_directories(${PROJECT_NAME} SYSTEM PRIVATE
#  "${GMOCK_INCLUDE_DIRS}")
# target_link_libraries(${PROJECT_NAME} PUBLIC ${GMOCK_LIBRARY})

set(TARGET_COMPILE_FLAGS "${TARGET_COMPILE_FLAGS} ${GOOG_CXX_FLAGS}")
set_target_properties(${PROJECT_NAME} PROPERTIES
  COMPILE_FLAGS ${TARGET_COMPILE_FLAGS})

install(
  TARGETS ${PROJECT_NAME}
  EXPORT CartographerExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install source headers.
file(GLOB_RECURSE hdrs "*.h")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(INSTALL_DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${INSTALL_DIR}
  )
endforeach()

# Install generated headers.
file(GLOB_RECURSE hdrs "*.h.in")
foreach(HDR ${hdrs})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  install(
    FILES
      ${PROJECT_BINARY_DIR}/${DIR}/${FILE_WE}
    DESTINATION
      include/${DIR}
  )
endforeach()

# Install proto headers.
foreach(HDR ${ALL_PROTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_BINARY_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES
      ${HDR}
    DESTINATION
      include/${DIR}
  )
endforeach()

set(CARTOGRAPHER_CMAKE_DIR share/cartographer/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cartographer-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  PATH_VARS CARTOGRAPHER_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/cartographer
)

install(
  EXPORT CartographerExport
  DESTINATION share/cartographer/cmake/
  FILE CartographerTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/cartographer/cartographer-config.cmake
  DESTINATION share/cartographer/
)
