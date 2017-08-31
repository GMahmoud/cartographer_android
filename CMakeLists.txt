cmake_minimum_required(VERSION 2.8.12)
## ************************************************************* CMAKE TOOLCHAIN & PARAMS
set (CMAKE_CXX_STANDARD 11) #with cmake 3.1 only, lesser version should use : set(CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_SYSTEM_NAME Android)
set(CMAKE_ANDROID_ARM_MODE ON)
set(ANDROID_TOOLCHAIN /opt/toolchain)
set(ANDROID_SYSROOT ${ANDROID_TOOLCHAIN}/sysroot/usr/include)
set(CMAKE_ANDROID_STANDALONE_TOOLCHAIN gcc)
set(CMAKE_CXX_COMPILER ${ANDROID_TOOLCHAIN}/bin/arm-linux-androideabi-g++)
set(CMAKE_C_COMPILER ${ANDROID_TOOLCHAIN}/bin/arm-linux-androideabi-gcc)

project(test-carto-generic)
## ************************************************************* INCLUDING EXTERNAL HDRS & PROTOC EXE
set(LUA_HDR ${ANDROID_SYSROOT}/lua-5.2.4)
set(EIGEN_HDR ${ANDROID_SYSROOT}/eigen3-3-4)
set(BOOST_HDR ${ANDROID_SYSROOT}/boost-1_53)
set(PROTOBUF_HDR ${ANDROID_SYSROOT})
set(CERES_HDR ${ANDROID_SYSROOT})
set(GLOG_HDR ${ANDROID_SYSROOT})
set(PROTOBUF_PROTOC_EXECUTABLE /usr/local/bin/protoc) 
link_directories(../libs)

## ************************************************************* GENERATING THE pb.c and the pb.h files 
set(PROJECT_INSTALL_DIR ${PROJECT_BINARY_DIR}/install)
set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
file(MAKE_DIRECTORY ${PROJECT_INSTALL_DIR})
set(CARTO_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src)
file(GLOB_RECURSE ALL_PROTOS "src/cartographer/*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)

foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${CARTO_SOURCE_DIR} ${ABS_FIL})

  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  
  list(APPEND ALL_PROTO_SRCS "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.h")
  	
  add_custom_command(
    OUTPUT "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out ${PROJECT_INSTALL_DIR}  
    -I ${CARTO_SOURCE_DIR} ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT "Running C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
  )
  
 STRING(REGEX REPLACE "/" "_" NAME_DIR ${DIR})
  
 add_custom_target(
    ${NAME_DIR}_${FIL_WE}_generate_proto ALL
    DEPENDS "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.cc"
            "${PROJECT_INSTALL_DIR}/${DIR}/${FIL_WE}.pb.h"
    )
endforeach()

set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
set(ALL_PROTO_GEN ${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS})

## ************************************************************* COPYING ALL HEADERS TO THE INCLUDE FOLDER 
file(GLOB_RECURSE ALL_CARTO_HDRS "src/cartographer/*.h")
file(GLOB_RECURSE ALL_CARTO_GEN_MSGS "src/cartographer_generic_msgs/*.h")
file(GLOB_RECURSE ALL_CARTO_GEN_HDRS "src/cartographer_generic/*.h")
file(GLOB_RECURSE ALL_CARTO_SRCS "src/cartographer/*.cc")
file(GLOB_RECURSE ALL_CARTO_TESTS "src/cartographer/*_test.cc")
file(GLOB_RECURSE ALL_CARTO_EXES "src/cartographer/*_main.cc")
file(GLOB_RECURSE ALL_CARTO_PROB "src/cartographer/proto_stream.cc")
list(REMOVE_ITEM ALL_CARTO_SRCS ${ALL_CARTO_TESTS})
list(REMOVE_ITEM ALL_CARTO_SRCS ${ALL_CARTO_EXES})
list(REMOVE_ITEM ALL_CARTO_SRCS ${ALL_CARTO_PROB})
set(ALL_CARTO_HDRS ${ALL_CARTO_HDRS} ${ALL_CARTO_GEN_MSGS} ${ALL_CARTO_GEN_HDRS})

foreach(ABS_FIL ${ALL_CARTO_HDRS})
  file(RELATIVE_PATH REL_FIL ${CARTO_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  file(COPY ${ABS_FIL} DESTINATION ${PROJECT_INCLUDE_DIR}/${DIR})
endforeach()

configure_file(
  ${CARTO_SOURCE_DIR}/cartographer/common/config.h.cmake
  ${PROJECT_INCLUDE_DIR}/cartographer/common/config.h )
  
## ************************************************************* INCLUDING ALL THE HEADERS 
include_directories(build/install)
include_directories(include)
## ************************************************************* 

# TODO Nous devons recompiler les lib statiques glog et protobuf avec ces nouvelles modifications
file(GLOB_RECURSE ALL_CARTO_GEN_SRCS  "/opt/toolchain/sysroot/usr/include/glog/*"
                             "${PROTOBUF_HDR}/google/protobuf/message_lite*"
                             "${PROTOBUF_HDR}/google/protobuf/message.*"
                      
                             "src/cartographer_generic/node_main*" 
			                 "src/cartographer_generic/node_constants.*"
							 "src/cartographer_generic/sensor_bridge*"
							 "src/cartographer_generic/msg_conv*"
                             "src/cartographer_generic/tf_bridge*"
#                            "src/cartographer_generic/test*"                         
                             "src/cartographer_generic/node_options.*"
                             "src/cartographer_generic/trajectory_options.*"			
							 "src/cartographer_generic/map_builder_bridge.*"  
							 "src/cartographer_generic/node.*"
			     "src/cartographer_generic/buddy*"                                                         
                        																 ) 


add_library(${PROJECT_NAME} SHARED ${ALL_CARTO_GEN_SRCS} ${ALL_CARTO_SRCS} ${ALL_PROTO_GEN}) 

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${EIGEN_HDR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${CERES_HDR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${LUA_HDR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${BOOST_HDR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${PROTOBUF_HDR}")
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC "${GLOG_HDR}")

target_link_libraries(${PROJECT_NAME} lua)
target_link_libraries(${PROJECT_NAME} protobuf)
target_link_libraries(${PROJECT_NAME} boost)
target_link_libraries(${PROJECT_NAME} ceres)
target_link_libraries(${PROJECT_NAME} log)
