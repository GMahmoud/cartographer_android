# Install script for directory: /home/maghob/root/android_cartographer/app

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/package.xml")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer/" TYPE DIRECTORY FILES "/home/maghob/root/android_cartographer/app/configuration_files")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer/" TYPE DIRECTORY FILES "/home/maghob/root/android_cartographer/app/cmake")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/cartographer_autogenerate_ground_truth")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/home/maghob/Android/Sdk/ndk-bundle/toolchains/x86-4.9/prebuilt/linux-x86_64/bin/i686-linux-android-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_autogenerate_ground_truth")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE EXECUTABLE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/cartographer_compute_relations_metrics")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/home/maghob/Android/Sdk/ndk-bundle/toolchains/x86-4.9/prebuilt/linux-x86_64/bin/i686-linux-android-strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/bin/cartographer_compute_relations_metrics")
    endif()
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/libcartographer.a")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/.externalNativeBuild/cmake/release/arm64-v8a/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/arm64-v8a/src/main/cpp/cartographer/common/config.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/.externalNativeBuild/cmake/release/armeabi-v7a/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/armeabi-v7a/src/main/cpp/cartographer/common/config.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/.externalNativeBuild/cmake/release/armeabi/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/armeabi/src/main/cpp/cartographer/common/config.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/common/config.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/blocking_queue.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/ceres_solver_options.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/configuration_file_resolver.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/fixed_ratio_sampler.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/histogram.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/lua.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/lua_parameter_dictionary.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/lua_parameter_dictionary_test_helpers.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/make_unique.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/math.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/mutex.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/port.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/rate_timer.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/thread_pool.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/common/time.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/ground_truth" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/ground_truth/relations_text_file.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/coloring_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/counting_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/file_writer.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/fixed_ratio_sampling_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/hybrid_grid_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/image.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/intensity_to_color_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/min_max_range_filtering_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/null_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/outlier_removing_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/pcd_writing_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/ply_writing_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/points_batch.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/points_processor_pipeline_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/probability_grid_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/proto_stream.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/xray_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/io" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/io/xyz_writing_points_processor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/collated_trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/detect_floors.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/global_trajectory_builder_interface.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/id.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/imu_tracker.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/map_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/odometry_state_tracker.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/pose_graph_trimmer.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/probability_values.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/sparse_pose_graph.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/sparse_pose_graph/constraint_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/sparse_pose_graph/optimization_problem_options.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/submaps.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/trajectory_connectivity.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping/trajectory_node.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/global_trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/local_trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/local_trajectory_builder_options.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/map_limits.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/probability_grid.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/range_data_inserter.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/ray_casting.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/ceres_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/correlative_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/fast_correlative_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/fast_global_localizer.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/occupied_space_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/real_time_correlative_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/rotation_delta_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/scan_matching/translation_delta_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph/constraint_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph/optimization_problem.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/sparse_pose_graph/spa_cost_function.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/submaps.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_2d/xy_index.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/acceleration_cost_function.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/ceres_pose.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/global_trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/hybrid_grid.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/imu_integration.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/local_trajectory_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/local_trajectory_builder_options.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/motion_filter.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/range_data_inserter.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/rotation_cost_function.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/ceres_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/fast_correlative_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/interpolated_grid.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/occupied_space_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/precomputation_grid.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/real_time_correlative_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/rotation_delta_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/rotational_scan_matcher.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/scan_matching/translation_delta_cost_functor.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph/constraint_builder.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph/optimization_problem.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/sparse_pose_graph/spa_cost_function.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/submaps.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/mapping_3d/translation_cost_function.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/collator.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/compressed_point_cloud.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/data.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/imu_data.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/ordered_multi_queue.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/point_cloud.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/range_data.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/sensor/voxel_filter.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/transform" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/transform/rigid_transform.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/transform" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/transform/rigid_transform_test_helpers.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/transform" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/transform/transform.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/transform" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/src/main/cpp/cartographer/transform/transform_interpolation_buffer.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/common/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/common/proto/ceres_solver_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/ground_truth/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/ground_truth/proto/relations.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/map_builder_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/serialization.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/sparse_pose_graph.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/sparse_pose_graph_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/submap.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/submap_visualization.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/trajectory.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/trajectory_builder_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/proto/trajectory_connectivity.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/sparse_pose_graph/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/sparse_pose_graph/proto/constraint_builder_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping/sparse_pose_graph/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping/sparse_pose_graph/proto/optimization_problem_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/cell_limits.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/local_trajectory_builder_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/map_limits.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/probability_grid.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/range_data_inserter_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/proto/submaps_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/scan_matching/proto/ceres_scan_matcher_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_2d/scan_matching/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_2d/scan_matching/proto/real_time_correlative_scan_matcher_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/proto/hybrid_grid.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/proto/local_trajectory_builder_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/proto/motion_filter_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/proto/range_data_inserter_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/proto/submaps_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/scan_matching/proto/ceres_scan_matcher_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/mapping_3d/scan_matching/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/mapping_3d/scan_matching/proto/fast_correlative_scan_matcher_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/sensor/proto/adaptive_voxel_filter_options.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/sensor/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/sensor/proto/sensor.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/src/main/cpp/cartographer/transform/proto" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/src/main/cpp/cartographer/transform/proto/transform.pb.h")
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake/CartographerTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake/CartographerTargets.cmake"
         "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/CMakeFiles/Export/share/cartographer/cmake/CartographerTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake/CartographerTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake/CartographerTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/CMakeFiles/Export/share/cartographer/cmake/CartographerTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer/cmake" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/CMakeFiles/Export/share/cartographer/cmake/CartographerTargets-release.cmake")
  endif()
endif()

if("${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/cartographer" TYPE FILE FILES "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/cmake/cartographer/cartographer-config.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/maghob/root/android_cartographer/app/.externalNativeBuild/cmake/release/x86/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
