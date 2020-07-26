cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(ignition-transport-examples)

# Find the Ignition_Transport library
find_package(ignition-transport8 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(IGN_TRANSPORT_VER ${ignition-transport8_VERSION_MAJOR})

if (EXISTS "${CMAKE_SOURCE_DIR}/msgs/")
  # Message generation. Only required when using custom Protobuf messages.
  find_package(Protobuf REQUIRED)
  add_subdirectory(msgs)
  set_source_files_properties(${PROTO_SRC} ${PROTO_HEADER} PROPERTIES GENERATED TRUE)
endif()

include_directories(${CMAKE_BINARY_DIR})

# Generate examples
if (EXISTS "${CMAKE_SOURCE_DIR}/bench.cc")
  ########################################
  # GFlags
  find_library(gflags_LIBRARIES NAMES gflags)
  find_path(gflags_INCLUDE_DIRS gflags/gflags.h ENV CPATH)
  if (NOT gflags_LIBRARIES OR NOT gflags_INCLUDE_DIRS)
    message (STATUS "Unable to compile 'bench.cc', missing GFLags library")
  else()
    include_directories(SYSTEM ${gflags_INCLUDE_DIRS})
    add_executable(bench bench.cc)
    target_link_libraries(bench
      ignition-transport${IGN_TRANSPORT_VER}::core
      ${gflags_LIBRARIES}
      pthread)
  endif()
endif()

if (EXISTS "${CMAKE_SOURCE_DIR}/lidar_node.cc")
  add_executable(lidar_node lidar_node.cc)
  target_link_libraries(lidar_node ignition-transport${IGN_TRANSPORT_VER}::core)
endif()

if (MSVC)
  # Suppress Protobuf message generation warnings.
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4018 /wd4100 /wd4127 /wd4244 /wd4267 /wd4512")

  # Suppress the "decorated name length exceed" warning (inside the STL).
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /wd4503")

  add_definitions(-DBUILDING_STATIC_LIBS -DWIN32_LEAN_AND_MEAN)

  # Don't pull in the Windows min/max macros
  add_definitions(-DNOMINMAX)

endif()
