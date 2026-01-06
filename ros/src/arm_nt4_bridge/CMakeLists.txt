cmake_minimum_required(VERSION 3.0.2)
project(arm_nt4_bridge)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  control_msgs
)

catkin_package()

include(ExternalProject)
include(GNUInstallDirs)

set(WPILIB_MAVEN_BASE_URL "https://frcmaven.wpi.edu/artifactory/release" CACHE STRING "WPILib Maven base URL")
# IMPORTANT (Jetson/Ubuntu20.04/glibc2.31):
# WPILib 2025/2026 linuxarm64 prebuilt .so require glibc>=2.32/2.34.
# Pin to a glibc2.31-compatible version by default. 2024.3.2 works on Ubuntu20.04
# and is newer than 2023, improving NT4 behavior while still avoiding glibc>=2.32/2.34.
set(WPILIB_VERSION "2024.3.2" CACHE STRING "WPILib version (override only if your runtime glibc supports it)")

function(_fetch_maven_release out_var group_path artifact)
  set(meta_url "${WPILIB_MAVEN_BASE_URL}/${group_path}/${artifact}/maven-metadata.xml")
  set(meta_file "${CMAKE_CURRENT_BINARY_DIR}/${artifact}-maven-metadata.xml")
  file(DOWNLOAD "${meta_url}" "${meta_file}" STATUS st SHOW_PROGRESS)
  list(GET st 0 code)
  if(NOT code EQUAL 0)
    message(WARNING "Failed to download ${meta_url} (code=${code}), falling back to 2023.4.3")
    set(${out_var} "2023.4.3" PARENT_SCOPE)
    return()
  endif()
  file(READ "${meta_file}" meta_txt)
  string(REGEX MATCH "<release>([^<]+)</release>" _m1 "${meta_txt}")
  if(CMAKE_MATCH_1)
    set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    return()
  endif()
  string(REGEX MATCH "<latest>([^<]+)</latest>" _m2 "${meta_txt}")
  if(CMAKE_MATCH_1)
    set(${out_var} "${CMAKE_MATCH_1}" PARENT_SCOPE)
    return()
  endif()
  message(WARNING "Could not parse release/latest from ${meta_url}, falling back to 2023.4.3")
  set(${out_var} "2023.4.3" PARENT_SCOPE)
endfunction()

if(WPILIB_VERSION STREQUAL "")
  message(WARNING "WPILIB_VERSION cache was empty; forcing to 2024.3.2 for glibc2.31 compatibility")
  set(WPILIB_VERSION "2024.3.2" CACHE STRING "WPILib version (override only if your runtime glibc supports it)" FORCE)
endif()

message(STATUS "Using WPILib version: ${WPILIB_VERSION}")

set(_DL_DIR "${CMAKE_CURRENT_BINARY_DIR}/wpilib_dl")
set(_EXT_DIR "${CMAKE_CURRENT_BINARY_DIR}/wpilib_ext")
file(MAKE_DIRECTORY "${_DL_DIR}")
file(MAKE_DIRECTORY "${_EXT_DIR}")

function(_dl_zip name url out_path)
  set(dst "${_DL_DIR}/${name}.zip")
  file(DOWNLOAD "${url}" "${dst}" STATUS st SHOW_PROGRESS)
  list(GET st 0 code)
  if(NOT code EQUAL 0)
    list(GET st 1 msg)
    message(FATAL_ERROR "Download failed: ${url} -> ${dst} (code=${code}) msg=${msg}")
  endif()
  set(${out_path} "${dst}" PARENT_SCOPE)
endfunction()

function(_extract_zip zip_path out_dir)
  file(MAKE_DIRECTORY "${out_dir}")
  execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf "${zip_path}"
                  WORKING_DIRECTORY "${out_dir}"
                  RESULT_VARIABLE rv)
  if(NOT rv EQUAL 0)
    # cmake -E tar xzf doesn't always like .zip on older cmake; try unzip
    find_program(UNZIP_EXE unzip)
    if(NOT UNZIP_EXE)
      message(FATAL_ERROR "Failed to extract ${zip_path} (cmake tar rv=${rv}) and unzip not found")
    endif()
    execute_process(COMMAND ${UNZIP_EXE} -q "${zip_path}" -d "${out_dir}" RESULT_VARIABLE rv2)
    if(NOT rv2 EQUAL 0)
      message(FATAL_ERROR "Failed to extract ${zip_path} with unzip (rv=${rv2})")
    endif()
  endif()
endfunction()

# Download zips
_dl_zip("ntcore-linuxarm64"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/ntcore/ntcore-cpp/${WPILIB_VERSION}/ntcore-cpp-${WPILIB_VERSION}-linuxarm64.zip"
  NTCORE_ZIP)
_dl_zip("ntcore-headers"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/ntcore/ntcore-cpp/${WPILIB_VERSION}/ntcore-cpp-${WPILIB_VERSION}-headers.zip"
  NTCORE_HEADERS_ZIP)

_dl_zip("wpinet-linuxarm64"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/wpinet/wpinet-cpp/${WPILIB_VERSION}/wpinet-cpp-${WPILIB_VERSION}-linuxarm64.zip"
  WPINET_ZIP)
_dl_zip("wpinet-headers"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/wpinet/wpinet-cpp/${WPILIB_VERSION}/wpinet-cpp-${WPILIB_VERSION}-headers.zip"
  WPINET_HEADERS_ZIP)

_dl_zip("wpiutil-linuxarm64"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/wpiutil/wpiutil-cpp/${WPILIB_VERSION}/wpiutil-cpp-${WPILIB_VERSION}-linuxarm64.zip"
  WPIUTIL_ZIP)
_dl_zip("wpiutil-headers"
  "${WPILIB_MAVEN_BASE_URL}/edu/wpi/first/wpiutil/wpiutil-cpp/${WPILIB_VERSION}/wpiutil-cpp-${WPILIB_VERSION}-headers.zip"
  WPIUTIL_HEADERS_ZIP)

# Extract
set(NTCORE_EXT_DIR "${_EXT_DIR}/ntcore")
set(WPINET_EXT_DIR "${_EXT_DIR}/wpinet")
set(WPIUTIL_EXT_DIR "${_EXT_DIR}/wpiutil")
set(NTCORE_HEADERS_DIR "${_EXT_DIR}/ntcore_headers")
set(WPINET_HEADERS_DIR "${_EXT_DIR}/wpinet_headers")
set(WPIUTIL_HEADERS_DIR "${_EXT_DIR}/wpiutil_headers")

_extract_zip("${NTCORE_ZIP}" "${NTCORE_EXT_DIR}")
_extract_zip("${WPINET_ZIP}" "${WPINET_EXT_DIR}")
_extract_zip("${WPIUTIL_ZIP}" "${WPIUTIL_EXT_DIR}")
_extract_zip("${NTCORE_HEADERS_ZIP}" "${NTCORE_HEADERS_DIR}")
_extract_zip("${WPINET_HEADERS_ZIP}" "${WPINET_HEADERS_DIR}")
_extract_zip("${WPIUTIL_HEADERS_ZIP}" "${WPIUTIL_HEADERS_DIR}")

# Heuristic include/lib locations (WPILib zips generally contain include/ and lib/)
set(WPILIB_INCLUDE_DIRS
  "${NTCORE_HEADERS_DIR}"
  "${WPINET_HEADERS_DIR}"
  "${WPIUTIL_HEADERS_DIR}"
)

set(WPILIB_LIB_DIRS
  "${NTCORE_EXT_DIR}/linux/arm64/shared"
  "${WPINET_EXT_DIR}/linux/arm64/shared"
  "${WPIUTIL_EXT_DIR}/linux/arm64/shared"
)

foreach(d IN LISTS WPILIB_INCLUDE_DIRS)
  if(NOT EXISTS "${d}")
    message(WARNING "WPILib include dir not found: ${d} (zip layout may differ)")
  endif()
endforeach()

foreach(d IN LISTS WPILIB_LIB_DIRS)
  if(NOT EXISTS "${d}")
    message(WARNING "WPILib lib dir not found: ${d} (zip layout may differ)")
  endif()
endforeach()

link_directories(${WPILIB_LIB_DIRS})

add_executable(arm_nt4_bridge_node
  src/arm_nt4_bridge_node.cpp
)
target_compile_options(arm_nt4_bridge_node PRIVATE -Wall -Wextra)
target_include_directories(arm_nt4_bridge_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${WPILIB_INCLUDE_DIRS}
)
target_link_libraries(arm_nt4_bridge_node
  ${catkin_LIBRARIES}
  # Force these to remain direct dependencies (avoid --as-needed dropping them),
  # otherwise libntcore's transitive deps won't be found via RUNPATH.
  -Wl,--no-as-needed
  ntcore
  wpinet
  wpiutil
  -Wl,--as-needed
)

# Make runtime find the downloaded .so without setting LD_LIBRARY_PATH
set_target_properties(arm_nt4_bridge_node PROPERTIES
  BUILD_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
  INSTALL_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
)

install(TARGETS arm_nt4_bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/nt4_monitor.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(nt4_seq_monitor
  src/nt4_seq_monitor.cpp
)
target_include_directories(nt4_seq_monitor PRIVATE
  ${WPILIB_INCLUDE_DIRS}
)
target_link_libraries(nt4_seq_monitor
  -Wl,--no-as-needed
  ntcore
  wpinet
  wpiutil
  -Wl,--as-needed
)
set_target_properties(nt4_seq_monitor PROPERTIES
  BUILD_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
  INSTALL_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
)

add_executable(servo_cmd_pub
  src/servo_cmd_pub.cpp
)
target_include_directories(servo_cmd_pub PRIVATE
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(servo_cmd_pub
  ${catkin_LIBRARIES}
)

add_executable(nt4_dump_arm
  src/nt4_dump_arm.cpp
)
target_include_directories(nt4_dump_arm PRIVATE
  ${WPILIB_INCLUDE_DIRS}
)
target_link_libraries(nt4_dump_arm
  -Wl,--no-as-needed
  ntcore
  wpinet
  wpiutil
  -Wl,--as-needed
)
set_target_properties(nt4_dump_arm PROPERTIES
  BUILD_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
  INSTALL_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
)

add_executable(nt4_rate_check
  src/nt4_rate_check.cpp
)
target_include_directories(nt4_rate_check PRIVATE
  ${WPILIB_INCLUDE_DIRS}
)
target_link_libraries(nt4_rate_check
  -Wl,--no-as-needed
  ntcore
  wpinet
  wpiutil
  -Wl,--as-needed
)
set_target_properties(nt4_rate_check PROPERTIES
  BUILD_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
  INSTALL_RPATH "${NTCORE_EXT_DIR}/linux/arm64/shared;${WPINET_EXT_DIR}/linux/arm64/shared;${WPIUTIL_EXT_DIR}/linux/arm64/shared"
)


