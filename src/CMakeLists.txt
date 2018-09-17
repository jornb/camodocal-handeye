cmake_minimum_required(VERSION 3.12)
project(handeye VERSION ${HANDEYE_VERSION})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres 1.14 REQUIRED)

set(libhandeye_HEADERS "handeye.hpp")

add_library(libhandeye STATIC
    handeye.cpp
    handeye.hpp
    EigenUtils.h
    calib/DualQuaternion.h
    calib/HandEyeCalibration.cc
    calib/HandEyeCalibration.h
    calib/QuaternionMapping.h
)

set_target_properties(libhandeye PROPERTIES PUBLIC_HEADER "handeye.hpp")

# Add Eigen reference
target_link_libraries(libhandeye PUBLIC Eigen3::Eigen)

# Add Ceres reference
target_include_directories(libhandeye PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(libhandeye PRIVATE ${CERES_LIBRARIES})



# Setup CMake package
include (CMakePackageConfigHelpers)

set(TARGETS_EXPORT_NAME "${PROJECT_NAME}-targets")
configure_package_config_file(../cmake/handeye-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/handeye-config.cmake
  INSTALL_DESTINATION cmake
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file (handeye-config-version.cmake VERSION ${HANDEYE_VERSION} COMPATIBILITY SameMinorVersion)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/handeye-config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/handeye-config-version.cmake
  DESTINATION cmake)

install(TARGETS libhandeye
    EXPORT "${TARGETS_EXPORT_NAME}"
    RUNTIME DESTINATION         bin
    LIBRARY DESTINATION         lib
    ARCHIVE DESTINATION         lib
    INCLUDES DESTINATION        include
    PUBLIC_HEADER DESTINATION   include/handeye
)

# Config
# TODO: Specify NAMESPACE "handeye::"
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "cmake"
)
