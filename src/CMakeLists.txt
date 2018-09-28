find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres 1.14 REQUIRED)

set(LibHandEye_HEADERS
    handeye.hpp
)

add_library(LibHandEye STATIC
    ${LibHandEye_HEADERS}
    handeye.cpp
    EigenUtils.h
    calib/DualQuaternion.h
    calib/HandEyeCalibration.cc
    calib/HandEyeCalibration.h
    calib/QuaternionMapping.h
)

set_target_properties(LibHandEye PROPERTIES DEBUG_POSTFIX "-d")
set_target_properties(LibHandEye PROPERTIES PUBLIC_HEADER ${LibHandEye_HEADERS})

# Add Eigen reference
target_link_libraries(LibHandEye PUBLIC Eigen3::Eigen)

# Add Ceres reference
target_include_directories(LibHandEye PRIVATE ${CERES_INCLUDE_DIRS})
target_link_libraries(LibHandEye PRIVATE ${CERES_LIBRARIES})


# -------------------------------------------------------
# INSTALL -----------------------------------------------
# -------------------------------------------------------

# Install output files and 
install(TARGETS LibHandEye EXPORT handeye-targets
    RUNTIME DESTINATION         bin
    LIBRARY DESTINATION         lib
    ARCHIVE DESTINATION         lib
    INCLUDES DESTINATION        include
    PUBLIC_HEADER DESTINATION   include/handeye
)

install(EXPORT handeye-targets
    FILE handeye-targets.cmake
    NAMESPACE handeye::
    DESTINATION lib/cmake/handeye
)

# Setup CMake package
write_basic_package_version_file(
    handeye-config-version.cmake
    VERSION ${LibHandEye_VERSION}
    COMPATIBILITY SameMinorVersion
)

configure_package_config_file(
    ../cmake/handeye-config.cmake.in
    handeye-config.cmake
    INSTALL_DESTINATION lib/cmake/handeye
    NO_CHECK_REQUIRED_COMPONENTS_MACRO
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/handeye-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/handeye-config-version.cmake
    DESTINATION lib/cmake/handeye
)
