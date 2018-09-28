cmake_minimum_required(VERSION 3.12)

include(CPack)

set(LibHandEye_VERSION 1.0)
project(handeye VERSION ${LibHandEye_VERSION})

include (CMakePackageConfigHelpers)

add_subdirectory(src)
