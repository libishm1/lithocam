# Python module `_lithocam` from the rosetta-generated, reflection-free nanobind source (generated/auto_nanobind.cpp).
#
# Build anywhere with an ordinary C++20 compiler (MSVC, GCC, Clang):
#   cmake -S bindings -B build_py -DPython_EXECUTABLE=<python with nanobind>
#   cmake --build build_py --config Release
#
# Regenerate generated/ after an API change on a host with the clang-p2996 fork (see README.md).
cmake_minimum_required(VERSION 3.18)
project(lithocam_python CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)
execute_process(COMMAND ${Python_EXECUTABLE} -c "import nanobind; print(nanobind.cmake_dir())"
                OUTPUT_VARIABLE nanobind_DIR OUTPUT_STRIP_TRAILING_WHITESPACE)
find_package(nanobind CONFIG REQUIRED)

set(LITHOCAM_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)
nanobind_add_module(_lithocam NB_STATIC
  generated/auto_nanobind.cpp
  ${LITHOCAM_ROOT}/src/scallop.cpp
  ${LITHOCAM_ROOT}/src/kerf.cpp
  ${LITHOCAM_ROOT}/src/dropcutter.cpp
  ${LITHOCAM_ROOT}/src/pathdrop.cpp)
target_include_directories(_lithocam PRIVATE ${LITHOCAM_ROOT}/include)
install(TARGETS _lithocam LIBRARY DESTINATION . RUNTIME DESTINATION .)
