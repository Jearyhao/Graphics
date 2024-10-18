# Install script for directory: D:/Desktop/homework2/code/CGL/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "D:/Desktop/homework2/code/out/install/x64-Release")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "D:/Desktop/homework2/code/out/build/x64-Release/CGL/src/CGL.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("D:/Desktop/homework2/code/out/build/x64-Release/CGL/src/CMakeFiles/CGL.dir/install-cxx-module-bmi-RelWithDebInfo.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/CGL" TYPE FILE FILES
    "D:/Desktop/homework2/code/CGL/src/CGL.h"
    "D:/Desktop/homework2/code/CGL/src/vector2D.h"
    "D:/Desktop/homework2/code/CGL/src/vector3D.h"
    "D:/Desktop/homework2/code/CGL/src/vector4D.h"
    "D:/Desktop/homework2/code/CGL/src/matrix3x3.h"
    "D:/Desktop/homework2/code/CGL/src/matrix4x4.h"
    "D:/Desktop/homework2/code/CGL/src/quaternion.h"
    "D:/Desktop/homework2/code/CGL/src/complex.h"
    "D:/Desktop/homework2/code/CGL/src/color.h"
    "D:/Desktop/homework2/code/CGL/src/osdtext.h"
    "D:/Desktop/homework2/code/CGL/src/viewer.h"
    "D:/Desktop/homework2/code/CGL/src/base64.h"
    "D:/Desktop/homework2/code/CGL/src/tinyxml2.h"
    "D:/Desktop/homework2/code/CGL/src/renderer.h"
    )
endif()

