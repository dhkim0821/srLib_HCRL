# Install script for directory: /Users/donghyunkim/new_srLib/srLib

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/donghyunkim/new_srLib/build/srLib/common/cmake_install.cmake")
  include("/Users/donghyunkim/new_srLib/build/srLib/LieGroup/cmake_install.cmake")
  include("/Users/donghyunkim/new_srLib/build/srLib/SceneGraph/cmake_install.cmake")
  include("/Users/donghyunkim/new_srLib/build/srLib/srDyn/cmake_install.cmake")
  include("/Users/donghyunkim/new_srLib/build/srLib/srExt/cmake_install.cmake")
  include("/Users/donghyunkim/new_srLib/build/srLib/srg/cmake_install.cmake")

endif()

