# Install script for directory: /home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/install-all")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/src/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/include/GeographicLib/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/tools/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/man/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/doc/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/js/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/matlab/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/python/geographiclib/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/examples/cmake_install.cmake")
  include("/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/cmake/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/lovezy/catkin_car/src/bitrobot/lib/repo-v0.0.4/cpp/build-/geo/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
