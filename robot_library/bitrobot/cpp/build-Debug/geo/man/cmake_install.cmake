# Install script for directory: /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/man

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/install-all")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man1" TYPE FILE FILES
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/CartConvert.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/ConicProj.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/GeodesicProj.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/GeoConvert.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/GeodSolve.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/GeoidEval.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/Gravity.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/MagneticField.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/Planimeter.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/RhumbSolve.1"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/TransverseMercatorProj.1"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/man/man8" TYPE FILE FILES
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/geographiclib-get-geoids.8"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/geographiclib-get-gravity.8"
    "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/man/geographiclib-get-magnetic.8"
    )
endif()

