# Install script for directory: /home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/Bitctrl/GeographicLib-1.46/cmake

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib" TYPE FILE RENAME "geographiclib-config.cmake" FILES "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/cmake/project-config.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib" TYPE FILE RENAME "geographiclib-config-version.cmake" FILES "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/cmake/project-config-version.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib/geographiclib-targets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib/geographiclib-targets.cmake"
         "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/cmake/CMakeFiles/Export/lib/cmake/GeographicLib/geographiclib-targets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib/geographiclib-targets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib/geographiclib-targets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib" TYPE FILE FILES "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/cmake/CMakeFiles/Export/lib/cmake/GeographicLib/geographiclib-targets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/GeographicLib" TYPE FILE FILES "/home/lovebc/Desktop/workspace/ros/lab_car_ws/src/bitrobot/lib/repo-v0.0.4/cpp/build-Debug/geo/cmake/CMakeFiles/Export/lib/cmake/GeographicLib/geographiclib-targets-debug.cmake")
  endif()
endif()

