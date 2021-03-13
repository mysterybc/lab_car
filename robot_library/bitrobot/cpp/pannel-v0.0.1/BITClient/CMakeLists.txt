cmake_minimum_required(VERSION 3.5)

project(BITClient LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# QtCreator supports the following variables for Android, which are identical to qmake Android variables.
# Check http://doc.qt.io/qt-5/deployment-android.html for more information.
# They need to be set before the find_package(Qt5 ...) call.

#if(ANDROID)
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#    if (ANDROID_ABI STREQUAL "armeabi-v7a")
#        set(ANDROID_EXTRA_LIBS
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libcrypto.so
#            ${CMAKE_CURRENT_SOURCE_DIR}/path/to/libssl.so)
#    endif()
#endif()

find_package(Qt5 COMPONENTS Gui Core Widgets PrintSupport REQUIRED)
find_package(ssnet REQUIRED)
find_package(Eigen3 REQUIRED)

set(basic_files
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    dialogstatus.h
    dialogstatus.cpp
    dialogstatus.ui
    dialogtask.h
    dialogtask.cpp
    dialogtask.ui
    gamepad.hpp
    ssconfig.hpp
    sscmd.hpp
    memfile.cpp
    graphcontrol.hpp
    mapitems.hpp
    mapitems.cpp
    wrapstat.h
)
set(items_plots
    qcustomplot.h
    qcustomplot.cpp
    items/basicitemex.cpp
    items/basicitemex.hpp
    items/compounditem.hpp
    items/mycolor.hpp
    items/robotitem.cpp
    items/robotitem.hpp
    items/util.hpp
)
set(task_files
    tasks/tracking.hpp
    tasks/obstacle.hpp
    tasks/taskbase.hpp
    tasks/winfunction.hpp
    tasks/formation.hpp
)
set(mapinfo_files
    mapinfo/Obstacles.hpp
    mapinfo/Obstacles.cpp
)

if(ANDROID)
  add_library(BITClient SHARED
    ${basic_files}
    ${items_plots}
    ${task_files}
	${mapinfo_files}
  )
else()
  add_executable(BITClient
    ${basic_files}
    ${items_plots}
    ${task_files}
	${mapinfo_files}
  )
endif()

target_link_libraries(BITClient PRIVATE Qt5::Widgets Qt5::PrintSupport Qt5::Core Qt5::Gui)
target_link_libraries(BITClient PRIVATE ssnet::ssnetwork Eigen3::Eigen)
if(WIN32)
target_link_libraries(BITClient PRIVATE Winmm)
target_compile_options(BITClient PRIVATE /utf-8)
endif()

# Install the library and headers.
set(INSTALL_TARGETS BITClient)
set(MY_LIB_DIR     ${CMAKE_INSTALL_LIBDIR})
set(MY_RUNTIME_DIR ${CMAKE_INSTALL_BINDIR})
set_target_properties(${INSTALL_TARGETS}
        PROPERTIES DEBUG_POSTFIX _d
)
install(TARGETS ${INSTALL_TARGETS}
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin)
#install(FILES $<TARGET_PDB_FILE:${INSTALL_TARGETS}> DESTINATION ${MY_LIB_DIR} OPTIONAL)
