cmake_minimum_required(VERSION 3.5)

project(BITServer LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

#set(CMAKE_CXX_STANDARD 11)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(ssnet REQUIRED)

set(basic_files
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    memfile.cpp
    gamepad.hpp
    ssconfig.hpp
	sscmd.hpp
    dialogdetail.ui
    dialogdetail.h
    dialogdetail.cpp
)


add_executable(BITServer
   ${basic_files}
)


target_link_libraries(BITServer PRIVATE Qt5::Widgets ssnet::ssnetwork)
if (WIN32)
target_compile_options(BITServer PRIVATE /utf-8)
target_link_libraries(BITServer PRIVATE Winmm)
endif()

# Install the library and headers.
set(INSTALL_TARGETS BITServer)
set(MY_LIB_DIR     ${CMAKE_INSTALL_LIBDIR})
set(MY_RUNTIME_DIR ${CMAKE_INSTALL_BINDIR})
set_target_properties(${INSTALL_TARGETS}
        PROPERTIES DEBUG_POSTFIX _d
)
install(TARGETS ${INSTALL_TARGETS}
          LIBRARY DESTINATION lib
          ARCHIVE DESTINATION lib
          RUNTIME DESTINATION bin
)
#install(FILES $<TARGET_PDB_FILE:${INSTALL_TARGETS}> DESTINATION ${MY_LIB_DIR} OPTIONAL)

