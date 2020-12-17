cmake_minimum_required(VERSION 3.0)

project(Bitctrl)
set( bitctrl_src_base ${CMAKE_CURRENT_SOURCE_DIR} )

include_directories(${bitctrl_src_base})
#include_directories(${bitctrl_src_base}/impl)

set(Eigen3_Include "" CACHE PATH "Directory containing Eigen/...")
include_directories(${Eigen3_Include})

#if (UNIX)
#	include_directories(/home/waterdrop/Library/GeographicLib/install-4.8/include)
#	add_library(libgeographic STATIC IMPORTED GLOBAL)
#	set_target_properties(libgeographic PROPERTIES
#		IMPORTED_LOCATION /home/waterdrop/Library/GeographicLib/install-4.8/lib/libGeographic.a
#	)
#endif()

set (GeographicLib_USE_STATIC_LIBS ON) 
find_package(GeographicLib REQUIRED COMPONENTS STATIC)
#find_package(Eigen3 REQUIRED)

set( LibBitctrl_withZMQ       OFF CACHE BOOL "ON or OFF" )
set( LibBitctrl_withLegacyExp OFF CACHE BOOL "If it's built with legacy experiments")
set( LibBitctrl_buildstatic   OFF CACHE BOOL "If a static version is also built")

set( LibBitctrl_withNetwork OFF CACHE BOOL "2020/08, bitctrl with custom tcp server" )


# 运行库
set(_bitctrl_base
	${bitctrl_src_base}/BIT.h
	${bitctrl_src_base}/impl/BIT.cpp
	${bitctrl_src_base}/impl/ssconfig.hpp
	${bitctrl_src_base}/impl/logconfig.h
	${bitctrl_src_base}/impl/helper.hpp
	${bitctrl_src_base}/impl/TraceInit.cpp
)
set(_bitctrl_midware
	${bitctrl_src_base}/impl/Action.hpp
	${bitctrl_src_base}/impl/Algo.hpp
	${bitctrl_src_base}/impl/Algo.cpp
	${bitctrl_src_base}/impl/Geotools.hpp
	${bitctrl_src_base}/impl/Geotools.cpp
	${bitctrl_src_base}/impl/Obstacles.hpp
	${bitctrl_src_base}/impl/Obstacles.cpp
	${bitctrl_src_base}/impl/astarplan.hpp
	${bitctrl_src_base}/impl/astarplan.cpp
	${bitctrl_src_base}/impl/toolbox/LoggingUtil.hpp
	${bitctrl_src_base}/impl/toolbox/EigenQP.hpp
	${bitctrl_src_base}/impl/toolbox/NSAvoid.hpp
	${bitctrl_src_base}/impl/toolbox/NSAvoidAlgo.hpp
	${bitctrl_src_base}/impl/toolbox/NSAvoidBound.hpp
	${bitctrl_src_base}/impl/toolbox/NSAvoidItem.hpp
	${bitctrl_src_base}/impl/toolbox/NSAvoidUtil.hpp
	${bitctrl_src_base}/impl/toolbox/CtrlBuffer.hpp
	${bitctrl_src_base}/impl/toolbox/CtrlBuffer.cpp
)
set(_bitctrl_impl
    ${bitctrl_src_base}/impl/Base.hpp 
	${bitctrl_src_base}/impl/Base.cpp
    ${bitctrl_src_base}/impl/BasicTest.hpp
    ${bitctrl_src_base}/impl/BasicTest.cpp
	${bitctrl_src_base}/impl/Combined.hpp
	${bitctrl_src_base}/impl/Combined_Init.cpp
    ${bitctrl_src_base}/impl/Combined_Commu.cpp
    ${bitctrl_src_base}/impl/Combined_Compute.cpp
    ${bitctrl_src_base}/impl/Combined_Function.cpp
	${bitctrl_src_base}/impl/tcpcom.hpp
	${bitctrl_src_base}/impl/tcpcom.cpp
	${bitctrl_src_base}/impl/Tasks.hpp
	${bitctrl_src_base}/impl/Tasks.cpp
)
set(_third_party
	${bitctrl_src_base}/impl/tkspline.h
	${bitctrl_src_base}/impl/tkspline.cpp
	${bitctrl_src_base}/impl/fmt/format.h
	${bitctrl_src_base}/impl/fmt/format.cc
)
set(_bitctrl_test8
	${bitctrl_src_base}/impl/test8/FormationTest.hpp
	${bitctrl_src_base}/impl/test8/FormationTest_Init.cpp
	${bitctrl_src_base}/impl/test8/FormationTest_Compute.cpp
)
#set(_quadprog
#	${bitctrl_src_base}/impl/quadprog/Array.hpp
#	${bitctrl_src_base}/impl/quadprog/Array.cpp
#	${bitctrl_src_base}/impl/quadprog/QuadProg++.hpp
#	${bitctrl_src_base}/impl/quadprog/QuadProg++.cpp
#)

set(bitctrl_src
	${_bitctrl_base}
	${_bitctrl_midware}
	${_bitctrl_impl}
	${_bitctrl_test8}
	${_third_party}
#	${_quadprog}
)
source_group(base FILES ${_bitctrl_base})
source_group(midware FILES ${_bitctrl_midware})
source_group(impl FILES ${_bitctrl_impl})
source_group(test8 FILES ${_bitctrl_test8})
source_group(3rd_party FILES ${_third_party})
#source_group(quadprog FILES ${_quadprog})

add_library(bitctrl  SHARED  ${bitctrl_src})
target_include_directories(bitctrl PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(bitctrl PRIVATE GeographicLib_STATIC)


target_compile_definitions(
    bitctrl
    PRIVATE _BITCTRL_EXPORT 
    PRIVATE _CRT_SECURE_NO_WARNINGS
    PRIVATE ENABLE_STDOUT_LOGGING
    PRIVATE DISABLE_DEBUG_LOGGING
    PRIVATE DISABLE_INFO_LOGGING
    #PRIVATE _HAS_REMOTE_TOOLS_
)
if (WIN32)
	#target_compile_options(bitctrl PRIVATE /wd4819)
endif()

if (LibBitctrl_withZMQ)
    target_compile_definitions( bitctrl PRIVATE _HAS_ZMQ_ )
    target_link_libraries(bitctrl libzmq)
    message(STATUS "libbitctrl is build with ZeroMQ")
endif()
if (!LibBitctrl_withLegacyExp)
   target_compile_definitions( bitctrl PRIVATE _NO_LEGACY_EXP )
   message(STATUS "libbitctrl is build without legacy experiments")
endif()
if (UNIX)
    target_compile_options(bitctrl PRIVATE -std=c++11 -fvisibility=hidden)
    message(STATUS "Using std=c++11")
endif()


if (LibBitctrl_buildstatic)
	add_library(bitctrl_s STATIC ${bitctrl_src})
	target_link_libraries(bitctrl_s  libgeographic)
	target_compile_definitions(
		bitctrl_s
		PRIVATE _BITCTRL_STATIC 
		PRIVATE _CRT_SECURE_NO_WARNINGS
		PRIVATE ENABLE_STDOUT_LOGGING
	)
	if (UNIX)
	    target_compile_options(bitctrl_s PRIVATE -std=c++11)
	endif()
endif()

# 2020/08/23
if (LibBitctrl_withNetwork)
	find_package(ssnet REQUIRED)
	target_link_libraries(bitctrl PRIVATE ssnet::ssnetwork)
	target_compile_definitions(bitctrl PRIVATE ENABLE_SSNETWORK)
	message(STATUS "libbitctrl is build with ssnetwork")
endif()


#add_definitions(-DDISABLE_LOGGING)
#add_definitions(-DDISABLE_DEBUG_LOGGING)
#set_target_properties(bitctrl PROPERTIES COMPILE_FLAGS -lang-c++)

# 测试程序
#add_executable(BIT_test test/test.cpp)
#target_link_libraries(BIT_test bitctrl)

add_executable(BIT_CTest ${bitctrl_src_base}/test/TestC.c)
target_link_libraries(BIT_CTest bitctrl)
if (UNIX)
	target_link_libraries(BIT_CTest m)
endif()

# 设置输出
#install(TARGETS bitctrl BIT_CTest
#		ARCHIVE DESTINATION $<CONFIG>
#		LIBRARY DESTINATION $<CONFIG>
#		RUNTIME DESTINATION $<CONFIG>
#)
#install(FILES BIT.h DESTINATION "./")

# 设置输出, 2020/10/20
#install(TARGETS bitctrl BIT_CTest
#	LIBRARY DESTINATION  ${CMAKE_INSTALL_LIBDIR}
#	ARCHIVE DESTINATION  ${CMAKE_INSTALL_LIBDIR}
#	RUNTIME DESTINATION  ${CMAKE_INSTALL_BINDIR}
#)
#install(TARGETS bitctrl
#	LIBRARY DESTINATION  lib
#	ARCHIVE DESTINATION  lib
#	RUNTIME DESTINATION  bin
#)
#install(FILES BIT.h DESTINATION "./")


# 设置Packagae输出 2020/10/26
# This Line Is Important
# It tells any app that links against mylib (installed) will include $<INSTALL_INTERFACE:include>
target_include_directories(bitctrl PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Not knowing the use of these, but it helps
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Defining the targets
set(INSTALL_TARGETS bitctrl)	

# Prepare a different name for debug
set_target_properties(${INSTALL_TARGETS} 
	PROPERTIES DEBUG_POSTFIX _d
)

# Defining Directories
set(targets_export_name bitctrl-targets)
set(MY_LIB_DIR   ${CMAKE_INSTALL_LIBDIR})
set(MY_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/bitctrl)
set(MY_INC_DIR   ${CMAKE_INSTALL_INCLUDEDIR})
set(MY_SRC_DIR   src/bitctrl)
set(MY_RUNTIME_DIR ${CMAKE_INSTALL_BINDIR})
set(MY_VERSION   1.0)
set(version_config ${PROJECT_BINARY_DIR}/bitctrl-config-version.cmake)
set(project_config ${PROJECT_BINARY_DIR}/bitctrl-config.cmake)

message(STATUS "MY_LIB_DIR is: "   ${MY_LIB_DIR})
message(STATUS "MY_CMAKE_DIR is: " ${MY_CMAKE_DIR})
message(STATUS "MY_INC_DIR is: "   ${MY_INC_DIR})
message(STATUS "MY_SRC_DIR is: "   ${MY_SRC_DIR})
message(STATUS "MY_RUNTIME_DIR is: "   ${MY_RUNTIME_DIR})

# Generate the version, config and target files into the build directory.
write_basic_package_version_file(
	${version_config}
	VERSION ${MY_VERSION}
	COMPATIBILITY AnyNewerVersion)
configure_file(
	"${PROJECT_SOURCE_DIR}/bitctrl.pc.in"
	"${pkgconfig}"
	@ONLY)
configure_package_config_file(
	${PROJECT_SOURCE_DIR}/bitctrl-config.cmake.in
	${project_config}
	INSTALL_DESTINATION ${MY_CMAKE_DIR})

export(TARGETS ${INSTALL_TARGETS} FILE ${PROJECT_BINARY_DIR}/${targets_export_name}.cmake)

# Install version, config and target files.
install(FILES ${project_config} ${version_config} DESTINATION ${MY_CMAKE_DIR})
install(EXPORT ${targets_export_name} DESTINATION ${MY_CMAKE_DIR})

# Install the library and headers.
install(TARGETS ${INSTALL_TARGETS} EXPORT ${targets_export_name}
	  LIBRARY DESTINATION ${MY_LIB_DIR}
	  ARCHIVE DESTINATION ${MY_LIB_DIR}
	  RUNTIME DESTINATION ${MY_RUNTIME_DIR})
if (WIN32)
	install(FILES $<TARGET_PDB_FILE:${INSTALL_TARGETS}> DESTINATION ${MY_LIB_DIR} OPTIONAL)
endif()

# Note!! Must Have the "/"
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/BIT.h DESTINATION ${MY_INC_DIR})
#install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${MY_SRC_DIR})




