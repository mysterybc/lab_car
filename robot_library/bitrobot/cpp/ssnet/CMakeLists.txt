cmake_minimum_required(VERSION 3.12)

# Define a project name
project(ssnet)
add_compile_options(-std=c++11)

# Common Practice, put bulid outputs together
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(file_rand_rw 
	mersenne/mersenne.h
	mersenne/mersenne.cpp
)
set(file_sstimer
	sstimer/sstimer.hpp
	sstimer/sstimer_win.cpp
	sstimer/sstimer_linux.cpp
	sstimer/sstimer_test.cpp
)
set(file_sstcp
	sstcp.hpp
	sstcp_api.hpp
    sstcp_win.cpp
    sstcp_bsd.cpp
	sstcp_common.cpp
    sstcp_test.cpp
)
set(file_protocol
	protocol/msgparser.hpp
	protocol/msgparser_binary.cpp
	protocol/msgparser_json.cpp
	protocol/robotmsg.hpp
	protocol/robotmsg_test.cpp
	protocol/robotmsg_test_data.cpp
)
set(file_proto
	proto/proto_base.hpp
	proto/proto_bit.hpp
	proto/proto_bit.cpp
	proto/proto_bit_test.cpp
	proto/proto_util.hpp
	proto/proto_util.cpp
)
set(file_sstcp_bit
	${file_proto}
	sstcp_protobit.hpp sstcp_protobit.cpp
	sstcp_protobit_test.cpp
)
set(file_sstcp_protocol
	${file_sstcp}
	${file_protocol}
	sstcp_proto.hpp
	sstcp_proto.cpp
)

add_executable(sstcp_test
	${file_sstcp}
	${file_rand_rw}
	sstcp_test_main.cpp
)
target_compile_definitions(sstcp_test PRIVATE BUILD_SSTCP_TEST _CRT_SECURE_NO_WARNINGS)

add_executable(sstimer_test
	${file_sstimer}
	sstimer/sstimer_test_main.cpp
)
target_compile_definitions(sstimer_test PRIVATE BUILD_SSTIMER_TEST _CRT_SECURE_NO_WARNINGS)

add_executable(protocol_test
	${file_protocol}
	${file_rand_rw}
	protocol/robotmsg_test_main.cpp
)
target_compile_definitions(protocol_test PRIVATE BUILD_ROBOTMSG_TEST _CRT_SECURE_NO_WARNINGS)

add_executable(mersenne_test
	${file_rand_rw}
	mersenne/mersenne_test.cpp
)
target_compile_definitions(mersenne_test PRIVATE BUILD_MERSENNE_TEST _CRT_SECURE_NO_WARNINGS)

add_executable(sstcp_proto_test
	${file_rand_rw}
	${file_sstcp_protocol}
	${file_sstimer}
	sstcp_test_proto_main.cpp
)
target_compile_definitions(sstcp_proto_test PRIVATE BUILD_SSTCP_TEST_PROTO _CRT_SECURE_NO_WARNINGS)

add_executable(protobit_test ${file_proto})
target_compile_definitions(protobit_test PRIVATE BUILD_PROTO_BIT_TEST _CRT_SECURE_NO_WARNINGS)

add_executable(sstcp_bit_test 
    ${file_proto} ${file_sstcp} ${file_rand_rw} ${file_sstimer}
	${file_sstcp_bit}
)
target_compile_definitions(sstcp_bit_test PRIVATE BUILD_SSTCP_PROTOBIT_TEST _CRT_SECURE_NO_WARNINGS)

add_library(ssnetwork STATIC 
	${file_sstcp_protocol} ${file_rand_rw} ${file_sstimer}
	${file_sstcp_bit}
)
# This Line Is Important
# It tells any app that links against mylib (installed) will include $<INSTALL_INTERFACE:include>
target_include_directories(ssnetwork PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>)

# Not knowing the use of these, but it helps
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Defining the targets
set(INSTALL_TARGETS 
	ssnetwork 
	sstcp_bit_test   protobit_test 
	sstcp_proto_test mersenne_test
	protocol_test    sstcp_test)	

# Prepare a different name for debug
set_target_properties(${INSTALL_TARGETS}
	PROPERTIES DEBUG_POSTFIX _d
)

# Defining Directories
set(targets_export_name ssnet-targets)
set(MY_LIB_DIR   ${CMAKE_INSTALL_LIBDIR})
set(MY_CMAKE_DIR ${CMAKE_INSTALL_LIBDIR}/cmake/ssnet)
set(MY_INC_DIR   ${CMAKE_INSTALL_INCLUDEDIR}/ssnet)
set(MY_SRC_DIR   src/ssnet)
set(MY_RUNTIME_DIR ${CMAKE_INSTALL_BINDIR})
set(MY_VERSION   1.0)
set(version_config ${PROJECT_BINARY_DIR}/ssnet-config-version.cmake)
set(project_config ${PROJECT_BINARY_DIR}/ssnet-config.cmake)

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
	"${PROJECT_SOURCE_DIR}/ssnet.pc.in"
	"${pkgconfig}"
	@ONLY)
configure_package_config_file(
	${PROJECT_SOURCE_DIR}/ssnet-config.cmake.in
	${project_config}
	INSTALL_DESTINATION ${MY_CMAKE_DIR})

export(TARGETS ${INSTALL_TARGETS} NAMESPACE ssnet::
         FILE ${PROJECT_BINARY_DIR}/${targets_export_name}.cmake)

# Install version, config and target files.
install(FILES ${project_config} ${version_config}
    DESTINATION ${MY_CMAKE_DIR})
install(EXPORT ${targets_export_name} DESTINATION ${MY_CMAKE_DIR}
	  NAMESPACE ssnet::)

# Install the library and headers.
install(TARGETS ${INSTALL_TARGETS} EXPORT ${targets_export_name}
	  LIBRARY DESTINATION ${MY_LIB_DIR}
	  ARCHIVE DESTINATION ${MY_LIB_DIR}
	  RUNTIME DESTINATION ${MY_RUNTIME_DIR})
install(FILES $<TARGET_PDB_FILE:${INSTALL_TARGETS}> DESTINATION ${MY_LIB_DIR} OPTIONAL)

# Note!! Must Have the "/"
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${MY_INC_DIR}
	    FILES_MATCHING PATTERN "*.h*")
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ DESTINATION ${MY_SRC_DIR})




