cmake_minimum_required(VERSION 3.10)

#find_package(ssnet REQUIRED)

set(project_files
	simu_mas.hpp
	utility.hpp
	utility.cpp
	ssconfig.hpp
	sscmd.hpp
	initialization.cpp
	configurations.cpp
	robot_model.cpp
	update_agent.cpp
	update_simu.cpp
	recording.cpp
	main.cpp
)

add_executable(simurobot ${project_files})
source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${project_files})

target_include_directories(simurobot PUBLIC ${bitctrl_dir})
target_compile_features(simurobot PUBLIC cxx_std_11)

target_link_libraries(simurobot bitctrl)
set_target_properties(simurobot
  PROPERTIES
  CXX_EXTENSIONS OFF
  DEBUG_POSTFIX _d
  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
install(TARGETS simurobot
	LIBRARY DESTINATION  lib
	ARCHIVE DESTINATION  lib
	RUNTIME DESTINATION  bin
)






