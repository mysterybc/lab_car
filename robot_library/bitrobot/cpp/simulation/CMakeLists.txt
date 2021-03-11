cmake_minimum_required(VERSION 3.0)

add_compile_options(-std=c++11)
project(simulation)

#set(bitctrl_dir D:/Coding/cpp/BFP/src/Bitctrl CACHE PATH "path to bitctrl project")
#add_subdirectory(${bitctrl_dir} ${CMAKE_CURRENT_BINARY_DIR}/bitctrl)


#find_package(ssnet REQUIRED)
find_package(bitctrl REQUIRED)
add_subdirectory(simulation)
#add_subdirectory(benchmark)



#set_target_properties(bitctrl
#  PROPERTIES
#  DEBUG_POSTFIX _d
#  ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
#  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib"
#  RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
#)






