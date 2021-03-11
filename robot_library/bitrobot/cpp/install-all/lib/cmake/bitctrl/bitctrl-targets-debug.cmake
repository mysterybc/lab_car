#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bitctrl" for configuration "Debug"
set_property(TARGET bitctrl APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(bitctrl PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libbitctrl_d.so"
  IMPORTED_SONAME_DEBUG "libbitctrl_d.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bitctrl )
list(APPEND _IMPORT_CHECK_FILES_FOR_bitctrl "${_IMPORT_PREFIX}/lib/libbitctrl_d.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
