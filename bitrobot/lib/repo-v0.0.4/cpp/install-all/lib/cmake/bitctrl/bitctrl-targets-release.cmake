#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "bitctrl" for configuration "Release"
set_property(TARGET bitctrl APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(bitctrl PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libbitctrl.so"
  IMPORTED_SONAME_RELEASE "libbitctrl.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS bitctrl )
list(APPEND _IMPORT_CHECK_FILES_FOR_bitctrl "${_IMPORT_PREFIX}/lib/libbitctrl.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
