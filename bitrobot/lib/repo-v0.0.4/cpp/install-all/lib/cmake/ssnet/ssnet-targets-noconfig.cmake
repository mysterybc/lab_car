#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ssnet::ssnetwork" for configuration ""
set_property(TARGET ssnet::ssnetwork APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::ssnetwork PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libssnetwork.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::ssnetwork )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::ssnetwork "${_IMPORT_PREFIX}/lib/libssnetwork.a" )

# Import target "ssnet::sstcp_bit_test" for configuration ""
set_property(TARGET ssnet::sstcp_bit_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::sstcp_bit_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/sstcp_bit_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::sstcp_bit_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::sstcp_bit_test "${_IMPORT_PREFIX}/bin/sstcp_bit_test" )

# Import target "ssnet::protobit_test" for configuration ""
set_property(TARGET ssnet::protobit_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::protobit_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/protobit_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::protobit_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::protobit_test "${_IMPORT_PREFIX}/bin/protobit_test" )

# Import target "ssnet::sstcp_proto_test" for configuration ""
set_property(TARGET ssnet::sstcp_proto_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::sstcp_proto_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/sstcp_proto_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::sstcp_proto_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::sstcp_proto_test "${_IMPORT_PREFIX}/bin/sstcp_proto_test" )

# Import target "ssnet::mersenne_test" for configuration ""
set_property(TARGET ssnet::mersenne_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::mersenne_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/mersenne_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::mersenne_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::mersenne_test "${_IMPORT_PREFIX}/bin/mersenne_test" )

# Import target "ssnet::protocol_test" for configuration ""
set_property(TARGET ssnet::protocol_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::protocol_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/protocol_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::protocol_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::protocol_test "${_IMPORT_PREFIX}/bin/protocol_test" )

# Import target "ssnet::sstcp_test" for configuration ""
set_property(TARGET ssnet::sstcp_test APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ssnet::sstcp_test PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/bin/sstcp_test"
  )

list(APPEND _IMPORT_CHECK_TARGETS ssnet::sstcp_test )
list(APPEND _IMPORT_CHECK_FILES_FOR_ssnet::sstcp_test "${_IMPORT_PREFIX}/bin/sstcp_test" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
