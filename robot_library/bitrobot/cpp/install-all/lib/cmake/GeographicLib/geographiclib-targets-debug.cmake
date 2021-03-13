#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "GeographicLib" for configuration "Debug"
set_property(TARGET GeographicLib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeographicLib PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libGeographic.so.17.0.0"
  IMPORTED_SONAME_DEBUG "libGeographic.so.17"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeographicLib )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeographicLib "${_IMPORT_PREFIX}/lib/libGeographic.so.17.0.0" )

# Import target "GeographicLib_STATIC" for configuration "Debug"
set_property(TARGET GeographicLib_STATIC APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeographicLib_STATIC PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libGeographic.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeographicLib_STATIC )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeographicLib_STATIC "${_IMPORT_PREFIX}/lib/libGeographic.a" )

# Import target "CartConvert" for configuration "Debug"
set_property(TARGET CartConvert APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(CartConvert PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/CartConvert"
  )

list(APPEND _IMPORT_CHECK_TARGETS CartConvert )
list(APPEND _IMPORT_CHECK_FILES_FOR_CartConvert "${_IMPORT_PREFIX}/bin/CartConvert" )

# Import target "ConicProj" for configuration "Debug"
set_property(TARGET ConicProj APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(ConicProj PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/ConicProj"
  )

list(APPEND _IMPORT_CHECK_TARGETS ConicProj )
list(APPEND _IMPORT_CHECK_FILES_FOR_ConicProj "${_IMPORT_PREFIX}/bin/ConicProj" )

# Import target "GeodesicProj" for configuration "Debug"
set_property(TARGET GeodesicProj APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeodesicProj PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/GeodesicProj"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeodesicProj )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeodesicProj "${_IMPORT_PREFIX}/bin/GeodesicProj" )

# Import target "GeoConvert" for configuration "Debug"
set_property(TARGET GeoConvert APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeoConvert PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/GeoConvert"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeoConvert )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeoConvert "${_IMPORT_PREFIX}/bin/GeoConvert" )

# Import target "GeodSolve" for configuration "Debug"
set_property(TARGET GeodSolve APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeodSolve PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/GeodSolve"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeodSolve )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeodSolve "${_IMPORT_PREFIX}/bin/GeodSolve" )

# Import target "GeoidEval" for configuration "Debug"
set_property(TARGET GeoidEval APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(GeoidEval PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/GeoidEval"
  )

list(APPEND _IMPORT_CHECK_TARGETS GeoidEval )
list(APPEND _IMPORT_CHECK_FILES_FOR_GeoidEval "${_IMPORT_PREFIX}/bin/GeoidEval" )

# Import target "Gravity" for configuration "Debug"
set_property(TARGET Gravity APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Gravity PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/Gravity"
  )

list(APPEND _IMPORT_CHECK_TARGETS Gravity )
list(APPEND _IMPORT_CHECK_FILES_FOR_Gravity "${_IMPORT_PREFIX}/bin/Gravity" )

# Import target "MagneticField" for configuration "Debug"
set_property(TARGET MagneticField APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(MagneticField PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/MagneticField"
  )

list(APPEND _IMPORT_CHECK_TARGETS MagneticField )
list(APPEND _IMPORT_CHECK_FILES_FOR_MagneticField "${_IMPORT_PREFIX}/bin/MagneticField" )

# Import target "Planimeter" for configuration "Debug"
set_property(TARGET Planimeter APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(Planimeter PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/Planimeter"
  )

list(APPEND _IMPORT_CHECK_TARGETS Planimeter )
list(APPEND _IMPORT_CHECK_FILES_FOR_Planimeter "${_IMPORT_PREFIX}/bin/Planimeter" )

# Import target "RhumbSolve" for configuration "Debug"
set_property(TARGET RhumbSolve APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(RhumbSolve PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/RhumbSolve"
  )

list(APPEND _IMPORT_CHECK_TARGETS RhumbSolve )
list(APPEND _IMPORT_CHECK_FILES_FOR_RhumbSolve "${_IMPORT_PREFIX}/bin/RhumbSolve" )

# Import target "TransverseMercatorProj" for configuration "Debug"
set_property(TARGET TransverseMercatorProj APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(TransverseMercatorProj PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/TransverseMercatorProj"
  )

list(APPEND _IMPORT_CHECK_TARGETS TransverseMercatorProj )
list(APPEND _IMPORT_CHECK_FILES_FOR_TransverseMercatorProj "${_IMPORT_PREFIX}/bin/TransverseMercatorProj" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
