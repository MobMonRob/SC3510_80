#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "3D-SensorSDK::MEGCPP" for configuration "Release"
set_property(TARGET 3D-SensorSDK::MEGCPP APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(3D-SensorSDK::MEGCPP PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/3D-Sensor-SDK-1.6.2/C++/bin/libMEGCPP.so"
  IMPORTED_SONAME_RELEASE "libMEGCPP.so.1"
  )

list(APPEND _IMPORT_CHECK_TARGETS 3D-SensorSDK::MEGCPP )
list(APPEND _IMPORT_CHECK_FILES_FOR_3D-SensorSDK::MEGCPP "${_IMPORT_PREFIX}/3D-Sensor-SDK-1.6.2/C++/bin/libMEGCPP.so" )

# Commands beyond this point should not need to know the version.
#set(CMAKE_IMPORT_FILE_VERSION)
