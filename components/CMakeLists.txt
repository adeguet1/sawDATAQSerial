#
# (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

project (sawOptoforceSensor)

cmake_minimum_required (VERSION 2.8)

# Core libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstParameterTypes)

find_package (cisst 1.0.8 COMPONENTS ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # catkin/ROS paths
  cisst_is_catkin_build (sawOptoforceSensor_IS_CATKIN_BUILT)
  if (sawOptoforceSensor_IS_CATKIN_BUILT)
    set (sawOptoforceSensor_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/sawOptoforceSensor/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawOptoforceSensor_BINARY_DIR}/lib")
  else ()
    set (sawOptoforceSensor_CONFIG_FILE_DIR "${sawOptoforceSensor_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawOptoforceSensor)
  set (sawOptoforceSensor_INCLUDE_DIR
       "${sawOptoforceSensor_SOURCE_DIR}/include"
       "${sawOptoforceSensor_BINARY_DIR}/include")
  set (sawOptoforceSensor_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawOptoforceSensor_LIBRARIES sawOptoforceSensor)

  # Set the version number
  set (sawOptoforceSensor_VERSION_MAJOR "1")
  set (sawOptoforceSensor_VERSION_MINOR "0")
  set (sawOptoforceSensor_VERSION_PATCH "0")
  set (sawOptoforceSensor_VERSION "${sawOptoforceSensor_VERSION_MAJOR}.${sawOptoforceSensor_VERSION_MINOR}.${sawOptoforceSensor_VERSION_PATCH}")

  # Generate sawOptoforceSensorRevision.h
  configure_file ("${sawOptoforceSensor_SOURCE_DIR}/code/sawOptoforceSensorRevision.h.in"
                  "${sawOptoforceSensor_BINARY_DIR}/include/sawOptoforceSensor/sawOptoforceSensorRevision.h")

  include_directories (${sawOptoforceSensor_INCLUDE_DIR})
  set (sawOptoforceSensor_HEADER_DIR "${sawOptoforceSensor_SOURCE_DIR}/include/sawOptoforceSensor")
  add_library (sawOptoforceSensor
               code/mtsOptoforce3D.cpp
               ${sawOptoforceSensor_HEADER_DIR}/sawOptoforceSensorExport.h
               ${sawOptoforceSensor_HEADER_DIR}/mtsOptoforce3D.h
               ${sawOptoforceSensor_BINARY_DIR}/include/sawOptoforceSensor/sawOptoforceSensorRevision.h)

  set_property (TARGET sawOptoforceSensor PROPERTY FOLDER "sawOptoforceSensor")
  cisst_target_link_libraries (sawOptoforceSensor ${REQUIRED_CISST_LIBRARIES})

  set (sawOptoforceSensor_CMAKE_CONFIG_FILE
       "${sawOptoforceSensor_CONFIG_FILE_DIR}/sawOptoforceSensorConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawOptoforceSensorConfig.cmake.in"
                  "${sawOptoforceSensor_CMAKE_CONFIG_FILE}"
                  @ONLY)

  cisst_add_config_version ("${sawOptoforceSensor_CONFIG_FILE_DIR}/sawOptoforceSensorConfigVersion.cmake"
                            VERSION ${sawOptoforceSensor_VERSION}
                            DESTINATION "share/sawOptoforceSensor"
                            COMPONENT sawOptoforceSensor)

  # Install target for headers and library
  install (DIRECTORY "${sawOptoforceSensor_SOURCE_DIR}/include/sawOptoforceSensor"
                     "${sawOptoforceSensor_BINARY_DIR}/include/sawOptoforceSensor"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawOptoforceSensor
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawOptoforceSensor_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawOptoforceSensor")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
