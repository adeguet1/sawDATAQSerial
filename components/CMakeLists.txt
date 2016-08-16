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

project (sawDATAQSerial)

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
  cisst_is_catkin_build (sawDATAQSerial_IS_CATKIN_BUILT)
  if (sawDATAQSerial_IS_CATKIN_BUILT)
    set (sawDATAQSerial_CONFIG_FILE_DIR "${CATKIN_DEVEL_PREFIX}/share/sawDATAQSerial/cmake")
    set (LIBRARY_OUTPUT_PATH "${sawDATAQSerial_BINARY_DIR}/lib")
  else ()
    set (sawDATAQSerial_CONFIG_FILE_DIR "${sawDATAQSerial_BINARY_DIR}")
  endif ()

  # create/configure file for find_package (sawDATAQSerial)
  set (sawDATAQSerial_INCLUDE_DIR
       "${sawDATAQSerial_SOURCE_DIR}/include"
       "${sawDATAQSerial_BINARY_DIR}/include")
  set (sawDATAQSerial_LIBRARY_DIR "${LIBRARY_OUTPUT_PATH}")
  set (sawDATAQSerial_LIBRARIES sawDATAQSerial)

  # Set the version number
  set (sawDATAQSerial_VERSION_MAJOR "1")
  set (sawDATAQSerial_VERSION_MINOR "0")
  set (sawDATAQSerial_VERSION_PATCH "0")
  set (sawDATAQSerial_VERSION "${sawDATAQSerial_VERSION_MAJOR}.${sawDATAQSerial_VERSION_MINOR}.${sawDATAQSerial_VERSION_PATCH}")

  # Generate sawDATAQSerialRevision.h
  configure_file ("${sawDATAQSerial_SOURCE_DIR}/code/sawDATAQSerialRevision.h.in"
                  "${sawDATAQSerial_BINARY_DIR}/include/sawDATAQSerial/sawDATAQSerialRevision.h")

  include_directories (${sawDATAQSerial_INCLUDE_DIR})
  set (sawDATAQSerial_HEADER_DIR "${sawDATAQSerial_SOURCE_DIR}/include/sawDATAQSerial")
  add_library (sawDATAQSerial
               code/mtsDATAQSerial.cpp
               ${sawDATAQSerial_HEADER_DIR}/sawDATAQSerialExport.h
               ${sawDATAQSerial_HEADER_DIR}/mtsDATAQSerial.h
               ${sawDATAQSerial_BINARY_DIR}/include/sawDATAQSerial/sawDATAQSerialRevision.h)

  set_property (TARGET sawDATAQSerial PROPERTY FOLDER "sawDATAQSerial")
  cisst_target_link_libraries (sawDATAQSerial ${REQUIRED_CISST_LIBRARIES})

  set (sawDATAQSerial_CMAKE_CONFIG_FILE
       "${sawDATAQSerial_CONFIG_FILE_DIR}/sawDATAQSerialConfig.cmake")

  configure_file ("${CMAKE_CURRENT_SOURCE_DIR}/cmake/sawDATAQSerialConfig.cmake.in"
                  "${sawDATAQSerial_CMAKE_CONFIG_FILE}"
                  @ONLY)

  cisst_add_config_version ("${sawDATAQSerial_CONFIG_FILE_DIR}/sawDATAQSerialConfigVersion.cmake"
                            VERSION ${sawDATAQSerial_VERSION}
                            DESTINATION "share/sawDATAQSerial"
                            COMPONENT sawDATAQSerial)

  # Install target for headers and library
  install (DIRECTORY "${sawDATAQSerial_SOURCE_DIR}/include/sawDATAQSerial"
                     "${sawDATAQSerial_BINARY_DIR}/include/sawDATAQSerial"
           DESTINATION include
           PATTERN .svn EXCLUDE)

  install (TARGETS sawDATAQSerial
           RUNTIME DESTINATION bin
           LIBRARY DESTINATION lib
           ARCHIVE DESTINATION lib)

  install (FILES "${sawDATAQSerial_CMAKE_CONFIG_FILE}"
           DESTINATION "share/sawDATAQSerial")

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
