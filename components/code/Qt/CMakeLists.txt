set (REQUIRED_CISST_LIBRARIES
     cisstCommon
     cisstCommonXML
     cisstVector
     cisstVectorQt
     cisstNumerical
     cisstOSAbstraction
     cisstMultiTask
     cisstMultiTaskQt
     cisstParameterTypes
     cisstQt)

# cisst
find_package (cisst REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND_AS_REQUIRED)

  # cisst
  include (${CISST_USE_FILE})

  # for Qt generated header files
  include_directories (${CMAKE_CURRENT_BINARY_DIR})
  if (CISST_HAS_QT4)
    qt4_wrap_cpp (SAW_DATAQSerial_QT_WRAP_CPP
                  ${sawDATAQSerial_HEADER_DIR}/mtsDATAQSerialQtWidget.h)
  else (CISST_HAS_QT4)
    set (CMAKE_AUTOMOC ON)
  endif (CISST_HAS_QT4)

  add_library (sawDATAQSerialQt
               ${sawDATAQSerial_HEADER_DIR}/mtsDATAQSerialQtWidget.h
               ${sawDATAQSerial_HEADER_DIR}/sawDATAQSerialQtExport.h
               mtsDATAQSerialQtWidget.cpp
               ${SAW_DATAQSERIAL_QT_WRAP_CPP})
  set_property (TARGET sawDATAQSerialQt PROPERTY FOLDER "sawDATAQSerial")
  cisst_target_link_libraries (sawDATAQSerialQt ${REQUIRED_CISST_LIBRARIES})

  # make sure the new library is known by the parent folder to add to the config file
  set (sawDATAQSerialQt_LIBRARIES sawDATAQSerialQt PARENT_SCOPE)

else (cisst_FOUND_AS_REQUIRED)
  message ("Information: code will not be compiled, it requires ${REQUIRED_CISST_LIBRARIES}")
endif (cisst_FOUND_AS_REQUIRED)
