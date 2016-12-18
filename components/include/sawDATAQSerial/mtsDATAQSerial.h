/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  (C) Copyright 2016 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#ifndef _mtsDATAQSerial_h
#define _mtsDATAQSerial_h
#include <stdio.h>
#include <stdlib.h>
#include <cisstOSAbstraction/osaSerialPort.h>
#include <cisstMultiTask/mtsTaskContinuous.h>
#include <cisstParameterTypes/prmInputData.h>
#include <sawDATAQSerial/sawDATAQSerialRevision.h>

// Always include last
#include <sawDATAQSerial/sawDATAQSerialExport.h>

class CISST_EXPORT mtsDATAQSerial: public mtsTaskContinuous {

    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

 public:
    mtsDATAQSerial(const std::string & name, const unsigned int portNumber);
    mtsDATAQSerial(const std::string & name, const std::string & portName);
    ~mtsDATAQSerial(void) {};
    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

 protected:
    void Init(void); // Initialization (called from constructors)
    void StartScanning(void);
    void StopScanning(void);

    // device info
    osaSerialPort mSerialPort;
    std::string mModel;
    std::string mSerialNumber;
    int mFirmware;

    // parsing
    bool mConfigured;
    bool mConnected;
    bool mIsScanning;
    bool mReadBinary = false; //true then read as binary false read as floats
    int mBufferIndex;
    char mBuffer[512];

    // data
    mtsStateTable mDataStateTable;
    prmInputData mInputs;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDATAQSerial);

#endif // _mtsDATAQSerial_h
