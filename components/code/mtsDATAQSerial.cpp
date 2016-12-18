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

#include <sawDATAQSerial/mtsDATAQSerial.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>

CMN_IMPLEMENT_SERVICES_DERIVED(mtsDATAQSerial, mtsTaskContinuous);

mtsDATAQSerial::mtsDATAQSerial(const std::string & name, const unsigned int portNumber):
    mtsTaskContinuous(name),
    mDataStateTable(1000, "Data")
{
    mSerialPort.SetPortNumber(portNumber);
    Init();
}

mtsDATAQSerial::mtsDATAQSerial(const std::string & name, const std::string & portName):
    mtsTaskContinuous(name),
    mDataStateTable(1000, "Data")
{
    mSerialPort.SetPortName(portName);
    Init();
}

void mtsDATAQSerial::Init(void)
{
    mConfigured = false;
    mBufferIndex = 0;

    mInputs.AnalogInputs().SetSize(4);
    mInputs.DigitalInputs().SetSize(2);

    AddStateTable(&mDataStateTable);
    mDataStateTable.SetAutomaticAdvance(false);
    mDataStateTable.AddData(mInputs, "Inputs");

    mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided("DAQ");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(mDataStateTable, mInputs,
                                               "GetInputs");
        interfaceProvided->AddCommandReadState(mDataStateTable, mDataStateTable.PeriodStats,
                                               "GetPeriodStatistics");
    }
}

void mtsDATAQSerial::StartScanning(void)
{
    if (!mIsScanning) {
        mSerialPort.Write("start\r", 6);
        mSerialPort.Flush();
        mIsScanning = true;
    }
}

void mtsDATAQSerial::StopScanning(void)
{
    if (mIsScanning) {
        mSerialPort.Write("stop\r", 5);
        mSerialPort.Flush();
        mIsScanning = false;
    }
}

void mtsDATAQSerial::Configure(const std::string & filename)
{
    mConfigured = true;  // remove this line once we have proper JSON parsing
#if 0
    mConfigured = false;
    try {
        std::ifstream jsonStream;
        Json::Value jsonConfig;
        Json::Reader jsonReader;

        jsonStream.open(filename.c_str());
        if (!jsonReader.parse(jsonStream, jsonConfig)) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: failed to parse configuration" << std::endl
                                     << jsonReader.getFormattedErrorMessages();
            return;
        }
        const Json::Value jsonScale = jsonConfig["scale"];
        if (jsonScale.isNull()) {
            CMN_LOG_CLASS_INIT_ERROR << "Configure: cannot find \"scale\" data in " << filename << std::endl;
            return;
        } else {
            cmnDataJSON<vctDouble3>::DeSerializeText(scale, jsonScale);
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed scale = " << scale << std::endl;
        }
        configured = true;
    } catch (const std::runtime_error & e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: runtime_error parsing JSON file: "
                                 << e.what() << std::endl;
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: make sure file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
#endif
}

void mtsDATAQSerial::Startup(void)
{
    if (!mConfigured) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: cannot start because component was not correctly configured" << std::endl;
    } else {
        if (!mSerialPort.Open(true)) {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: cannot open serial port: "
                                     << mSerialPort.GetPortName() << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: serial port "
                                       << mSerialPort.GetPortName()
                                       << " successfully opened" << std::endl;

            // query system configuration
            char buffer[256];
            int nbRead;
            std::string read, expected;

            // check manufacturer
            mSerialPort.Write("info 0\r", 7);
            nbRead = mSerialPort.Read(buffer, 256);
            // replace the carriage return by end of line
            buffer[nbRead - 1] = '\0';
            read = buffer;
            expected = "info 0 DATAQ";
            if (read != expected) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: cannot check manufacturer, command `info 0` should return `DATAQ`"
                                         << std::endl;
                return;
            }

            // check model name
            mSerialPort.Write("info 1\r", 7);
            nbRead = mSerialPort.Read(buffer, 256);
            // replace the carriage return by end of line
            buffer[nbRead - 1] = '\0';
            read = buffer;
            if (read.compare(0, 6, "info 1") != 0) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get answer for command `info 1`"
                                         << std::endl;
                return;
            }
            // model number
            mModel = read.substr(7, nbRead - 8); // start after info 1 + space
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found DATAQ model ["
                                       << mModel << "]" << std::endl;

            // check firmware revision
            mSerialPort.Write("info 2\r", 7);
            nbRead = mSerialPort.Read(buffer, 256);
            // replace the carriage return by end of line
            buffer[nbRead - 1] = '\0';
            read = buffer;
            if (read.compare(0, 6, "info 2") != 0) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get answer for command `info 2`"
                                         << std::endl;
                return;
            }
            // extract model number
            std::stringstream stream;
            stream << std::hex << buffer[7] << buffer[8];
            stream >> mFirmware;
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found firmware revision ["
                                       << mFirmware << "]" << std::endl;

            // check serial number
            mSerialPort.Write("info 6\r", 7);
            nbRead = mSerialPort.Read(buffer, 256);
            // replace the carriage return by end of line
            buffer[nbRead - 1] = '\0';

            read = buffer;
            if (read.compare(0, 6, "info 6") != 0) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get answer for command `info 6`"
                                         << std::endl;
                return;
            }
            // serial number
            mSerialNumber = read.substr(7, nbRead - 10); // start after info 1 + space and last 2 digits are for internal use, not part of SN
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found serial number ["
                                       << mSerialNumber << "]" << std::endl;

            mConnected = true;

            // in current implementation, configure the scanlist to use all channels
            // analog inputs
            mSerialPort.Write("slist 0 x0\r", 11);
            mSerialPort.Write("slist 1 x1\r", 11);
            mSerialPort.Write("slist 2 x2\r", 11);
            mSerialPort.Write("slist 3 x3\r", 11);
            

            // complete the scan list based on float or binary mode
            // there's also a "asc" mode but we don't see the point to report ADC counts
            if (mReadBinary) {
                // terminate scan list, in binary mode, digital inputs are reported along analog inputs using 2 of the 16 bits per channel

                //mSerialPort.Write("slist 4 xffff\r", 14); - do this and the data has an extra 2 binary inputs
                mSerialPort.Write("bin\r", 4);
            } else {
                // in current implementation, configure the scanlist to use all channels


                // digital inputs are separate in float more and need to be added to the scan list
                mSerialPort.Write("slist 4 x8\r", 11);
                // terminate scan list
                mSerialPort.Write("slist 5 xffff\r", 14);
                mSerialPort.Write("float\r", 6);
            }
            // by default assume we should start scanning
            StartScanning();
        }
    }
}

void mtsDATAQSerial::Run(void)
{
    ProcessQueuedCommands();

    if (mConnected && mIsScanning) {
        if (mReadBinary) {
            /*
                Read byte
                Find first byte that has B0 = 0, this is the start of the message
                Read next 7 bytes, make sure for these 7 bytes, B0 is equal to 1
                Use acquired 8 bytes in 4 sets of 2, each set represents a channel
                
                For digital input, pick one channel, say first 2 bytes and extract D1 = B2 and D0 = B1

                From first byte, get A0 to A4 and from second byte, get A5 to A11 then convert to float
            */

            char buffer[16];
            int nbRead = mSerialPort.Read(buffer, 512);


            int currentValue = 0;
            int tempAnalogValue = 0;
            int analogIndex = 0;
            int analogValue = 0;
            int digitalValues;
            int restartAnalog;
            double convertVolt;

            for (int index = 0; index < nbRead; index++) {
                currentValue = (int)buffer[index]; //note this is a 2's complement conversion

                restartAnalog = currentValue & 1;
                currentValue = currentValue >> 1;

                if (index % 2 == 0) {
                    if(restartAnalog == 0) {
                        mDataStateTable.Advance();
                        std ::cout << "----------start ------------" << std::endl;
                        analogIndex = 0;
                    } else {
                        analogIndex ++;
                    }
                    //------do something with digital values ----------//
                    digitalValues = (currentValue & 3);

                    mInputs.DigitalInputs()[0] = digitalValues & 2;
                    mInputs.DigitalInputs()[1] = digitalValues & 1;

                    //do something with digital here

                    tempAnalogValue = currentValue >> 2;
                } else {
                    analogValue = currentValue << 5;
                    analogValue += tempAnalogValue;


                    convertVolt = analogValue * 0.00488;
                    std::cout <<" i "<< analogIndex << " - " << convertVolt << std::endl;
                    mInputs.AnalogInputs()[analogIndex] = convertVolt;
                }

                    /*
                    //----------delete later on (currently used for reference) -------//
                    int nbits = 8;
                    char s[nbits+1];  // +1 for '\0' terminator
                    s[nbits] = '\0';
                    unsigned int u = *(unsigned int*)&buffer[index];

                    unsigned int mask = 1 << (nbits-1); // fill in values right-to-left
                    for(int i = 0; i < nbits; i++, mask >>= 1) {
                        s[i] = ((u & mask) != 0) + '0';
                    }
                    std::cout <<s<<std::endl;
                    //----------------------------
                    */
            }
        } else {
            char buffer[512];
            int nbRead = mSerialPort.Read(buffer, 512);
            for (int index = 0; index < nbRead; index++) {
                if (buffer[index] == 's') {
                    mBufferIndex = 0;
                    mBuffer[mBufferIndex] = buffer[index];
                    mBufferIndex++;
                } else {
                    if (mBufferIndex < sizeof(mBuffer) && index < sizeof(buffer)) {
                        if (mBufferIndex != 0 && buffer[index] != '\0') {
                            if (buffer[index] != '\r') {
                                mBuffer[mBufferIndex] = buffer[index];
                                mBufferIndex++;
                            } else {
                                // digital inputs are that last element in buffer
                                int digitalValue = mBuffer[mBufferIndex - 1];
                                mBuffer[mBufferIndex] = '\0';
                                mDataStateTable.Start();

                                std::stringstream stream;
                                stream << mBuffer;

                                std::string header;
                                stream >> header
                                       >> mInputs.AnalogInputs()[0]
                                       >> mInputs.AnalogInputs()[1]
                                       >> mInputs.AnalogInputs()[2]
                                       >> mInputs.AnalogInputs()[3];

                                mInputs.DigitalInputs()[0] = digitalValue / 2;
                                mInputs.DigitalInputs()[1] = digitalValue % 2;

                                // std::cout <<  "Data  : " << mInputs << std::endl;
                                mDataStateTable.Advance();
                            }
                        }
                    } else {
                        std::cerr << "#" << std::flush;
                    }
                }
            }
        }
    }
}

void mtsDATAQSerial::Cleanup(void)
{
    // Close the port
    if (mConnected) {
        StopScanning();
        mSerialPort.Close();
        mConnected = false;
    }
}
