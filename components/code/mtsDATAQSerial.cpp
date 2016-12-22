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
#include <stdio.h>

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
    mReadBinary = true;

    temp = 0;

    //set the size of the analog and digital inputs un accordiance with the corrrect model
    //in this istnance it is the DATAQ DI-145
    mInputs.AnalogInputs().SetSize(4);
    mInputs.DigitalInputs().SetSize(2);

    //create the state table for the recorded values
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
    //if we are not currently scanning, then we start scanning
    if (!mIsScanning) {
        WriteAndCheck("start");
        mSerialPort.Flush();
        mIsScanning = true;
    }
}

void mtsDATAQSerial::StopScanning(void)
{
    //if we are currently scaning, then we stop scanning
    if (mIsScanning) {
        WriteAndCheck("stop");
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

std::string mtsDATAQSerial::WriteAndCheck(const std::string & read) {//change read to std::string
    //call this method when you want to write and then read in input to clear the buffer

    //write to mSerialPort the command
    std::string command = read;
    command.append("\r");
    int readLen = command.size();
    mSerialPort.Write(command.c_str(), readLen);

    //read from mSerialPort the output
    char buffer[256];
    int nbRead = mSerialPort.Read(buffer, 256);
    // replace the carriage return by end of line
    buffer[nbRead - 1] = '\0';

    //convert char array to a string
    std::string output = buffer;

    if (output.compare(0, read.size(), read) != 0) {
        CMN_LOG_CLASS_INIT_ERROR << "Failed to get answer for command `" << read << "`, received `"
                                 << output << "`" << std::endl;

    }
    return output;
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
            std::string read, expected;

            // send a stop scanning in case we left it in scan mode
            mSerialPort.Write("stop\r", 5);
            mSerialPort.Flush();
            mIsScanning = false;

            // purge whatever was left on serial port
            int nbRead = 0;
            char buffer[256];
            do {
                nbRead = mSerialPort.Read(buffer, sizeof(buffer));
            } while (nbRead != 0);

            //check manufacturer
            read = WriteAndCheck("info 0");
            expected = "info 0 DATAQ";
            if (read != expected) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: cannot check manufacturer, command `info 0` should return `DATAQ`, not ["
                                         << read << "]" << std::endl;
                return;
            }

            // check model name
            read = WriteAndCheck("info 1");

            // model number
            mModel = read.substr(7, nbRead - 8); // start after info 1 + space
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found DATAQ model ["
                                       << mModel << "]" << std::endl;

            // check firmware revision
            read = WriteAndCheck("info 2");
            /*if (read.compare(0, 6, "info 2") != 0) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get answer for command `info 2`, received ["
                                         << read << "]" << std::endl;
                return;
            }*/
            // extract model number
            std::stringstream stream;
            stream << std::hex << read[7] << read[8];
            stream >> mFirmware;
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found firmware revision ["
                                       << mFirmware << "]" << std::endl;

            // check serial number
            read = WriteAndCheck("info 6");
            /*if (read.compare(0, 6, "info 6") != 0) {
                CMN_LOG_CLASS_INIT_ERROR << "Startup: failed to get answer for command `info 6`, received ["
                                         << read << "]" << std::endl;
                return;
            }*/

            // serial number
            mSerialNumber = read.substr(7, nbRead - 10); // start after info 1 + space and last 2 digits are for internal use, not part of SN
            CMN_LOG_CLASS_INIT_VERBOSE << "Startup: found serial number ["
                                       << mSerialNumber << "]" << std::endl;

            mConnected = true;

            read = WriteAndCheck("asc");
            // in current implementation, configure the scanlist to use all channels
            // analog inputs
            read = WriteAndCheck("slist 0 x0");
            read = WriteAndCheck("slist 1 x1");
            read = WriteAndCheck("slist 2 x2");
            read = WriteAndCheck("slist 3 x3");

            // complete the scan list based on float or binary mode
            // there's also a "asc" mode but we don't see the point to report ADC counts
            if (mReadBinary) {
                // terminate scan list, in binary mode, digital inputs are reported along analog inputs using 2 of the 16 bits per channel
                read = WriteAndCheck("slist 4 xffff");
                read = WriteAndCheck("bin");
            } else {
                // in current implementation, configure the scanlist to use all channels
                // digital inputs are separate in float mode and need to be added to the scan list
                read = WriteAndCheck("slist 4 x8");
                read = WriteAndCheck("slist 5 xffff");
                read = WriteAndCheck("float");
            }
            // by default assume we should start scanning
            StartScanning();
        }
    }
}

void mtsDATAQSerial::Run(void)
{
    //choose the type of reading we want to perform
    ProcessQueuedCommands();
    if (mConnected && mIsScanning) {
        if (mReadBinary) {
            ReadBinary();
        } else {
            ReadAscii();
        }
    }
}

void mtsDATAQSerial::ReadBinary(void)
{
    //read in data as binary

    char buffer[8];
    int nbRead = mSerialPort.Read(buffer, sizeof(buffer));



    char currentValue;
    int tempAnalogValue;
    int analogIndex = 0;
    int analogValue = 0;
    int digitalValues;
    char syncBit;
    double convertVolt;

    //iterate throught evalues we have read in
    for (int index = 0; index < nbRead; index++) {
        currentValue = buffer[index];

        syncBit = currentValue & 1;
        currentValue = currentValue >> 1;

        if (index % 2 == 0) {
            //find out if we are the sync bit if so we should start
            if (syncBit == 0) {
                mDataStateTable.Start();
                analogIndex = 0;
            } else {
                analogIndex ++;
            }

            //------get and parse the digital values
            digitalValues = (currentValue & 3);
            mInputs.DigitalInputs()[0] = digitalValues & 2;
            mInputs.DigitalInputs()[1] = digitalValues & 1;
            tempAnalogValue = currentValue >> 2;


        } else {
            int value = currentValue & 63;
            int highBit = currentValue >> 6;
            int negateHighBit = !highBit;
            int shiftHighBit = negateHighBit<< 6;
            int currentValue = value + shiftHighBit;
                
           
            //handle 2's complement conversion from binary to integer
            if(highBit + 1 == 0) { //positive
                currentValue++;
                analogValue = currentValue << 5;
                analogValue += tempAnalogValue;

            }else if(highBit == 0) { //negitive

                currentValue++;
                analogValue = currentValue << 5;
                analogValue += tempAnalogValue;

                int dValue = analogValue ^ 4095;
                int value6 = dValue * -1;
                analogValue = value6--;
            }

            //convert to volts and parse in to database
            convertVolt = analogValue * 0.00488;
            mInputs.AnalogInputs()[analogIndex] = convertVolt;

            //if we reach the end we should advance
            if (analogIndex == 3) {
                mDataStateTable.Advance();
            }
        }
    }

}


void mtsDATAQSerial::ReadAscii(void)
{
    //read in data as ascii values

    char buffer[512];
    int nbRead = mSerialPort.Read(buffer, 512);
    for (int index = 0; index < nbRead; index++) {

        if (buffer[index] == 's') {
            //if this is the start of the list, reset everything
            mBufferIndex = 0;
            mBuffer[mBufferIndex] = buffer[index];
            mBufferIndex++;
        } else {
            if (mBufferIndex < sizeof(mBuffer) && index < sizeof(buffer)) {
                if (mBufferIndex != 0 && buffer[index] != '\0') {
                    if (buffer[index] != '\r') {
                        //if this is the not the end of a row keep going
                        mBuffer[mBufferIndex] = buffer[index];
                        mBufferIndex++;
                    } else {
                        // end of a row parse information

                        //get the digital values
                        int digitalValue = mBuffer[mBufferIndex - 1];
                        mBuffer[mBufferIndex] = '\0';

                        mDataStateTable.Start();

                        //parse in the analog values
                        std::stringstream stream;
                        stream << mBuffer;

                        std::string header;
                        stream >> header
                               >> mInputs.AnalogInputs()[0]
                               >> mInputs.AnalogInputs()[1]
                               >> mInputs.AnalogInputs()[2]
                               >> mInputs.AnalogInputs()[3];

                        //pasre in the digital values
                        mInputs.DigitalInputs()[0] = digitalValue / 2;
                        mInputs.DigitalInputs()[1] = digitalValue % 2;

                        mDataStateTable.Advance();
                    }
                }
            } else {
                std::cerr << "#" << std::flush;
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
