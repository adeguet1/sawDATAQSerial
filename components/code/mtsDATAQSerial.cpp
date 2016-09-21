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

#include <cisstConfig.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <sawOptoforceSensor/mtsOptoforce3D.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <cisstNumerical/nmrGaussJordanInverse.h>
#if CISST_HAS_JSON
#include <cisstVector/vctDataFunctionsFixedSizeVectorJSON.h>
#include <cisstVector/vctDataFunctionsFixedSizeMatrixJSON.h>
#endif
#if (CISST_OS != CISST_WINDOWS)
#include <byteswap.h>
#endif


CMN_IMPLEMENT_SERVICES_DERIVED(mtsOptoforce3D, mtsTaskContinuous);

mtsOptoforce3D::mtsOptoforce3D(const std::string &name, unsigned int port) : mtsTaskContinuous(name), 
                                                                             Length(0.0),
                                                                             bias(0.0), scale(1.0),
                                                                             matrix_a_valid(false),
                                                                             sensorSpeed(10),  // 100 Hz
                                                                             sensorFilter(4),  // 15 Hz
                                                                             sensorBias(0),    // unbias
                                                                             configured(false), connected(false)
{
    serialPort.SetPortNumber(port);
    Init();
}

mtsOptoforce3D::mtsOptoforce3D(const std::string &name, const std::string &portName) : mtsTaskContinuous(name), 
                                                                                       Length(0.0),
                                                                                       bias(0.0), scale(1.0),
                                                                                       matrix_a_valid(false),
                                                                                       sensorSpeed(10),  // 100 Hz
                                                                                       sensorFilter(4),  // 15 Hz
                                                                                       sensorBias(0),    // unbias
                                                                                       configured(false), connected(false)
{
    serialPort.SetPortName(portName);
    Init();
}

void mtsOptoforce3D::Init(void)
{
    StateTable.AddData(Count, "Count");
    StateTable.AddData(Status, "Status");
    StateTable.AddData(RawSensor, "ForceRaw");
    StateTable.AddData(Force, "Force");
    StateTable.AddData(ForceTorque, "ForceTorque");
    StateTable.AddData(connected, "Connected");

    mtsInterfaceProvided * interfaceProvided = this->AddInterfaceProvided("Force");
    if (interfaceProvided) {
        interfaceProvided->AddCommandReadState(StateTable, Count, "GetCount");
        interfaceProvided->AddCommandReadState(StateTable, Status, "GetStatus");
        interfaceProvided->AddCommandReadState(StateTable, RawSensor, "GetForceRaw");
        interfaceProvided->AddCommandReadState(StateTable, Force, "GetForce");
        interfaceProvided->AddCommandReadState(StateTable, ForceTorque, "GetForceTorque");
        interfaceProvided->AddCommandReadState(StateTable, connected, "GetConnected");
        interfaceProvided->AddCommandReadState(StateTable, StateTable.Period, "GetTaskPeriod");
        interfaceProvided->AddCommandRead(&mtsOptoforce3D::GetSensorConfig, this, "GetSensorConfig");
        interfaceProvided->AddCommandWrite(&mtsOptoforce3D::SetSensorConfig, this, "SetSensorConfig");
        interfaceProvided->AddCommandVoid(&mtsOptoforce3D::Rebias, this, "Rebias");
        interfaceProvided->AddCommandVoid(&mtsOptoforce3D::Unbias, this, "Unbias");
        interfaceProvided->AddCommandRead(&mtsOptoforce3D::GetBias, this, "GetBias");
        interfaceProvided->AddCommandWrite(&mtsOptoforce3D::SetBias, this, "SetBias");
        interfaceProvided->AddCommandRead(&mtsOptoforce3D::GetLength, this, "GetLength");
        interfaceProvided->AddCommandWrite(&mtsOptoforce3D::SetLength, this, "SetLength");
        interfaceProvided->AddCommandRead(&mtsOptoforce3D::GetScale, this, "GetScale");
        interfaceProvided->AddCommandWrite(&mtsOptoforce3D::SetScale, this, "SetScale");
    }

    // Configure the serial port
    serialPort.SetBaudRate(osaSerialPort::BaudRate115200);
    serialPort.SetCharacterSize(osaSerialPort::CharacterSize8);
    serialPort.SetParityChecking(osaSerialPort::ParityCheckingNone);
    serialPort.SetStopBits(osaSerialPort::StopBitsOne);
    serialPort.SetFlowControl(osaSerialPort::FlowControlNone);

    // Initialize the constant terms in matrix_l
    matrix_l.SetAll(0);
    matrix_l[0][0] = 1;
    matrix_l[1][1] = 1;
    matrix_l[2][2] = 1;

    // Initialize matrix_cal to the identity
    matrix_cal = vctDouble3x3::Eye();
}

void mtsOptoforce3D::Configure(const std::string &filename)
{
    matrix_a_valid = false;
#if CISST_HAS_JSON
    configured = false;
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
        const Json::Value jsonSpeed = jsonConfig["speed"];
        if (jsonSpeed.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"speed\" (update rate from sensor) not specified,"
                                       << " using default value of " << sensorSpeed << std::endl;
        } else {
            sensorSpeed = static_cast<unsigned char>(jsonSpeed.asUInt());
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed speed (update rate from sensor) = "
                                       << sensorSpeed << std::endl;
        }
        const Json::Value jsonFilter = jsonConfig["filter"];
        if (jsonFilter.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"filter\" (cutoff frequency) not specified,"
                                       << " using default value of " << sensorFilter << std::endl;
        } else {
            sensorFilter = static_cast<unsigned char>(jsonFilter.asUInt());
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed filter (cutoff frequency) = "
                                       << sensorFilter << std::endl;
        }
        const Json::Value jsonCalMatrix = jsonConfig["cal-matrix"];
        if (jsonCalMatrix.isNull()) {
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: \"cal-matrix\" not specified" << std::endl;
        } else {
            cmnDataJSON<vctDouble3x6>::DeSerializeText(matrix_a, jsonCalMatrix);
            matrix_a_valid = true;
            CMN_LOG_CLASS_INIT_VERBOSE << "Configure: parsed cal-matrix (A) = " << std::endl
                                       << matrix_a << std::endl;
        }
        configured = true;
    } catch (const std::runtime_error &e) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: runtime_error parsing JSON file: "
                                 << e.what() << std::endl;
    } catch (...) {
        CMN_LOG_CLASS_INIT_ERROR << "Configure: make sure file \""
                                 << filename << "\" is in JSON format" << std::endl;
    }
#else
    // If cisst is not compiled with JSON support, the software returns the raw force values by default.
    CMN_LOG_CLASS_INIT_WARNING << "Configure: JSON support not enabled in cisst, setting scale to 1" << std::endl;
    scale.SetAll(1.0);
    configured = true;
#endif
}

void mtsOptoforce3D::Startup(void)
{
    if (!configured) {
        CMN_LOG_CLASS_INIT_ERROR << "Startup: cannot start because component was not correctly configured" << std::endl;
    }
    else {
        // true --> open serial port in blocking mode
        if (!serialPort.Open(true)) {
            CMN_LOG_CLASS_INIT_ERROR << "Cannot open serial port: " << serialPort.GetPortName() << std::endl;
        }
        else {
            connected = true;
            CMN_LOG_CLASS_INIT_VERBOSE << "Serial Port " << serialPort.GetPortName() << " successfully opened" << std::endl;
            // Send the speed, filter, and bias to the sensor so we know how it is configured
            SendCommand(sensorSpeed, sensorFilter, sensorBias);
        }
    }
}

void mtsOptoforce3D::Run(void)
{
    struct optopacket {
        unsigned char header[4];
        unsigned short count;
        unsigned short status;
        short fx;
        short fy;
        short fz;
        unsigned short checksum;
    };

    union PacketDataType {
        unsigned char bytes[16];
        optopacket packet;
    };

    PacketDataType buffer;
    ProcessQueuedCommands();

    if (connected) {
        bool found = false;
        unsigned short recvChecksum;
#if (CISST_OS == CISST_WINDOWS)
        // On Windows, serialPort.Read seems to always return the requested number
        // of characters, which is sizeof(buffer).
        // Thus, we have to check whether part of the expected packet has been combined
        // with another packet, such as the 7 byte response to the command sent to the sensor.
        int n = serialPort.Read((char *)&buffer, sizeof(buffer));
        while (!found) {
            for (int i = 0; i < n - 3; i++) {
                if ((buffer.bytes[i] == 170) && (buffer.bytes[i + 1] == 7)
                    && (buffer.bytes[i + 2] == 8) && (buffer.bytes[i + 3] == 10)) {
                    if (i != 0) {                               // If pattern not found at beginning of buffer
                        memmove(buffer.bytes, buffer.bytes + i, n - i);    //    shift so that 170 is in buffer[0]
                        serialPort.Read(buffer.bytes + n - i, i);          //    fill the rest of the buffer
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {                                       // If pattern not yet found
                memmove(buffer.bytes, buffer.bytes + n - 4, 4);               //    move last 4 characters to beginning of buffer
                serialPort.Read(buffer.bytes + 4, sizeof(buffer.bytes) - 4);  //    get another 12 characters
            }
        }
        // Now, process the data
        RawSensor.X() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fx)) * scale.X();
        RawSensor.Y() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fy)) * scale.Y();
        RawSensor.Z() = (double)static_cast<short>(_byteswap_ushort(buffer.packet.fz)) * scale.Z();
        Count = _byteswap_ushort(buffer.packet.count);
        Status = _byteswap_ushort(buffer.packet.status);
        recvChecksum = _byteswap_ushort(buffer.packet.checksum);
#else
        // On Linux, serialPort.Read seems to return a complete packet, even if it is less than the
        // requested size.
        // Thus, we can discard packets that are not the correct size.
        while (!found) {
            if (serialPort.Read((char *)&buffer, sizeof(buffer)) == sizeof(buffer)) {
                // Check for expected 4 byte packet header
                found = ((buffer.bytes[0] == 170) && (buffer.bytes[1] == 7)
                         && (buffer.bytes[2] == 8) && (buffer.bytes[3] == 10));
            }
        }
        // Now, process the data
        RawSensor.X() = (double)static_cast<short>(bswap_16(buffer.packet.fx)) * scale.X();
        RawSensor.Y() = (double)static_cast<short>(bswap_16(buffer.packet.fy)) * scale.Y();
        RawSensor.Z() = (double)static_cast<short>(bswap_16(buffer.packet.fz)) * scale.Z();
        Count = bswap_16(buffer.packet.count);
        Status = bswap_16(buffer.packet.status);
        recvChecksum = bswap_16(buffer.packet.checksum);
#endif
        // Verify the checksum (last 2 bytes).
        unsigned short checksum = buffer.bytes[0];
        for (size_t i = 1; i < sizeof(buffer)-2; i++)
            checksum += buffer.bytes[i];
        // (Status == 0) means no errors or overload warnings.
        // For now, we check ((Status&0xFC00) == 0), which ignores overload warnings.
        bool valid = (checksum == recvChecksum) && ((Status&0xFC00) == 0);
        ForceTorque.SetValid(valid);

        if (valid) {
            if (matrix_a_valid) {
                // Obtain forces by applying the calibration matrix, which depends on sensor-specific calibration
                // values (A) and the assumed length to where the forces are applied (Length), which determines matrix_l (L).
                // The calibration matrix, matrix_cal, is equal to inv(A*L)
                Force = matrix_cal*RawSensor - bias;  // F = inv(A*L)*S - bias
            }
            else {
                Force = RawSensor - bias;
            }
            ForceTorque.SetForce(vctDouble6(Force.X(), Force.Y(), Force.Z(), 0.0, 0.0, 0.0));
        }  
    }
    else {
        ForceTorque.SetValid(false);
        osaSleep(0.1);  // If not connected, wait
    }
}

void mtsOptoforce3D::Cleanup(void)
{
    // Close the port
    if (connected) {
        serialPort.Close();
        connected = false;
    }
}

void mtsOptoforce3D::SendCommand(unsigned char speed, unsigned char filter,
                                 unsigned char zero)
{
    if (!connected)
        return;

    struct configpacket {
        unsigned char header[4];
        unsigned char speed;
        unsigned char filter;
        unsigned char zero;
        unsigned char checksum[2];
    };
    configpacket Buffer;
    // Send command via serial port

    Buffer.header[0] = 170;
    Buffer.header[1] = 0;
    Buffer.header[2] = 50;
    Buffer.header[3] = 3;
    Buffer.speed = speed;
    Buffer.filter = filter;
    Buffer.zero = zero;
    unsigned short checksum = 223 + speed + filter + zero;
    Buffer.checksum[0] = (checksum >> 8)&0x00ff;
    Buffer.checksum[1] = checksum&0x00ff;

    serialPort.Write((char*)&Buffer, sizeof(Buffer));

    // Optoforce sensor returns a packet with 7 bytes
    // 170 0 80 1 X CS0 CS1
    //   The X byte is 0 if no error.
    //   CS0,CS1 are the checksum
}

void mtsOptoforce3D::SetSensorConfig(const vctUChar3 &parms)
{
    // Currently, not checking for valid values
    sensorSpeed = parms.X();
    sensorFilter = parms.Y();
    sensorBias = parms.Z();
    SendCommand(sensorSpeed, sensorFilter, sensorBias);
}

void mtsOptoforce3D::GetSensorConfig(vctUChar3 &parms) const
{
    // Only returns local (shadow) copies, since there does not appear
    // to be a way to query the sensor.
    parms.X() = sensorSpeed;
    parms.Y() = sensorFilter;
    parms.Z() = sensorBias;
}

// Note that two consecutive calls to Rebias will not work;
// it is necessary to call Unbias in between and wait at least
// 2 msec before calling Rebias again.
void mtsOptoforce3D::Rebias(void)
{
    sensorBias = 255;
    SendCommand(sensorSpeed, sensorFilter, sensorBias);
}

void mtsOptoforce3D::Unbias(void)
{
    sensorBias = 0;
    SendCommand(sensorSpeed, sensorFilter, sensorBias);
}

void mtsOptoforce3D::GetBias(vctDouble3 &b) const
{
    b = bias;
}

void mtsOptoforce3D::SetBias(const vctDouble3 &b)
{
    bias = b;
}

void mtsOptoforce3D::GetLength(vctDouble3 &len) const
{
    len = Length;
}

void mtsOptoforce3D::SetLength(const vctDouble3 &len)
{
    if (!matrix_a_valid) {
        CMN_LOG_CLASS_RUN_WARNING << "SetLength: matrix_a is not valid (length ignored)" << std::endl;
        return;
    }

    Length = len;

    // Update matrix_l elements that depend on length
    matrix_l[3][1] = Length.Z();
    matrix_l[3][2] = -Length.Y();
    matrix_l[4][0] = -Length.Z();
    matrix_l[4][2] = Length.X();
    matrix_l[5][0] = Length.Y();
    matrix_l[5][1] = -Length.X();

    // Now, compute inverse of A*L
    vctDouble3x3 matrix_product = matrix_a*matrix_l;
    vctDouble3x3 matrix_product_inverse;
    bool cal_valid;

    nmrGaussJordanInverse3x3(matrix_product, cal_valid, matrix_product_inverse, 0.0);

    if (cal_valid)
        matrix_cal = matrix_product_inverse;   // if nonsingular, update class member matrix_cal

    else
        CMN_LOG_CLASS_RUN_WARNING << "SetLength: calibration matrix is singular" << std::endl;
}

void mtsOptoforce3D::GetScale(vctDouble3 &s) const
{
    s = scale;
}

void mtsOptoforce3D::SetScale(const vctDouble3 &s)
{
    scale = s;
}
