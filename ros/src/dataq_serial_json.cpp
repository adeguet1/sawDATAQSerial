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

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnGetChar.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <sawDATAQSerial/mtsDATAQSerial.h>
#include <sawDATAQSerial/mtsDATAQSerialQtWidget.h>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsDATAQSerial", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS_AND_WARNINGS);

    // parse options
    cmnCommandLineOptions options;
    std::string jsonConfigFile = "";
    std::string serialPort = "";
    double rosPeriod = 10.0 * cmn_ms;
    std::string rosNamespace = "/dataq_serial";

    options.AddOptionOneValue("j", "json-config",
                              "json configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &jsonConfigFile);
    options.AddOptionOneValue("s", "serial-port",
                              "serial port as a string",
                              cmnCommandLineOptions::REQUIRED_OPTION, &serialPort);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all data (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the actual device",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);
    options.AddOptionOneValue("n", "ros-namespace",
                              "ROS topic namespace, default is \"/dataq_serial\"",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosNamespace);
    options.AddOptionNoValue("t", "text-only",
                             "text only interface, do not create Qt widgets");

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    const bool hasQt = !options.IsSet("text-only");

    // create the components
    mtsDATAQSerial * sensor = new mtsDATAQSerial("DATAQ", serialPort);
    sensor->Configure(jsonConfigFile);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(sensor);

    // ROS bridge
    std::string bridgeName = "sawDATAQSerial" + rosNamespace;
    std::replace(bridgeName.begin(), bridgeName.end(), '/', '_');
    mtsROSBridge * rosBridge = new mtsROSBridge(bridgeName,
                                                rosPeriod, true);

    // create a Qt user interface if needed
    QApplication * application;
    mtsDATAQSerialQtWidget * sensorWidget;
    if (hasQt) {
        application = new QApplication(argc, argv);
        sensorWidget = new mtsDATAQSerialQtWidget("DATAQ-GUI");
        sensorWidget->Configure();
        componentManager->AddComponent(sensorWidget);
        componentManager->Connect(sensorWidget->GetName(), "DAQ",
                                  sensor->GetName(), "DAQ");
    }

    // configure the bridge
    rosBridge->AddPublisherFromCommandRead<prmInputData, sensor_msgs::Joy>
        ("DAQ", "GetInputs",
         rosNamespace + "/data");


    // add the bridge after all interfaces have been created
    componentManager->AddComponent(rosBridge);

    // connect all interfaces for the ROS bridge
    componentManager->Connect(rosBridge->GetName(), "DAQ",
                              sensor->GetName(), "DAQ");

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    if (hasQt) {
        sensorWidget->show();
        application->exec();
    } else {
        do {
            std::cout << "Press 'q' to quit" << std::endl;
        } while (cmnGetChar() != 'q');
    }

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    cmnLogger::Kill();

    return 0;
}
