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

// system include
#include <iostream>

// Qt include
#include <QMessageBox>
#include <QCloseEvent>
#include <QCoreApplication>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmJointType.h>

#include <sawDATAQSerial/mtsDATAQSerialQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDATAQSerialQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt);

mtsDATAQSerialQtWidget::mtsDATAQSerialQtWidget(const std::string & componentName, double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000) // Qt timers are in milliseconds
{
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("DAQ");
    if (interfaceRequired) {
        interfaceRequired->AddFunction("GetDigitalInputs", DAQ.GetAnalogInputs);
        interfaceRequired->AddFunction("GetAnalogInputs", DAQ.GetDigitalInputs);   
    }
}

void mtsDATAQSerialQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}

void mtsDATAQSerialQtWidget::Startup(void)
{
    setupUi();
    startTimer(TimerPeriodInMilliseconds);
    if (!parent()) {
        show();
    }
}

void mtsDATAQSerialQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "Cleanup" << std::endl;
}


void mtsDATAQSerialQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsTeleOperationPSMQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsDATAQSerialQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // retrieve transformations
    DAQ.GetAnalogInputs(DAQ.AnalogInputs);
    DAQ.GetDigitalInputs(DAQ.DigitalInputs);

    // update display
    QVRAnalogInputsWidget->SetValue(DAQ.AnalogInputs);
    QVRDigitalInputsWidget->SetValue(DAQ.DigitalInputs);

    // TeleOperation.GetPeriodStatistics(IntervalStatistics);
    // QMIntervalStatistics->SetValue(IntervalStatistics);
}

void mtsDATAQSerialQtWidget::setupUi(void)
{
    // 3D frames ...

    // state info
    QHBoxLayout * topLayout = new QHBoxLayout;
    this->setLayout(topLayout);

    QVRAnalogInputsWidget = new vctQtWidgetDynamicVectorDoubleRead();
    topLayout->addWidget(QVRAnalogInputsWidget);

    QVRDigitalInputsWidget = new vctQtWidgetDynamicVectorBoolRead();
    topLayout->addWidget(QVRDigitalInputsWidget);

    // Timing
    QMIntervalStatistics = new mtsQtWidgetIntervalStatistics();
    topLayout->addWidget(QMIntervalStatistics);
}
