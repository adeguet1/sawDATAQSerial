/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Zihan Chen, Anton Deguet
  Created on: 2013-02-20

  (C) Copyright 2013-2015 Johns Hopkins University (JHU), All Rights Reserved.

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
#include <QGridLayout>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QCloseEvent>
#include <QCoreApplication>

// cisst
#include <cisstMultiTask/mtsInterfaceRequired.h>
#include <cisstParameterTypes/prmJointType.h>
#include <cisstCommon/cmnConstants.h>
#include <cisstCommon/cmnUnits.h>

#include <sawControllers/mtsDATAQSerialQtWidget.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsDATAQSerialQtWidget, mtsComponent, mtsComponentConstructorNameAndUInt)

mtsDATAQSerialQtWidget::mtsDATAQSerialQtWidget(const std::string & componentName,
                               unsigned int numberOfAxis,
                               double periodInSeconds):
    mtsComponent(componentName),
    TimerPeriodInMilliseconds(periodInSeconds * 1000), // Qt timer are in milliseconds
    NumberOfAxis(numberOfAxis)
{
    Init();
}

mtsDATAQSerialQtWidget::mtsDATAQSerialQtWidget(const mtsComponentConstructorNameAndUInt & arg):
    mtsComponent(arg.Name),
    TimerPeriodInMilliseconds(50),
    NumberOfAxis(arg.Arg)
{
    Init();
}

void mtsDATAQSerialQtWidget::Init(void)
{
    PID.StateJoint.Position().SetSize(NumberOfAxis);
    PID.StateJoint.Velocity().SetSize(NumberOfAxis);
    PID.StateJoint.Effort().SetSize(NumberOfAxis);
    PID.StateJointDesired.Position().SetSize(NumberOfAxis);
    PID.StateJointDesired.Velocity().SetSize(0);
    PID.StateJointDesired.Effort().SetSize(NumberOfAxis);

    DesiredPosition.SetSize(NumberOfAxis);
    DesiredPosition.SetAll(0.0);
    UnitFactor.SetSize(NumberOfAxis);
    UnitFactor.SetAll(1.0);

    DirectControl = false;
    PlotIndex = 0;

    // Setup cisst interface
    mtsInterfaceRequired * interfaceRequired = AddInterfaceRequired("Controller");
    if (interfaceRequired) {
        
        interfaceRequired->AddFunction("GetAnalogValues", PID.GetAnalogValues);
        interfaceRequired->AddFunction("GetDigitalValues", PID.GetDigitalValues);
    }
    setupUi();
    startTimer(TimerPeriodInMilliseconds); // ms
}

void mtsDATAQSerialQtWidget::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: " << filename << std::endl;
}


void mtsDATAQSerialQtWidget::Cleanup(void)
{
    this->hide();
    CMN_LOG_CLASS_INIT_VERBOSE << "mtsPIDQtWidget::Cleanup" << std::endl;
}

//---------- Protected --------------------------
void mtsPIDQtWidget::closeEvent(QCloseEvent * event)
{
    int answer = QMessageBox::warning(this, tr("mtsPIDQtWidget"),
                                      tr("Do you really want to quit this application?"),
                                      QMessageBox::No | QMessageBox::Yes);
    if (answer == QMessageBox::Yes) {
        event->accept();
        QCoreApplication::exit();
    } else {
        event->ignore();
    }
}

void mtsDATAQSerialQtWidget::SlotPlotIndex(int newAxis)
{
    PlotIndex = newAxis;
    QVPlot->SetContinuousExpandYResetSlot();
}


void mtsDATAQSerialQtWidget::timerEvent(QTimerEvent * CMN_UNUSED(event))
{
    // make sure we should update the display
    if (this->isHidden()) {
        return;
    }

    // get data from the PID
    PID.GetAnalogValues(PID.AnalogValues);
    PID.GetDigitalValues(PID.DigitalValues);

    // update GUI
    QVRCurrentPositionWidget->SetValue(PID.AnalogValues[0]);
    QVRCurrentEffortWidget->SetValue(PID.Digitalvalues[0]);

    // plot
    AnalogSignal->AppendPoint(vctDouble2(PID.AnalogValues.Timestamp(),
                                                  PID.AnalogValues[0].Element(PlotIndex)));
    DigitalSignal->AppendPoint(vctDouble2(PID.DigitalValues.Timestamp(),
                                                  PID.AnalogValues[0].Position().Element(PlotIndex)));
    QVPlot->updateGL();
}

////------------ Private Methods ----------------
void  mtsDATAQSerialQtWidget::setupUi(void)
{
    QFont font;
    font.setBold(true);
    font.setPointSize(12);

    const double maximum = 30000;

    QGridLayout * gridLayout = new QGridLayout();
    gridLayout->setSpacing(1);

    int row = 0;
    QLabel * analogLabel = new QLabel("Current Analog");
    analogLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(analogLabel, row, 0);
    QVRAnalogValueWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRAnalogValueWidget->SetPrecision(3);
    gridLayout->addWidget(QVRAnalogValueWidget, row, 1);
    row++;

    QLabel * digitalLabel = new QLabel("Current Digital");
    digitalLabel->setAlignment(Qt::AlignRight);
    gridLayout->addWidget(digitalLabel, row, 0);
    QVRDitialValueWidget = new vctQtWidgetDynamicVectorDoubleRead();
    QVRDigitalValueWidget->SetPrecision(3);
    gridLayout->addWidget(QVRDigitalValueWidget, row, 1);
    row++;

    // main layout
    QVBoxLayout * mainLayout = new QVBoxLayout;
    mainLayout->addLayout(gridLayout);
    setLayout(mainLayout);

    setWindowTitle(this->GetName().c_str());
    setMinimumWidth(750);
    resize(sizeHint());
}

void  mtsDATAQSerialQtWidget::JointLimitEventHandler(const vctBoolVec & flags)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "JointLimitEventHandler: " << flags << std::endl;
}

void  mtsDATAQSerialQtWidget::ErrorEventHandler(const std::string & message)
{
    CMN_LOG_CLASS_RUN_VERBOSE << "ErrorEventHandler: " << message << std::endl;
}

void  mtsDATAQSerialQtWidget::EnableEventHandler(const bool & enable)
{
    emit SignalEnablePID(enable);
}
