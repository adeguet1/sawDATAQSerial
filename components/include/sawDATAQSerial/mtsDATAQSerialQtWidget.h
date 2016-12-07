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

#ifndef _mtsDATAQSerialQtWidget_h
#define _mtsDATAQSerialQtWidget_h

#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstMultiTask/mtsQtWidgetIntervalStatistics.h>
#include <cisstParameterTypes/prmInputData.h>

#include <QTabWidget>
#include <QSpinBox>

// Always include last
#include <sawDATAQSerial/sawDATAQSerialQtExport.h>

class CISST_EXPORT mtsDATAQSerialQtWidget: public QTabWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_NO_DYNAMIC_CREATION, CMN_LOG_ALLOW_DEFAULT);

public:

    mtsDATAQSerialQtWidget(const std::string & componentName,
			   double periodInSeconds = 50.0 * cmn_ms);
    ~mtsDATAQSerialQtWidget(){}

    void Configure(const std::string & filename = "");
    void Startup(void);
    void Cleanup(void);

protected:
    virtual void timerEvent(QTimerEvent * event);
    virtual void closeEvent(QCloseEvent * event);

    //! setup TeleOperationPSM controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    struct {
      mtsFunctionRead GetInputs;
      prmInputData Inputs;
    } DAQ;

    vctQtWidgetDynamicVectorDoubleRead * QVRAnalogInputsWidget;
    vctQtWidgetDynamicVectorBoolRead * QVRDigitalInputsWidget;

    // timing
    mtsIntervalStatistics IntervalStatistics;
    mtsQtWidgetIntervalStatistics * QMIntervalStatistics;
    mtsFunctionRead GetPeriodStatistics;

    vctPlot2DOpenGLQtWidget * QVPlot;
    vctPlot2DBase::Signal * AnalogSignal;
    QSpinBox * QSBPlotIndex;
    int PlotIndex;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDATAQSerialQtWidget);

#endif // _mtsDATAQSerialQtWidget_h
