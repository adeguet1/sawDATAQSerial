#ifndef _mtsPIDQtWidget_h
#define _mtsPIDQtWidget_h

#include <cisstCommon/cmnXMLPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstVector/vctPlot2DOpenGLQtWidget.h>
#include <cisstMultiTask/mtsComponent.h>
#include <cisstVector/vctQtWidgetDynamicVector.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmPositionJointSet.h>

#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>
#include <sawControllers/sawControllersQtExport.h>

class CISST_EXPORT mtsPIDQtWidget: public QWidget, public mtsComponent
{
    Q_OBJECT;
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

public:
    mtsPIDQtWidget(const std::string & componentName, unsigned int numberOfAxis,
                   double periodInSeconds = 50.0 * cmn_ms);
    mtsPIDQtWidget(const mtsComponentConstructorNameAndUInt &arg);
    ~mtsPIDQtWidget(){}

    void Configure(const std::string & filename = "");
    void Cleanup(void);

protected:
    void Init(void);
    virtual void closeEvent(QCloseEvent * event);

signals:
    void SignalEnablePID(bool enable);

private slots:
    //! slot to select which axis to plot
    void SlotPlotIndex(int newAxis);
    //! timer event to update GUI
    void timerEvent(QTimerEvent * event);

private:
    //! setup PID controller GUI
    void setupUi(void);
    int TimerPeriodInMilliseconds;

    void JointLimitEventHandler(const vctBoolVec & flags);
    void ErrorEventHandler(const std::string & message);
    void EnableEventHandler(const bool & enable);

protected:

    struct ControllerPIDStruct {
        mtsFunctionRead  GetAnalogValues;
        mtsFunctionRead  GetDigitalValues;
    } PID;

private:

    // GUI: Commands
    vctQtWidgetDynamicVectorDoubleRead * QVRAnalogValueWidget;
    vctQtWidgetDynamicVectorDoubleRead * QVRDigitalValueWidget;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsDATAQSerialQtWidget);

#endif // _mtsDATAQSerialQtWidget_h
