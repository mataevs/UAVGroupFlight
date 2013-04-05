/**
 * @file
 *   @brief UI module for UAV Instances control and brief summary
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

#ifndef _UAVCONTROL_H_
#define _UAVCONTROL_H_

#include <QWidget>
#include <QMap>
#include <QVBoxLayout>
#include <QFile>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>
#include "UASManager.h"
#include "UAVInstance.h"
#include "UASInterface.h"
#include "UAVControlInstance.h"
#include "UAS.h"
#include "UASAgent.h"
#include "MAVLinkProtocol.h"
#include "UAVLogging.h"
#include "ui_UAVControl.h"

class UAVControl : public QWidget
{
	Q_OBJECT
public:
	UAVControl(QWidget* parent = 0);
	~UAVControl();

public slots:
	void addUAV();
    void removeUAV(UASInterface *uav);
	void loadScenario();
	void saveScenario();
    void changeCollAvParams();
	void toggleSimulation();
	void sendLeaderWaypoints();

signals:
    void setUAVControl(double roll, double pitch, double yaw, double thrust);
    void setUAVTarget(float x, float y, float z, float yaw);

protected:
    QMap<UASInterface*, UAVControlInstance*> uavInstances;
	QVBoxLayout* listLayout;

	void changeEvent(QEvent* e);

private:
	Ui::UAVControl* m_ui;
    UAS* uas;
	QTimer* timer;
	bool simStarted;
	UAVLogging* logger;

};

#endif //_UAVCONTROL_H_
