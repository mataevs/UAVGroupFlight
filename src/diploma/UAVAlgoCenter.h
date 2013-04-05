/**
 * @file
 *   @brief UI module for algorithm control
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

#ifndef _UAVALGCEN_H_
#define _UAVALGCEN_H_

#include <QWidget>
#include <QMap>
#include <QVBoxLayout>
#include "UASManager.h"
#include "UAVInstance.h"
#include "UASInterface.h"
#include "UAS.h"
#include "ui_AlgoCenter.h"

class UAVAlgoCenter : public QWidget
{
	Q_OBJECT
public:
	UAVAlgoCenter(QWidget* parent = 0);
	~UAVAlgoCenter();

public slots:
	void addUAV(UASInterface* uav);
    void removeUAV(UASInterface *uav);
    void sendSpeedParams();

signals:
    void setUAVControl(double roll, double pitch, double yaw, double thrust);
    void setUAVTarget(float x, float y, float z, float yaw);

protected:
	QMap<UASInterface*, UAVInstance*> uavInstances;
	QVBoxLayout* listLayout;

	void changeEvent(QEvent* e);

private:
	Ui::AlgoCenter* m_ui;
    UAS* uas;

};

#endif //UAVALGCEN_H
