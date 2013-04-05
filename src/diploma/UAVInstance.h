/**
 * @file
 *   @brief UI component for UAV info display inside UAVAlgoCenter
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

#ifndef _UAVINST_H_
#define _UAVINST_H_

#include <QWidget>
#include <QString>
#include "UASInterface.h"

namespace Ui
{
	class UAVInstance;
}

class UAVInstance : public QWidget
{
	Q_OBJECT
public:
	UAVInstance(UASInterface* uas, QWidget* parent = 0);
    double getThrust();
	~UAVInstance();

public slots:
	void receiveName(const QString& name);
	void receiveUAVLocalPosition(UASInterface* uav, double x, double y, double z, quint64 usec);
    void receiveUAVGlobalPosition(UASInterface* uav, double lon, double lat, double alt, quint64 usec);
    void receiveUAVSpeed(UASInterface* uav, double x, double y, double z, quint64 usec);
	void receiveUAVAttitude(UASInterface* uav, double roll, double pitch, double yaw, quint64 usec);
	void receiveUAVAttitudeSpeed(int uas, double rollspeed, double pitchspeed, double yawspeed, quint64 usec);
    void receiveThrust(UASInterface* uas, double thrust);

protected:
	void changeEvent(QEvent *e);
	UASInterface* uas;
	double x;
	double y;
	double z;
	double vx;
	double vy;
	double vz;
	double lat;
	double lon;
	double alt;
	double roll;
	double yaw;
	double pitch;
	double v_roll;
	double v_yaw;
	double v_pitch;
	double groundDistance;
    double thrust;
	QString name;

private:
	Ui::UAVInstance *m_ui;
};

#endif //UAVINST_H
