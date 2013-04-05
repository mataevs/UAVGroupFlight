/**
 * @file
 *   @brief UI component for UAV info display inside UAVControl
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

#ifndef _UAVCTRLINST_H_
#define _UAVCTRLINST_H_

#include <QWidget>
#include <QString>
#include "UASInterface.h"
#include "UAS.h"
#include "UASAgent.h"

namespace Ui
{
	class UAVControlInstance;
}

class UAVControlInstance : public QWidget
{
	Q_OBJECT
public:
	UAVControlInstance(UAS* uas, QWidget* parent = 0);
    double getThrust();
	void changeStartCoordinates(float lat, float lon, float alt);
	float getStartLatitude();
	float getStartLongitude();
	float getStartAltitude();
	~UAVControlInstance();

public slots:
	void receiveName(const QString& name);
	void receiveUAVLocalPosition(UASInterface* uav, double x, double y, double z, quint64 usec);
    void receiveUAVGlobalPosition(UASInterface* uav, double lon, double lat, double alt, quint64 usec);
    void receiveUAVSpeed(UASInterface* uav, double x, double y, double z, quint64 usec);
	void receiveUAVAttitude(UASInterface* uav, double roll, double pitch, double yaw, quint64 usec);
	void receiveUAVAttitudeSpeed(int uas, double rollspeed, double pitchspeed, double yawspeed, quint64 usec);
    void receiveThrust(UASInterface* uas, double thrust);
	void receiveLeaderStatus();
	void receiveObstacleStatus();
	void toggleSimulation();

protected:
	void changeEvent(QEvent *e);
	UAS* uas;
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
	Ui::UAVControlInstance *m_ui;
	bool simStarted;
};

#endif //_UAVCTRLINST_H_
