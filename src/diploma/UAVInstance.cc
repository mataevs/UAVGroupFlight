/**
 * @file
 *   @brief UI component for UAV info display inside UAVAlgoCenter implementation
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */


#include "UAVInstance.h"
#include "QGC.h"
#include "ui_UAVInstance.h"
#include <QMenu>


UAVInstance::UAVInstance(UASInterface* uas, QWidget* parent) :
		QWidget(parent),
		uas(uas),
		x(0),
		y(0),
		z(0),
		lat(0),
		lon(0),
		alt(0),
		roll(0),
		pitch(0),
		yaw(0),
		groundDistance(0),
		m_ui(new Ui::UAVInstance)
{
	m_ui->setupUi(this);

	if (uas) {
		connect(uas, SIGNAL(nameChanged(QString)), this, SLOT(receiveName(QString)));
		connect(uas, SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVLocalPosition(UASInterface*,double,double,double,quint64)));
		connect(uas, SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVGlobalPosition(UASInterface*,double,double,double,quint64)));
		connect(uas, SIGNAL(speedChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVSpeed(UASInterface*,double,double,double,quint64)));
		connect(uas, SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVAttitude(UASInterface*,double,double,double,quint64)));
		connect(uas, SIGNAL(attitudeSpeedChanged(int,double,double,double,quint64)), this, SLOT(receiveUAVAttitudeSpeed(int,double,double,double,quint64)));
        connect(uas, SIGNAL(thrustChanged(UASInterface*, double)), this, SLOT(receiveThrust(UASInterface*,double)));
    }

	if (uas)
		m_ui->uavName->setText(uas->getUASName());
}

UAVInstance::~UAVInstance() {
	delete m_ui;
}

void UAVInstance::receiveName(const QString& name) {
	//m_ui->uavName->setText(name);
}

void UAVInstance::receiveUAVLocalPosition(UASInterface* uas, double x, double y, double z, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->x = x;
	this->y = y;
	this->z = z;

	stringValue.sprintf("%.2f", this->x);
	m_ui->xPos->setText(stringValue);

	stringValue.sprintf("%.2f", this->y);
	m_ui->yPos->setText(stringValue);

	stringValue.sprintf("%.2f", this->z);
	m_ui->zPos->setText(stringValue);
}

void UAVInstance::receiveUAVGlobalPosition(UASInterface* uas, double lat, double lon, double alt, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->lat = lat;
	this->lon = lon;
	this->alt = alt;

	stringValue.sprintf("%.2f", this->lat);
	m_ui->lat->setText(stringValue);

	stringValue.sprintf("%.2f", this->lon);
	m_ui->lon->setText(stringValue);

	stringValue.sprintf("%.2f", this->alt);
	m_ui->alt->setText(stringValue);
}

void UAVInstance::receiveUAVSpeed(UASInterface* uas, double x, double y, double z, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->vx = x;
	this->vy = y;
	this->vz = z;
	
	stringValue.sprintf("%.2f", this->vx);
	m_ui->xSpeed->setText(stringValue);

	stringValue.sprintf("%.2f", this->vy);
	m_ui->ySpeed->setText(stringValue);

	stringValue.sprintf("%.2f", this->vz);
	m_ui->zSpeed->setText(stringValue);
}

void UAVInstance::receiveUAVAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;

	stringValue.sprintf("%.2f", this->roll);
	m_ui->roll->setText(stringValue);

	stringValue.sprintf("%.2f", this->pitch);
	m_ui->pitch->setText(stringValue);

	stringValue.sprintf("%.2f", this->yaw);
	m_ui->yaw->setText(stringValue);
}

void UAVInstance::receiveUAVAttitudeSpeed(int uas, double rollSpeed, double pitchSpeed, double yawSpeed, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	
	this->v_roll = rollSpeed;
	this->v_pitch = pitchSpeed;
	this->v_yaw = yawSpeed;

	stringValue.sprintf("%.2f", this->v_roll);
	m_ui->rollSpeed->setText(stringValue);

	stringValue.sprintf("%.2f", this->v_pitch);
	m_ui->pitchSpeed->setText(stringValue);

	stringValue.sprintf("%.2f", this->v_yaw);
	m_ui->yawSpeed->setText(stringValue);
}

void UAVInstance::receiveThrust(UASInterface *uas, double thrust) {
    Q_UNUSED(uas);
    this->thrust = thrust;
}

void UAVInstance::changeEvent(QEvent* e) {
	QWidget::changeEvent(e);
}

double UAVInstance::getThrust() {
    return this->thrust;
}
