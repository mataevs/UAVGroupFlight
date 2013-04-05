/**
 * @file
 *   @brief UI component implementation for UAV info display inside UAVControl
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */


#include "UAVControlInstance.h"
#include "QGC.h"
#include "ui_UAVControlInstance.h"
#include "UASManager.h"
#include <QMenu>


UAVControlInstance::UAVControlInstance(UAS* uas, QWidget* parent) :
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
		m_ui(new Ui::UAVControlInstance),
		simStarted(false)
{
	m_ui->setupUi(this);

	if (uas) {
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(nameChanged(QString)), this, SLOT(receiveName(QString)));
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(localPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVLocalPosition(UASInterface*,double,double,double,quint64)));
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(globalPositionChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVGlobalPosition(UASInterface*,double,double,double,quint64)));
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(speedChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVSpeed(UASInterface*,double,double,double,quint64)));
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(attitudeChanged(UASInterface*,double,double,double,quint64)), this, SLOT(receiveUAVAttitude(UASInterface*,double,double,double,quint64)));
		connect(dynamic_cast<UASInterface*>(uas), SIGNAL(attitudeSpeedChanged(int,double,double,double,quint64)), this, SLOT(receiveUAVAttitudeSpeed(int,double,double,double,quint64)));
        connect(dynamic_cast<UASInterface*>(uas), SIGNAL(thrustChanged(UASInterface*, double)), this, SLOT(receiveThrust(UASInterface*,double)));
		connect(m_ui->simulationButton, SIGNAL(clicked()), this, SLOT(toggleSimulation()));
		connect(m_ui->leaderPushButton, SIGNAL(clicked()), this, SLOT(receiveLeaderStatus()));
		connect(m_ui->obstaclePushButton, SIGNAL(clicked()), this, SLOT(receiveObstacleStatus()));
    }

	if (uas)
		m_ui->uavIdLabel->setText(uas->getUASName());

	qDebug() << "UAV Instance created." << uas->getUASName();

	this->setVisible(true);
	this->m_ui->simulationButton->setVisible(true);
}

UAVControlInstance::~UAVControlInstance() {
	delete m_ui;
}

void UAVControlInstance::changeStartCoordinates(float lat, float lon, float alt) {
	m_ui->latStartSpinBox->setValue(lat);
	m_ui->lonStartSpinBox->setValue(lon);
	m_ui->altStartSpinBox->setValue(alt);
}

float UAVControlInstance::getStartLatitude() {
	return m_ui->latStartSpinBox->value();
}

float UAVControlInstance::getStartLongitude() {
	return m_ui->lonStartSpinBox->value();
}

float UAVControlInstance::getStartAltitude() {
	return m_ui->altStartSpinBox->value();
}

void UAVControlInstance::toggleSimulation() {

	double homeLatitude, homeLongitude, homeAltitude;

	if (!simStarted) {
		homeLatitude = UASManager::instance()->getHomeLatitude();
		homeLongitude = UASManager::instance()->getHomeLongitude();
		homeAltitude = UASManager::instance()->getHomeAltitude();

		UASManager::instance()->setHomePosition(m_ui->latStartSpinBox->value(), m_ui->lonStartSpinBox->value(), m_ui->altStartSpinBox->value());
		this->uas->startHil();
		UASManager::instance()->setHomePosition(homeLatitude, homeLongitude, homeAltitude);
		m_ui->simulationButton->setText(QString("Stop"));

		simStarted = true;
	}
	else {
		this->uas->stopHil();
		simStarted = false;
		m_ui->simulationButton->setText(QString("Simulate"));
	}
}

void UAVControlInstance::receiveLeaderStatus() {
	(dynamic_cast<UASAgent*>(this->uas))->setLeaderStatus();
}

void UAVControlInstance::receiveObstacleStatus() {
	(dynamic_cast<UASAgent*>(this->uas))->setObstacleStatus();
}

void UAVControlInstance::receiveName(const QString& name) {
	m_ui->uavIdLabel->setText(name);
}

void UAVControlInstance::receiveUAVLocalPosition(UASInterface* uas, double x, double y, double z, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->x = x;
	this->y = y;
	this->z = z;
}

void UAVControlInstance::receiveUAVGlobalPosition(UASInterface* uas, double lat, double lon, double alt, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->lat = lat;
	this->lon = lon;
	this->alt = alt;

	stringValue.sprintf("%.5f", this->lat);
    m_ui->latLabel->setText(stringValue);

	stringValue.sprintf("%.5f", this->lon);
    m_ui->lonLabel->setText(stringValue);

	stringValue.sprintf("%.5f", this->alt);
    m_ui->altLabel->setText(stringValue);
}

void UAVControlInstance::receiveUAVSpeed(UASInterface* uas, double x, double y, double z, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->vx = x;
	this->vy = y;
	this->vz = z;
	
	stringValue.sprintf("%.2f", this->vx);
	m_ui->xSpeedLabel->setText(stringValue);

	stringValue.sprintf("%.2f", this->vy);
	m_ui->ySpeedLabel->setText(stringValue);

	stringValue.sprintf("%.2f", this->vz);
	m_ui->zSpeedLabel->setText(stringValue);
}

void UAVControlInstance::receiveUAVAttitude(UASInterface* uas, double roll, double pitch, double yaw, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	this->roll = roll;
	this->pitch = pitch;
	this->yaw = yaw;
}

void UAVControlInstance::receiveUAVAttitudeSpeed(int uas, double rollSpeed, double pitchSpeed, double yawSpeed, quint64 usec) {
	QString stringValue;
	
	Q_UNUSED(uas);
	Q_UNUSED(usec);
	
	this->v_roll = rollSpeed;
	this->v_pitch = pitchSpeed;
	this->v_yaw = yawSpeed;
}

void UAVControlInstance::receiveThrust(UASInterface *uas, double thrust) {
    Q_UNUSED(uas);
    this->thrust = thrust;
}

void UAVControlInstance::changeEvent(QEvent* e) {
	QWidget::changeEvent(e);
}

double UAVControlInstance::getThrust() {
    return this->thrust;
}
