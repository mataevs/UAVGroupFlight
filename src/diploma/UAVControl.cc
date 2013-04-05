/**
 * @file
 *   @brief UI module for UAV Instances control and brief summary
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

#include "UAVControl.h"


UAVControl::UAVControl(QWidget* parent) : QWidget(parent), m_ui(new Ui::UAVControl)
{

	m_ui->setupUi(this);

	setMinimumSize(600, 200);

    listLayout = new QVBoxLayout(m_ui->uavListView);
	listLayout->setMargin(0);
	listLayout->setSpacing(3);
	listLayout->setAlignment(Qt::AlignTop);
    m_ui->uavListView->setLayout(listLayout);
	setObjectName("UAV Control Widget");

    uavInstances = QMap<UASInterface*, UAVControlInstance*>();

	this->setVisible(true);

	connect(m_ui->loadScenarioButton, SIGNAL(clicked()), this, SLOT(loadScenario()));
	connect(m_ui->saveScenarioButton, SIGNAL(clicked()), this, SLOT(saveScenario()));
	connect(m_ui->simActionButton, SIGNAL(clicked()), this, SLOT(toggleSimulation()));
    connect(m_ui->addUavButton, SIGNAL(clicked()), this, SLOT(addUAV()));
	connect(m_ui->changeParamsButton, SIGNAL(clicked()), this, SLOT(changeCollAvParams()));

    m_ui->addUavButton->setEnabled(true);
	m_ui->simActionButton->setVisible(false);
	m_ui->saveScenarioButton->setVisible(false);

	simStarted = false;

	timer = new QTimer(this);

	logger = new UAVLogging();
}

UAVControl::~UAVControl()
{
	delete m_ui;
	delete timer;
}

void UAVControl::changeEvent(QEvent *e)
{
	QWidget::changeEvent(e);
}

void UAVControl::addUAV()
{
	m_ui->simActionButton->setVisible(true);
	m_ui->saveScenarioButton->setVisible(true);

	UASInterface* uavExistingId = UASManager::instance()->getUASForId(m_ui->uavIdBox->value());
	if (uavExistingId != NULL)
		return;

	MAVLinkProtocol *mavLinkProt = new MAVLinkProtocol();
	UASInterface* newUASInstance = new UASAgent(mavLinkProt, m_ui->uavIdBox->value());
    UASManager::instance()->addUAS(newUASInstance);

	if (!uavInstances.contains(newUASInstance))
	{
        uavInstances.insert(newUASInstance, new UAVControlInstance(dynamic_cast<UAS*>(newUASInstance), this));
        listLayout->addWidget(uavInstances.value(newUASInstance));

		m_ui->uavIdBox->setValue(m_ui->uavIdBox->value() + 1);
	}

	logger->addUAV(newUASInstance);
}

void UAVControl::changeCollAvParams()
{
	UASAgent::radius_ = m_ui->radiusSpinBox->value();
	UASAgent::timeHorizon_ = m_ui->timeFrameSpinBox->value();
	UASAgent::enabled_ = m_ui->colAvEnabledCheckBox->isChecked();
	UASAgent::leaderSyncZone_ = m_ui->syncZoneSpinBox->value();
}

void UAVControl::removeUAV(UASInterface* uav)
{
	UAVControlInstance* uavControlInstance = uavInstances.value(uav);
	
	if (uavControlInstance) {
		listLayout->removeWidget(uavInstances.value(uav));
		uavInstances.remove(uav);
		delete uavControlInstance;
	}

	logger->removeUAV(uav);
}


void UAVControl::loadScenario() {
	qDebug() << "Load scenario called.";

	QString fileName = QFileDialog::getOpenFileName(this, tr("Load File"), ".", tr("Waypoint File (*.txt)"));
	if (fileName == NULL)
		return;
	
	QFile file(fileName);
	
	if (!file.open(QIODevice::ReadOnly)) {
		QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Could not open file!");
        msgBox.setInformativeText("The specified file could not be opened.");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
		return;
	}
	QTextStream inputSource(&file);

	int aircraftCount;

	inputSource >> aircraftCount;

	int id;
	float lat, lon, alt;
	//Read the aircrafts' initial positions
	for (int i = 0; i < aircraftCount; i++) {
		inputSource >> id >> lat >> lon >> alt;
		qDebug() << id << lat << lon << alt;
		//Add a new uav
		m_ui->uavIdBox->setValue(id);
		this->addUAV();

		//Set the starting point
		UASInterface* uasInstance = UASManager::instance()->getUASForId(id);
		UAVControlInstance* uasControlInstance = uavInstances.value(uasInstance);
		uasControlInstance->changeStartCoordinates(lat, lon, alt);
	}

	//Get the instance of the leader uav
	int leaderId;
	inputSource >> leaderId;
	UAS* uasLeaderInstance = dynamic_cast<UAS*>(UASManager::instance()->getUASForId(leaderId));
	UAVControlInstance* leaderControlInstance = uavInstances.value(dynamic_cast<UASInterface*>(uasLeaderInstance));
	leaderControlInstance->receiveLeaderStatus();
	
	int waypointsCount;
	inputSource >> waypointsCount;
	//Read the waypoints
	for (int i = 0; i < waypointsCount; i++) {
		inputSource >> lat >> lon >> alt;
		qDebug() << "wp" << lat << lon << alt;
		Waypoint *wp = new Waypoint();
		wp->setLatitude(lat);
		wp->setLongitude(lon);
		wp->setAltitude(alt);
		uasLeaderInstance->getWaypointManager()->addWaypointEditable(wp);
	}

	file.close();
}


void UAVControl::saveScenario() {
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save File"), ".", tr("Waypoint File (*.txt)"));
	
	if (fileName == NULL)
		return;

	QFile file(fileName);
	
	if (!file.open(QIODevice::WriteOnly)) {
		QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Could not open file!");
        msgBox.setInformativeText("The specified file could not be opened.");
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
		return;
	}
	QTextStream outputSource(&file);

	QList<UASInterface*> uasList = UASManager::instance()->getUASList();

	outputSource << uasList.count() << endl;

	UASAgent* leaderAgent = NULL;

	for (int i = 0; i < uasList.count(); i++) {
		UASInterface* uasInterface = uasList.at(i);
		UASAgent* uasAgent = dynamic_cast<UASAgent*>(uasInterface);
		UAVControlInstance* uavControlInstance = uavInstances.value(uasInterface);
		
		outputSource << uasAgent->getUASID() << " " << uavControlInstance->getStartLatitude() << " " << uavControlInstance->getStartLongitude() << " " << uavControlInstance->getStartAltitude() << endl;

		if (uasAgent->isLeader()) {
			leaderAgent = uasAgent;
		}
	}

	outputSource << leaderAgent->getUASID() << endl;

	QVector<Waypoint*> waypoints = leaderAgent->getWaypointManager()->getWaypointEditableList();

	outputSource<< waypoints.count() << endl;

	for (int i = 0; i < waypoints.count(); i++) {
		outputSource << waypoints.at(i)->getLatitude() << " " << waypoints.at(i)->getLongitude() << " " << waypoints.at(i)->getAltitude() << endl;
	}

	file.close();
}

void UAVControl::toggleSimulation() {
	timer->setInterval(30000);
	timer->setSingleShot(true);
	connect(timer, SIGNAL(timeout()), this, SLOT(sendLeaderWaypoints()));

	for (int i = 0; i < UASManager::instance()->getUASList().count(); i++) {
		UASInterface* uasInterface = UASManager::instance()->getUASList().at(i);
		UAVControlInstance* uavControlInstance = uavInstances.value(uasInterface);
		
		uavControlInstance->toggleSimulation();

		if (simStarted == false)
			if (dynamic_cast<UASAgent*>(uasInterface)->isLeader())
				timer->start();
	}

	if (!simStarted) {
		simStarted = true;
		m_ui->simActionButton->setText("Stop sim");
	}
	else {
		simStarted = false;
		m_ui->simActionButton->setText("Start sim");
	}

	logger->setSimulationStatus(simStarted);
}

void UAVControl::sendLeaderWaypoints() {
	for (int i = 0; i < UASManager::instance()->getUASList().count(); i++) {
		UASInterface* uasInterface = UASManager::instance()->getUASList().at(i);
		UASAgent* uasAgent = dynamic_cast<UASAgent*>(uasInterface);

		if (uasAgent->isLeader()) {
			uasAgent->getWaypointManager()->writeWaypoints();
			break;
		}
	}
}