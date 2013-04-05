
#include "UAVLogging.h"

UAVLogging::UAVLogging() : QObject() {
	timer = new QTimer(this);
	agents = QList<UASAgent*>();
	agentsPositions = QHash<UASAgent*, QVector3D>();
	agentsSpeeds = QHash<UASAgent*, QVector3D>();
	agentsStatus = QHash<UASAgent*, int>();

	active = false;

	timer->setInterval(1000);
	connect(timer, SIGNAL(timeout()), this, SLOT(logData()));
}

UAVLogging::~UAVLogging() {
	delete timer;
}

void UAVLogging::receiveAgentStats(UASAgent* agent, QVector3D position, QVector3D speed, int state) {
	
	if (agents.contains(agent)) {
		agentsPositions.remove(agent);
		agentsSpeeds.remove(agent);
		agentsStatus.remove(agent);
		agentsPositions.insert(agent, position);
		agentsSpeeds.insert(agent, speed);
		agentsStatus.insert(agent, state);
	}
}

void UAVLogging::setSimulationStatus(bool activate) {
	if (activate) {
		logFileMoveState = new QFile("logState.csv");
		logFileMoveState->open(QIODevice::WriteOnly);

		logMoveState = new QTextStream(logFileMoveState);
		
		active = true;

		for (int i = 0; i < agents.count(); i++) {
			UASAgent* agent = agents.at(i);
			*logMoveState << agent->getUASName() << ",";
		}

		*logMoveState << "avgDistLeader," << "avgDist" << endl;

		timer->start();
	}
	else {
		logFileMoveState->close();
		delete logMoveState;
		delete logFileMoveState;

		timer->stop();

		active = false;
	}
}

void UAVLogging::logData() {

	for (int i = 0; i < agents.count(); i++) {
		UASAgent* agent = agents.at(i);
		*logMoveState << agentsStatus.value(agent) << ",";
	}


	float avgDist = 0, avgDistLeader = 0;

	for (int i = 0; i < agents.count() - 1; i++) {
		UASAgent* agent = agents.at(i);
		for (int j = i + 1; j < agents.count(); j++) {
			UASAgent* otherAgent = agents.at(j);

			QVector3D relPosition = agent->getPosition() - otherAgent->getPosition();
			relPosition.setX(relPosition.x() * 111100);
			relPosition.setY(relPosition.y() * 82400);
			float dist = std::sqrt(QVector3D::dotProduct(relPosition, relPosition));

			avgDist += dist;

			if (agent->isLeader() || otherAgent->isLeader())
				avgDistLeader += dist;
		}
	}

	avgDist = avgDist / (agents.count() * (agents.count() - 1));
	avgDistLeader = avgDistLeader / (agents.count() - 1);

	*logMoveState << avgDistLeader << "," << avgDist << endl;
}

void UAVLogging::addUAV(UASInterface* uas) {
	UASAgent* uasAgent = dynamic_cast<UASAgent*>(uas);
	agents.append(uasAgent);
	connect(uasAgent, SIGNAL(agentLogStats(UASAgent*, QVector3D, QVector3D, int)), this, SLOT(receiveAgentStats(UASAgent*, QVector3D, QVector3D, int)));

	qDebug() << "UAV added.";
}

void UAVLogging::removeUAV(UASInterface* uas) {
	Q_UNUSED(uas);
}