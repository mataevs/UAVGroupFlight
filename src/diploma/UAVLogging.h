

#ifndef _UAV_LOG_
#define _UAV_LOG_

#include "UASInterface.h"
#include "UASManager.h"
#include "UAS.h"
#include "UASAgent.h"
#include <QHash>
#include <QList>
#include <QVector3D>
#include <QObject>

class UAVLogging : public QObject {

	Q_OBJECT
public:
	UAVLogging();
	~UAVLogging();
	void setSimulationStatus(bool activate);
	void addUAV(UASInterface* uas);
	void removeUAV(UASInterface* uas);

public slots:
	void receiveAgentStats(UASAgent* agent, QVector3D position, QVector3D speed, int state);
	void logData();

private:
	QList<UASAgent*> agents;
	QHash<UASAgent*, QVector3D> agentsPositions;
	QHash<UASAgent*, QVector3D> agentsSpeeds;
	QHash<UASAgent*, int> agentsStatus;
	QTimer* timer;
	bool active;

	QFile* logFileMoveState;
	QTextStream* logMoveState;
};

#endif //_UAV_LOG_