
#ifndef _UASAGENT_H_
#define _UASAGENT_H_

#include "UASInterface.h"
#include "UASManager.h"
#include "UAS.h"
#include <QHash>
#include <QList>
#include <QVector3D>
#include <QtCore/qmath.h>


#define APPROACH	1
#define	SYNC		2
#define COLLISION	3


#define NEIGHBOUR_AREA 5
#define SYNC_AREA 10


#define LEADER_RATIO 0.05


struct Line
{
	QVector3D point;
	QVector3D direction;
};

struct Plane
{
	QVector3D point;
	QVector3D normal;
};


class UASAgent : public UAS
{
	Q_OBJECT

public:
	UASAgent(MAVLinkProtocol* mavlink, int id = 0);
	~UASAgent();

	bool isLeader();
	bool isObstacle();
	QVector3D getPosition();
	QVector3D getSpeed();
	void registerExistingUAS(UASAgent* agent);
	void setLeaderStatus();
	void setObstacleStatus();

public slots:
	void addAgent(UASInterface* uas);
	void removeAgent(UASInterface* uas);
	void receiveAgentInfo(UASAgent* agent, QVector3D pos, QVector3D speed, float airspeed);
	void receiveChangedLeader(UASAgent* agent);

	void receiveHilState(uint64_t time_us, float roll, float pitch, float yaw, float rollspeed,
                       float pitchspeed, float yawspeed, float lat, float lon, float alt,
                       float vx, float vy, float vz, float airspeed, int currentwp, QString rmgrStatus);

signals:
	void sendInfo(UASAgent* agent, QVector3D position, QVector3D speed, float airspeed);
	void emitNewLeader(UASAgent* agent);
	void agentLogStats(UASAgent* agent, QVector3D position, QVector3D speed, int status);

protected:
	void collisionAvoidance();
	void moveNewDirection();
	void moveCollisionAvoidMode();
	void moveSyncMode();
	void moveApproachMode();
	void moveObstacleMode();

	void changeHeadingFromSpeed(QVector3D speed, float airspeed);
	bool changeWaypoint();

	QList<UASAgent*> agents;
	QHash<UASAgent*, QVector3D> agentsPositions;
	QHash<UASAgent*, QVector3D> agentsSpeeds;
	QHash<UASAgent*, float> agentsAirspeeds;
	
	UASAgent* currentLeader;
	bool leader;
	bool obstacle;

	QString routeManagerStatus;
	int currentWaypoint;


	//===RVO variables===//
	QVector3D newVelocity_;
	QVector3D prefVelocity_;

	int nrInfoUpdates;
	bool collisionWithLeader;

	std::vector<Plane> orcaPlanes_;

public:
	static float timeHorizon_;
	static float radius_;
	static float timeStep_;
	static float maxSpeed_;
	static int leaderSyncZone_;
	static bool enabled_;
	static float leaderAirspeed_;
};

#endif //_UASAGENT_H_
