
#include "UASAgent.h"
#include <cstddef>
#include <vector>


float UASAgent::timeHorizon_ = 5;
float UASAgent::radius_ = 30;
float UASAgent::timeStep_ = 0.5;
float UASAgent::maxSpeed_ = 35;
int	UASAgent::leaderSyncZone_ = 6;
bool UASAgent::enabled_ = true;
float UASAgent::leaderAirspeed_ = 25;

float absSq(QVector3D v) {
	return QVector3D::dotProduct(v, v);
}

float sqr(float value) {
	return value * value;
}


static const float RVO_EPSILON = 0.00001f;

/*!
 *  @brief		Solves a one-dimensional linear program on a specified line subject to linear constraints defined by planes and a spherical constraint.
 *  @param		planes				Planes defining the linear constraints.
 *  @param		planeNo				The plane on which the line lies.
 *  @param		line				The line on which the 1-d linear program is solved
 *  @param		radius				The radius of the spherical constraint.
 *  @param		optVelocity			The optimization velocity.
 *  @param		directionOpt		True if the direction should be optimized.
 *  @param		result				A reference to the result of the linear program.
 *  @returns	True if successful.
 */
bool linearProgram1(const std::vector<Plane>& planes, size_t planeNo, const Line& line, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result);

/*!
 *  @brief      Solves a two-dimensional linear program on a specified plane subject to linear constraints defined by planes and a spherical constraint.
 *  @param      planes				Planes defining the linear constraints.
 *  @param      planeNo				The plane on which the 2-d linear program is solved
 *  @param      radius				The radius of the spherical constraint.
 *  @param      optVelocity			The optimization velocity.
 *  @param      directionOpt		True if the direction should be optimized.
 *  @param      result				A reference to the result of the linear program.
 *  @returns    True if successful.
 */
bool linearProgram2(const std::vector<Plane>& planes, size_t planeNo, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result);


/*!
 *  @brief      Solves a three-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
 *  @param      planes				Planes defining the linear constraints.
 *  @param      radius				The radius of the spherical constraint.
 *  @param      optVelocity			The optimization velocity.
 *  @param      directionOpt		True if the direction should be optimized.
 *  @param      result				A reference to the result of the linear program.
 *  @returns    The number of the plane it fails on, and the number of planes if successful.
 */
size_t linearProgram3(const std::vector<Plane>& planes, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result);

/*!
 *  @brief      Solves a four-dimensional linear program subject to linear constraints defined by planes and a spherical constraint.
 *  @param      planes			Planes defining the linear constraints.
 *  @param      beginLine		The plane on which the 3-d linear program failed.
 *  @param      radius			The radius of the spherical constraint.
 *  @param      result			A reference to the result of the linear program.
 */
void linearProgram4(const std::vector<Plane>& planes, size_t beginPlane, float radius, QVector3D& result);


float horizontalSpeedToHeading(float vx, float vy) {
	float angle = qAtan(vy / vx) * 180 / 3.141592;
	if (vx < 0)
		return 180 + angle; 
	return angle;
}


UASAgent::UASAgent(MAVLinkProtocol* mavlink, int id) :
    UAS(mavlink, id)
{
	agents = QList<UASAgent*>();
	agentsPositions = QHash<UASAgent*, QVector3D>();
	agentsSpeeds = QHash<UASAgent*, QVector3D>();
	agentsAirspeeds = QHash<UASAgent*, float>();

	leader = false;
	obstacle = false;
	currentLeader = NULL;

	nrInfoUpdates = 0;

	//Connect to the UASManager slots for capturing creation and destruction of agents
	connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(addAgent(UASInterface*)));
	//connect(UASManager::instance(), SLOT(removeUAS(UASInterface*)), this, SLOT(addAgent(UASInterface*)));
}

UASAgent::~UASAgent()
{

}


void UASAgent::receiveHilState(uint64_t time_us, float roll, float pitch, float yaw, float rollspeed,
                       float pitchspeed, float yawspeed, float lat, float lon, float alt,
                       float vx, float vy, float vz, float airspeed, int currentwp, QString rmgrStatus) {
	
	this->sendHilState(time_us, roll, pitch, yaw, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, airspeed, currentwp, rmgrStatus);

	this->currentWaypoint = currentwp;
	this->routeManagerStatus = rmgrStatus;
	
	nrInfoUpdates++;

	//Preferred velocity is the velocity of the leader if the agent is in SYNC and is not the leader
	prefVelocity_ = QVector3D(vx, vy, -1 * vz);

	//Each three position updates, inform the other agents and run the coordination algorithm
	if (nrInfoUpdates == 3) {
		//Inform the other agents of the agent's position
		QVector3D position(lat, lon, alt);
		QVector3D speed(vx, vy, vz);

		emit sendInfo(this, position, speed, airspeed);
		
		nrInfoUpdates = 0;

		if (obstacle) {
			moveObstacleMode();
			return;
		}

		if (enabled_) {
			//Run the collision avoidance algorithm
			collisionAvoidance();
		}
		else {
			newVelocity_ = prefVelocity_;
		}

		moveNewDirection();
	}
}


void UASAgent::collisionAvoidance()
{
	collisionWithLeader = false;

	//----ORCA----//
	orcaPlanes_.clear();
	const float invTimeHorizon = 1.0f / timeHorizon_;

	for (int i = 0; i < agents.count(); i++) {
		UASAgent* other = agents.at(i);

		QVector3D relativePosition = agentsPositions.value(other) - getPosition();
		relativePosition.setX(relativePosition.x() * 111100);
		relativePosition.setY(relativePosition.y() * 82400);

		float distance = std::sqrt(QVector3D::dotProduct(relativePosition, relativePosition));
		float distSq = QVector3D::dotProduct(relativePosition, relativePosition);

		//Consider only the neighbours of the UAV
		if (distance > 2 * timeHorizon_ * maxSpeed_)
			continue;

		QVector3D relativeVelocity = getSpeed() - agentsSpeeds.value(other);
		relativeVelocity.setZ(-1 * relativeVelocity.z());

		float combinedRadius = radius_ + other->radius_;
		float combinedRadiusSq = combinedRadius * combinedRadius;

		Plane plane;
		QVector3D u;

		if (distSq > combinedRadiusSq) {
			/* No collision */
			QVector3D w = relativeVelocity - invTimeHorizon * relativePosition;
			/* Vector from cutoff center to relative velocity. */
			float wLengthSq = QVector3D::dotProduct(w, w);

			float dotProduct1 = QVector3D::dotProduct(w, relativePosition);

			if (dotProduct1 < 0.0f && (dotProduct1 * dotProduct1) > combinedRadiusSq * wLengthSq) {
				/* Project on cut-off center */
				float wLength = std::sqrt(wLengthSq);
				QVector3D unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * invTimeHorizon - wLength) * unitW;
			}
			else {
				/* Project on cone */
				float A = distSq;
				float B = QVector3D::dotProduct(relativePosition, relativeVelocity);
				QVector3D tmp = QVector3D::crossProduct(relativePosition, relativeVelocity);
				float C = QVector3D::dotProduct(relativeVelocity, relativeVelocity) - 
					QVector3D::dotProduct(tmp, tmp) / (distSq - combinedRadiusSq);

				float t = (B + std::sqrt(B * B - A * C)) / A;
				QVector3D w = relativeVelocity - t * relativePosition;
				float wLength = std::sqrt(QVector3D::dotProduct(w, w));
				QVector3D unitW = w / wLength;

				plane.normal = unitW;
				u = (combinedRadius * t - wLength) * unitW;
			}
		}
		else {
			/* Collision */

			float invTimeStep = 1.0f / timeStep_;

			QVector3D w = relativeVelocity - invTimeStep * relativePosition;
			float wLength = std::sqrt(QVector3D::dotProduct(w, w));
			QVector3D unitW = w / wLength;

			plane.normal = unitW;
			u = (combinedRadius * invTimeStep - wLength) * unitW;
		}

		QVector3D speed = getSpeed();
		speed.setZ(-1 * speed.z());


		//The leader has lower responsibility for avoiding a collision
		if (other->isObstacle())
			plane.point = speed + u;
		else if (currentLeader && this == currentLeader)
			plane.point = speed + LEADER_RATIO * u;
		else if (currentLeader && other == currentLeader) {
			plane.point = speed + (1 - LEADER_RATIO) * u;
			collisionWithLeader = true;
		}
		else
			plane.point = speed + 0.5 * u;

		orcaPlanes_.push_back(plane);
	}

	if (orcaPlanes_.size() != 0) {
		size_t planeFail = linearProgram3(orcaPlanes_, maxSpeed_, prefVelocity_, false, newVelocity_);

		if (planeFail < orcaPlanes_.size()) {
			linearProgram4(orcaPlanes_, planeFail, maxSpeed_, newVelocity_);
		}
	}
	else {
		newVelocity_ = prefVelocity_;
	}
}


void UASAgent::moveCollisionAvoidMode() {
	float heading = horizontalSpeedToHeading(newVelocity_.x(), newVelocity_.y());
	float altitude = this->altitude - newVelocity_.z() * 3;

	//qDebug() << "Collision avoidance: true airspeed = " << std::sqrt(QVector3D::dotProduct(newVelocity_, newVelocity_));
	//qDebug() << "assigned airspeed = " << airspeed;

	airspeed = std::sqrt(QVector3D::dotProduct(newVelocity_, newVelocity_));

	if (collisionWithLeader)
		airspeed = agentsAirspeeds.value(currentLeader) * 0.75;

	transmitHilHeadingCommand(heading, altitude, airspeed, QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"));

	qDebug() << "Collision avoidance mode." << getUASName();

	emit agentLogStats(this, getPosition(), getSpeed(), 2);
}


void UASAgent::moveSyncMode() {
	float heading = horizontalSpeedToHeading(newVelocity_.x(), newVelocity_.y());
	float altitude = agentsPositions.value(currentLeader).z();

	float leaderAirspeed = agentsAirspeeds.value(currentLeader);

	//Check if the agent is in front of the leader
	QVector3D leaderPosition = agentsPositions.value(currentLeader);
	QVector3D leaderVelocity = agentsSpeeds.value(currentLeader);

	QVector3D relativePosition = getPosition() - leaderPosition;
	relativePosition.setX(relativePosition.x() * 111100);
	relativePosition.setY(relativePosition.y() * 82400);

	float speedAngle = horizontalSpeedToHeading(leaderVelocity.x(), leaderVelocity.y());
	float relativePositionAngle = horizontalSpeedToHeading(relativePosition.x(), relativePosition.y());

	//If it is in front of the leader, should move slower so the leader can catch up
	float relativeAngle = std::abs(speedAngle - relativePositionAngle);
	if (relativeAngle < 100 || relativeAngle > 260) {
		//qDebug() << "Slow down" << getUASName();
		leaderAirspeed *= 0.8;
	}

	transmitHilHeadingCommand(heading, altitude, leaderAirspeed, QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"));

	qDebug() << "Sync mode." << relativeAngle << getUASName();

	emit agentLogStats(this, getPosition(), getSpeed(), 1);
}


void UASAgent::moveObstacleMode() {
	qDebug() << "Obstacle mode." << getUASName();
	if (!changeWaypoint())
		transmitHilHeadingCommand(45, 200, leaderAirspeed_, QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"));
}

void UASAgent::moveApproachMode() {
	if (currentLeader && currentLeader != this) {
		float leaderAirspeed = agentsAirspeeds.value(currentLeader);

		QVector3D leaderPos = agentsPositions.value(currentLeader);

		transmitHilRouteManagerCommand(QString("@CLEAR"), NULL);
		QString state("@INSERT-1:%1,%2@%3");
		state = state.arg(leaderPos.y()).arg(leaderPos.x()).arg(leaderPos.z() * 3.2808);
		transmitHilRouteManagerCommand(state, QString("true"));
		transmitHilSpeedRouteManagerCommand(1.5 * leaderAirspeed, QString("@ACTIVATE"), QString("true"));
		transmitHilSpeedRouteManagerCommand(1.5 * leaderAirspeed, QString("@JUMPTO0"), QString("true"));
	}
	//Daca nu e definit un lider, sau agentul e lider, e instruit sa mearga in sus
	else {
		if (currentLeader == this) { 
			if (!changeWaypoint())
				transmitHilHeadingCommand(45, 200, leaderAirspeed_, QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"));
		}
		else {
			transmitHilHeadingCommand(45, 200, leaderAirspeed_, QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"));
		}
	}

	qDebug() << "Approach mode." << getUASName();

	emit agentLogStats(this, getPosition(), getSpeed(), 0);
}


void UASAgent::moveNewDirection()
{
	
	if (newVelocity_ != prefVelocity_) {
		moveCollisionAvoidMode();
		
		return;
	}
		
	if (currentLeader && currentLeader != this) {
		QVector3D leaderVelocity = agentsSpeeds.value(currentLeader);
		QVector3D leaderPosition = agentsPositions.value(currentLeader);

		QVector3D relativeLeaderPosition = leaderPosition - getPosition();
		relativeLeaderPosition.setX(relativeLeaderPosition.x() * 111100);
		relativeLeaderPosition.setY(relativeLeaderPosition.y() * 82400);

		float distanceToLeader = std::sqrt(QVector3D::dotProduct(relativeLeaderPosition, relativeLeaderPosition));

		//Daca distanta pana la lider e mai mica decat syncZone * radius, incearca sa mearga alaturi de el
		if (distanceToLeader < leaderSyncZone_ * radius_) {
			newVelocity_ = agentsSpeeds.value(currentLeader);
			moveSyncMode();
		}
		else {
			moveApproachMode();
		}
	}
	else {
		moveApproachMode();
	}
}

bool UASAgent::changeWaypoint() {
	QVector<Waypoint*> waypoints = this->waypointManager.getWaypointEditableList();

	if (waypoints.count() == 0)
		return false;

	if (currentWaypoint == -1) {
		transmitHilSpeedRouteManagerCommand(leaderAirspeed_, QString("@ACTIVATE"), QString("true"));
		QString state("@JUMP0");
		transmitHilSpeedRouteManagerCommand(leaderAirspeed_, state, QString("true"));

		return true;
	}
	else {
		if (currentWaypoint >= waypoints.count())
			currentWaypoint = 0;

		Waypoint* wp = waypoints.at(currentWaypoint);
		float latitude = (wp->getLatitude() - this->latitude) * 111100;
		float longitude = (wp->getLongitude() - this->longitude) * 82400;
		float altitude = wp->getAltitude() - this->altitude;
		float distance = std::sqrt(latitude * latitude + longitude * longitude + altitude * altitude);

		if (distance < timeHorizon_ * radius_) {
			if (currentWaypoint == waypoints.count() - 1) {
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, QString("@ACTIVATE"), QString("true"));
				QString state("@JUMP0");
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, state, QString("true"));

				return true;
			}
			else {
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, QString("@ACTIVATE"), QString("true"));
				QString state("@JUMP%1");
				state = state.arg(currentWaypoint + 1);
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, state, QString("true"));
				
				return true;
			}
		}
		else {
			if (routeManagerStatus.compare("true") == 0)
				return true;
			else {
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, QString("@ACTIVATE"), QString("true"));
				QString state("@JUMP%1");
				state = state.arg(currentWaypoint);
				transmitHilSpeedRouteManagerCommand(leaderAirspeed_, state, QString("true"));

				return true;
			}
		}
	}
}

QVector3D UASAgent::getPosition()
{
	return QVector3D(this->latitude, this->longitude, this->altitude);
}

QVector3D UASAgent::getSpeed()
{
	return QVector3D(this->speedX, this->speedY, this->speedZ);
}

//Signal received from UASManager - a new UAS was created
void UASAgent::addAgent(UASInterface* uas)
{
	UASAgent* agent = dynamic_cast<UASAgent*>(uas);

	if (!agents.contains(agent) && agent != this)
	{
		//Add the new UAS to the agents list
		agents.append(agent);

		agent->registerExistingUAS(this);

		//Make the bidirectional signal communication between this UAS and the newly created UAS
		//Info sending and receiving
		connect(this, SIGNAL(sendInfo(UASAgent*, QVector3D, QVector3D, float)), agent, SLOT(receiveAgentInfo(UASAgent*, QVector3D, QVector3D, float)));
		connect(agent, SIGNAL(sendInfo(UASAgent*, QVector3D, QVector3D, float)), this, SLOT(receiveAgentInfo(UASAgent*, QVector3D, QVector3D, float)));
 		
		//Leader inform sending and receiving
		connect(this, SIGNAL(emitNewLeader(UASAgent*)), agent, SLOT(receiveChangedLeader(UASAgent*)));
		connect(agent, SIGNAL(emitNewLeader(UASAgent*)), this, SLOT(receiveChangedLeader(UASAgent*)));

		if (currentLeader && currentLeader == this) {
			emit emitNewLeader(this);
		}
	}
}


//Signal received from UASManager - a UAS was destroyed
void UASAgent::removeAgent(UASInterface* uas)
{
	UASAgent* agent = dynamic_cast<UASAgent*>(uas);

	if (agents.contains(agent)) {
		//Remove from the list
		agents.removeOne(agent);
		//Remove the last known UAS infos
		agentsPositions.remove(agent);
		agentsSpeeds.remove(agent);

		//Destroy the signal communication between the two agents
		disconnect(this, SIGNAL(sendInfo(UASAgent*, QVector3D, QVector3D, float)), agent, SLOT(receiveAgentInfo(UASAgent*, QVector3D, QVector3D, float)));
		disconnect(agent, SIGNAL(sendInfo(UASAgent*, QVector3D, QVector3D, float)), this, SLOT(receiveAgentInfo(UASAgent*, QVector3D, QVector3D, float)));

		disconnect(this, SIGNAL(emitNewLeader(UASAgent*)), agent, SLOT(receiveChangedLeader(UASAgent*)));
		disconnect(agent, SIGNAL(emitNewLeader(UASAgent*)), this, SLOT(receiveChangedLeader(UASAgent*)));
	}
}

void UASAgent::receiveAgentInfo(UASAgent* agent, QVector3D pos, QVector3D speed, float airspeed)
{
	if (agents.contains(agent)) {
		agentsPositions.remove(agent);
		agentsSpeeds.remove(agent);
		agentsAirspeeds.remove(agent);
		agentsPositions.insert(agent, pos);
		agentsSpeeds.insert(agent, speed);
		agentsAirspeeds.insert(agent, airspeed);
	}
}


void UASAgent::registerExistingUAS(UASAgent* agent)
{
	if (!agents.contains(agent))
		agents.append(agent);
}

void UASAgent::receiveChangedLeader(UASAgent* agent)
{
	currentLeader = agent;
	if (agent == NULL || agent != this)
		leader = false;
}

bool UASAgent::isLeader()
{
	return leader;
}

bool UASAgent::isObstacle() {
	return obstacle;
}

void UASAgent::setLeaderStatus() {
	leader = true;
	currentLeader = this;
	emit emitNewLeader(this);
}

void UASAgent::setObstacleStatus() {
	obstacle = true;
	if (currentLeader == this) {
		emit emitNewLeader(NULL);
		leader = false;
		currentLeader = NULL;
	}
}


bool linearProgram1(const std::vector<Plane>& planes, size_t planeNo, const Line& line, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result)
{
	const float dotProduct = QVector3D::dotProduct(line.point, line.direction);
	const float discriminant = sqr(dotProduct) + sqr(radius) - absSq(line.point);

	if (discriminant < 0.0f) {
		/* Max speed sphere fully invalidates line. */
		return false;
	}

	const float sqrtDiscriminant = std::sqrt(discriminant);

	float tLeft = -dotProduct - sqrtDiscriminant;

	float tRight = -dotProduct + sqrtDiscriminant;

	for (size_t i = 0; i < planeNo; ++i) {
		const float numerator = QVector3D::dotProduct((planes[i].point - line.point), planes[i].normal);
		const float denominator = QVector3D::dotProduct(line.direction, planes[i].normal);

		if (sqr(denominator) <= RVO_EPSILON) {
			/* Lines line is (almost) parallel to plane i. */
			if (numerator > 0.0f) {
				return false;
			}
			else {
				continue;
			}
		}

		const float t = numerator / denominator;

		if (denominator >= 0.0f) {
			/* Plane i bounds line on the left. */
			tLeft = std::max(tLeft, t);
		}
		else {
			/* Plane i bounds line on the right. */
			tRight = std::min(tRight, t);
		}

		if (tLeft > tRight) {
			return false;
		}
	}

	if (directionOpt) {
		/* Optimize direction. */
		if (QVector3D::dotProduct(optVelocity, line.direction) > 0.0f) {
			/* Take right extreme. */
			result = line.point + tRight * line.direction;
		}
		else {
			/* Take left extreme. */
			result = line.point + tLeft * line.direction;
		}
	}
	else {
		/* Optimize closest point. */
		const float t = QVector3D::dotProduct(line.direction, (optVelocity - line.point));

		if (t < tLeft) {
			result = line.point + tLeft * line.direction;
		}
		else if (t > tRight) {
			result = line.point + tRight * line.direction;
		}
		else {
			result = line.point + t * line.direction;
		}
	}

	return true;
}


bool linearProgram2(const std::vector<Plane>& planes, size_t planeNo, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result)
{
	const float planeDist = QVector3D::dotProduct(planes[planeNo].point, planes[planeNo].normal);
	const float planeDistSq = sqr(planeDist);
	const float radiusSq = sqr(radius);

	if (planeDistSq > radiusSq) {
		/* Max speed sphere fully invalidates plane planeNo. */
		return false;
	}

	const float planeRadiusSq = radiusSq - planeDistSq;

	const QVector3D planeCenter = planeDist * planes[planeNo].normal;

	if (directionOpt) {
		/* Project direction optVelocity on plane planeNo. */
		const QVector3D planeOptVelocity = optVelocity - QVector3D::dotProduct(optVelocity, planes[planeNo].normal) * planes[planeNo].normal;
		const float planeOptVelocityLengthSq = absSq(planeOptVelocity);

		if (planeOptVelocityLengthSq <= RVO_EPSILON) {
			result = planeCenter;
		}
		else {
			result = planeCenter + std::sqrt(planeRadiusSq / planeOptVelocityLengthSq) * planeOptVelocity;
		}
	}
	else {
		/* Project point optVelocity on plane planeNo. */
		result = optVelocity + QVector3D::dotProduct(planes[planeNo].point - optVelocity, planes[planeNo].normal) * planes[planeNo].normal;

		/* If outside planeCircle, project on planeCircle. */
		if (absSq(result) > radiusSq) {
			const QVector3D planeResult = result - planeCenter;
			const float planeResultLengthSq = absSq(planeResult);
			result = planeCenter + std::sqrt(planeRadiusSq / planeResultLengthSq) * planeResult;
		}
	}

	for (size_t i = 0; i < planeNo; ++i) {
		if (QVector3D::dotProduct(planes[i].normal, (planes[i].point - result)) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */

			/* Compute intersection line of plane i and plane planeNo. */
			QVector3D crossProduct = QVector3D::crossProduct(planes[i].normal, planes[planeNo].normal);

			if (absSq(crossProduct) <= RVO_EPSILON) {
				/* Planes planeNo and i are (almost) parallel, and plane i fully invalidates plane planeNo. */
				return false;
			}

			Line line;
			line.direction = crossProduct.normalized();
			const QVector3D lineNormal = QVector3D::crossProduct(line.direction, planes[planeNo].normal);
			line.point = planes[planeNo].point + (QVector3D::dotProduct(planes[i].point - planes[planeNo].point, planes[i].normal) / 
													QVector3D::dotProduct(lineNormal, planes[i].normal)) * lineNormal;

			if (!linearProgram1(planes, i, line, radius, optVelocity, directionOpt, result)) {
				return false;
			}
		}
	}

	return true;
}


size_t linearProgram3(const std::vector<Plane>& planes, float radius, const QVector3D& optVelocity, bool directionOpt, QVector3D& result)
{
	if (directionOpt) {
		/* Optimize direction. Note that the optimization velocity is of unit length in this case. */
		result = optVelocity * radius;
	}
	else if (absSq(optVelocity) > sqr(radius)) {
		/* Optimize closest point and outside circle. */
		result = optVelocity.normalized() * radius;
	}
	else {
		/* Optimize closest point and inside circle. */
		result = optVelocity;
	}

	for (size_t i = 0; i < planes.size(); ++i) {
		if (QVector3D::dotProduct(planes[i].normal, (planes[i].point - result)) > 0.0f) {
			/* Result does not satisfy constraint i. Compute new optimal result. */
			const QVector3D tempResult = result;

			if (!linearProgram2(planes, i, radius, optVelocity, directionOpt, result)) {
				result = tempResult;
				return i;
			}
		}
	}

	return planes.size();
}


void linearProgram4(const std::vector<Plane>& planes, size_t beginPlane, float radius, QVector3D& result)
{
	float distance = 0.0f;

	for (size_t i = beginPlane; i < planes.size(); ++i) {
		if (QVector3D::dotProduct(planes[i].normal, (planes[i].point - result)) > distance) {
			/* Result does not satisfy constraint of plane i. */
			std::vector<Plane> projPlanes;

			for (size_t j = 0; j < i; ++j) {
				Plane plane;

				const QVector3D crossProduct = QVector3D::crossProduct(planes[j].normal, planes[i].normal);

				if (absSq(crossProduct) <= RVO_EPSILON) {
					/* Plane i and plane j are (almost) parallel. */
					if (QVector3D::dotProduct(planes[i].normal, planes[j].normal) > 0.0f) {
						/* Plane i and plane j point in the same direction. */
						continue;
					}
					else {
						/* Plane i and plane j point in opposite direction. */
						plane.point = 0.5f * (planes[i].point + planes[j].point);
					}
				}
				else {
					/* Plane.point is point on line of intersection between plane i and plane j. */
					const QVector3D lineNormal = QVector3D::crossProduct(crossProduct, planes[i].normal);
					plane.point = planes[i].point + (QVector3D::dotProduct((planes[j].point - planes[i].point), planes[j].normal) / 
														QVector3D::dotProduct(lineNormal, planes[j].normal)) * lineNormal;
				}

				plane.normal = (planes[j].normal - planes[i].normal).normalized();
				projPlanes.push_back(plane);
			}

			const QVector3D tempResult = result;

			if (linearProgram3(projPlanes, radius, planes[i].normal, true, result) < projPlanes.size()) {
				/* This should in principle not happen.  The result is by definition already in the feasible region of this linear program. If it fails, it is due to small floating point error, and the current result is kept. */
				result = tempResult;
			}

			distance = QVector3D::dotProduct(planes[i].normal, (planes[i].point - result));
		}
	}
}

