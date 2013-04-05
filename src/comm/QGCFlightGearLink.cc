/*=====================================================================

QGroundControl Open Source Ground Control Station

(c) 2009 - 2011 QGROUNDCONTROL PROJECT <http://www.qgroundcontrol.org>

This file is part of the QGROUNDCONTROL project

    QGROUNDCONTROL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    QGROUNDCONTROL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with QGROUNDCONTROL. If not, see <http://www.gnu.org/licenses/>.

======================================================================*/

/**
 * @file
 *   @brief Definition of UDP connection (server) for unmanned vehicles
 *   @see Flightgear Manual http://mapserver.flightgear.org/getstart.pdf
 *   @author Lorenz Meier <mavteam@student.ethz.ch>
 *
 */

#include <QTimer>
#include <QList>
#include <QDebug>
#include <QMutexLocker>
#include <iostream>
#include "QGCFlightGearLink.h"
#include "QGC.h"
#include <QHostInfo>
#include "MainWindow.h"

QGCFlightGearLink::QGCFlightGearLink(UASInterface* mav, QString remoteHost, QHostAddress host, quint16 port) :
    process(NULL),
    terraSync(NULL)
{
    this->host = host;
    this->port = port + mav->getUASID();
    this->connectState = false;
    this->currentPort = 49000 + mav->getUASID();
    
	this->mav = mav;
    this->name = tr("FlightGear Link (port:%1)").arg(port);
    setRemoteHost(remoteHost);

	this->currentPort = 49000 + mav->getUASID();
	this->port = port + mav->getUASID();

    qDebug() << this->port << " " << this->currentPort;
}

QGCFlightGearLink::~QGCFlightGearLink()
{
    disconnectSimulation();
}

/**
 * @brief Runs the thread
 *
 **/
void QGCFlightGearLink::run()
{
    exec();
}

void QGCFlightGearLink::setPort(int port)
{
    this->port = port;
    disconnectSimulation();
    connectSimulation();
}

void QGCFlightGearLink::processError(QProcess::ProcessError err)
{
    switch(err)
    {
    case QProcess::FailedToStart:
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Failed to Start"), tr("Please check if the path and command is correct"));
        break;
    case QProcess::Crashed:
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Crashed"), tr("This is a FlightGear-related problem. Please upgrade FlightGear"));
        break;
    case QProcess::Timedout:
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Start Timed Out"), tr("Please check if the path and command is correct"));
        break;
    case QProcess::WriteError:
        MainWindow::instance()->showCriticalMessage(tr("Could not Communicate with FlightGear"), tr("Please check if the path and command is correct"));
        break;
    case QProcess::ReadError:
        MainWindow::instance()->showCriticalMessage(tr("Could not Communicate with FlightGear"), tr("Please check if the path and command is correct"));
        break;
    case QProcess::UnknownError:
    default:
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Error"), tr("Please check if the path and command is correct."));
        break;
    }
}

/**
 * @param host Hostname in standard formatting, e.g. localhost:14551 or 192.168.1.1:14551
 */
void QGCFlightGearLink::setRemoteHost(const QString& host)
{
    if (host.contains(":"))
    {
        //qDebug() << "HOST: " << host.split(":").first();
        QHostInfo info = QHostInfo::fromName(host.split(":").first());
        if (info.error() == QHostInfo::NoError)
        {
            // Add host
            QList<QHostAddress> hostAddresses = info.addresses();
            QHostAddress address;
            for (int i = 0; i < hostAddresses.size(); i++)
            {
                // Exclude loopback IPv4 and all IPv6 addresses
                if (!hostAddresses.at(i).toString().contains(":"))
                {
                    address = hostAddresses.at(i);
                }
            }
            currentHost = address;
            //qDebug() << "Address:" << address.toString();
            // Set port according to user input
            currentPort = host.split(":").last().toInt();
        }
    }
    else
    {
        QHostInfo info = QHostInfo::fromName(host);
        if (info.error() == QHostInfo::NoError)
        {
            // Add host
            currentHost = info.addresses().first();
        }
    }
}

void QGCFlightGearLink::setFlightGearMode(float trueHeadingDeg, float targetAltitudeFt, float targetSpeedKt, QString ap_head, QString ap_alt, QString ap_speed, QString rmgr_input, QString rmgr_enable)
{
    QString state("%1\t%2\t%3\t%4\t%5\t%6\t%7\t%8\n");
	state = state.arg(trueHeadingDeg).arg(targetAltitudeFt / 0.3048).arg(targetSpeedKt / 0.51444444).arg(ap_head).arg(ap_alt).arg(ap_speed).arg(rmgr_input).arg(rmgr_enable);
	writeBytes(state.toAscii().constData(), state.length());

	this->headingModeValue = ap_head;
	this->altitudeModeValue = ap_alt;
	this->speedModeValue = ap_speed;
	this->routeManagerStatus = rmgr_enable;

    //qDebug() << "FlightGear mode set." << state;
}

void QGCFlightGearLink::setHeadingControls(float trueHeadingDeg, float targetAltitudeFt, float targetSpeedKt, QString ap_head, QString ap_alt, QString ap_speed)
{
	setFlightGearMode(trueHeadingDeg, targetAltitudeFt, targetSpeedKt, ap_head, ap_alt, ap_speed, QString(""), QString("false"));
}

void QGCFlightGearLink::setRouteManagerCommand(QString rmgr_input, QString rmgr_enable) {
	setSpeedRouteManagerCommand(this->targetSpeedValue, rmgr_input, rmgr_enable);
}

void QGCFlightGearLink::setSpeedRouteManagerCommand(float speed, QString rmgr_input, QString rmgr_enable) {
	//qDebug() << rmgr_input << " " << rmgr_enable;

	if (rmgr_enable == NULL) {
		setFlightGearMode(this->targetHeadValue, this->targetAltitudeValue, speed,
							this->headingModeValue, this->altitudeModeValue, this->speedModeValue,
							rmgr_input, this->routeManagerStatus);
	}
	else if (rmgr_enable.compare(QString("true")) == 0) {
		setFlightGearMode(this->targetHeadValue, this->targetAltitudeValue, speed,
							QString("true-heading-hold"), QString("altitude-hold"), QString("speed-with-throttle"),
							rmgr_input, rmgr_enable);
	}
	else {
		setFlightGearMode(this->targetHeadValue, this->targetAltitudeValue, speed,
							this->headingModeValue, this->altitudeModeValue, this->speedModeValue,
							rmgr_input, rmgr_enable);
	}
}

void QGCFlightGearLink::writeBytes(const char* data, qint64 size)
{
    //#define QGCFlightGearLink_DEBUG
#ifdef QGCFlightGearLink_DEBUG
    QString bytes;
    QString ascii;
    for (int i=0; i<size; i++)
    {
        unsigned char v = data[i];
        bytes.append(QString().sprintf("%02x ", v));
        if (data[i] > 31 && data[i] < 127)
        {
            ascii.append(data[i]);
        }
        else
        {
            ascii.append(219);
        }
    }
    qDebug() << "Sent" << size << "bytes to" << currentHost.toString() << ":" << currentPort << "data:";
    qDebug() << bytes;
    qDebug() << "ASCII:" << ascii;
#endif
    if (connectState && socket) socket->writeDatagram(data, size, currentHost, currentPort);
}

/**
 * @brief Read a number of bytes from the interface.
 *
 * @param data Pointer to the data byte array to write the bytes to
 * @param maxLength The maximum number of bytes to write
 **/
void QGCFlightGearLink::readBytes()
{
    const qint64 maxLength = 65536;
    char data[maxLength];
    QHostAddress sender;
    quint16 senderPort;

    unsigned int s = socket->pendingDatagramSize();
    if (s > maxLength) std::cerr << __FILE__ << __LINE__ << " UDP datagram overflow, allowed to read less bytes than datagram size" << std::endl;
    socket->readDatagram(data, maxLength, &sender, &senderPort);

    QByteArray b(data, s);

    // Print string
    QString state(b);
    //qDebug() << "FG LINK GOT:" << state;

    QStringList values = state.split("\t");

    // Parse string
    double time;
    float roll, pitch, yaw, rollspeed, pitchspeed, yawspeed;
    double lat, lon, alt;
    double vx, vy, vz;
    double airspeed;
	int currentwp;
	QString rmgrStatus;


	if (values.count() >= 19) {
		time = values.at(0).toDouble();
		lat = values.at(1).toDouble();
		lon = values.at(2).toDouble();
		alt = values.at(3).toDouble();
		roll = values.at(4).toDouble();
		pitch = values.at(5).toDouble();
		yaw = values.at(6).toDouble();
		rollspeed = values.at(7).toDouble();
		pitchspeed = values.at(8).toDouble();
		yawspeed = values.at(9).toDouble();

		vx = values.at(10).toDouble();
		vy = values.at(11).toDouble();
		vz = values.at(12).toDouble();

		airspeed = values.at(13).toDouble();

		currentwp = values.at(14).toInt();
		
		rmgrStatus = values.at(15);
		
		this->targetHeadValue = values.at(16).toDouble();
		this->targetAltitudeValue = values.at(17).toDouble();
		this->targetSpeedValue = values.at(18).toDouble();

		// Send updated state
		emit hilStateChanged(QGC::groundTimeUsecs(), roll, pitch, yaw, rollspeed,
							 pitchspeed, yawspeed, lat, lon, alt,
							 vx, vy, vz, airspeed, currentwp, rmgrStatus);
	}
    //    // Echo data for debugging purposes
    //    std::cerr << __FILE__ << __LINE__ << "Received datagram:" << std::endl;
    //    int i;
    //    for (i=0; i<s; i++)
    //    {
    //        unsigned int v=data[i];
    //        fprintf(stderr,"%02x ", v);
    //    }
    //    std::cerr << std::endl;
}


/**
 * @brief Get the number of bytes to read.
 *
 * @return The number of bytes to read
 **/
qint64 QGCFlightGearLink::bytesAvailable()
{
    return socket->pendingDatagramSize();
}

/**
 * @brief Disconnect the connection.
 *
 * @return True if connection has been disconnected, false if connection couldn't be disconnected.
 **/
bool QGCFlightGearLink::disconnectSimulation()
{
    disconnect(process, SIGNAL(error(QProcess::ProcessError)),
               this, SLOT(processError(QProcess::ProcessError)));
    disconnect(mav, SIGNAL(hilControlsChanged(float, float, float, QString, QString, QString, QString, QString)), this, SLOT(setFlightGearMode(float, float, float, QString, QString, QString, QString, QString)));
	disconnect(mav, SIGNAL(hilHeadingCommand(float, float, float, QString, QString, QString)), this, SLOT(setHeadingControls(float, float, float, QString, QString, QString)));
	disconnect(mav, SIGNAL(hilRouteManagerCommand(QString, QString)), this, SLOT(setRouteManagerCommand(QString, QString)));
    disconnect(mav, SIGNAL(hilSpeedRouteManagerCommand(float, QString, QString)), this, SLOT(setSpeedRouteManagerCommand(float, QString, QString)));
	disconnect(this, SIGNAL(hilStateChanged(uint64_t,float,float,float,float,float,float,float,float,float,float,float,float,float,int,QString)), dynamic_cast<UASAgent*>(mav), SLOT(receiveHilState(uint64_t,float,float,float,float,float,float,float,float,float,float,float,float,float,int,QString)));

    if (process)
    {
        process->close();
        delete process;
        process = NULL;
    }
    if (terraSync)
    {
        terraSync->close();
        delete terraSync;
        terraSync = NULL;
    }
    if (socket)
    {
        socket->close();
        delete socket;
        socket = NULL;
    }

    connectState = false;

    emit flightGearDisconnected();
    emit flightGearConnected(false);
    return !connectState;
}

/**
 * @brief Connect the connection.
 *
 * @return True if connection has been established, false if connection couldn't be established.
 **/
bool QGCFlightGearLink::connectSimulation()
{
    if (!mav) return false;
    socket = new QUdpSocket(this);
    connectState = socket->bind(host, port);

    QObject::connect(socket, SIGNAL(readyRead()), this, SLOT(readBytes()));

    process = new QProcess(this);
    //terraSync = new QProcess(this);

    connect(mav, SIGNAL(hilControlsChanged(float, float, float, QString, QString, QString, QString, QString)), this, SLOT(setFlightGearMode(float, float, float, QString, QString, QString, QString, QString)));
    connect(mav, SIGNAL(hilHeadingCommand(float, float, float, QString, QString, QString)), this, SLOT(setHeadingControls(float, float, float, QString, QString, QString)));
	connect(mav, SIGNAL(hilRouteManagerCommand(QString, QString)), this, SLOT(setRouteManagerCommand(QString, QString)));
	connect(mav, SIGNAL(hilSpeedRouteManagerCommand(float, QString, QString)), this, SLOT(setSpeedRouteManagerCommand(float, QString, QString)));
	connect(this, SIGNAL(hilStateChanged(uint64_t,float,float,float,float,float,float,float,float,float,float,float,float,float,int,QString)), dynamic_cast<UASAgent*>(mav), SLOT(receiveHilState(uint64_t,float,float,float,float,float,float,float,float,float,float,float,float,float,int,QString)));

    //connect(&refreshTimer, SIGNAL(timeout()), this, SLOT(sendUAVUpdate()));
    // Catch process error
    QObject::connect( process, SIGNAL(error(QProcess::ProcessError)),
                      this, SLOT(processError(QProcess::ProcessError)));
    //QObject::connect( terraSync, SIGNAL(error(QProcess::ProcessError)),
    //                  this, SLOT(processError(QProcess::ProcessError)));
    // Start Flightgear
    QStringList processCall;
    QString processFgfs;
    //QString processTerraSync;
    QString fgRoot;
    QString fgScenery;
    QString aircraft;

    if (mav->getSystemType() == MAV_TYPE_FIXED_WING)
    {
        aircraft = "Rascal110-JSBSim";
    }
    else if (mav->getSystemType() == MAV_TYPE_QUADROTOR)
    {
        aircraft = "arducopter";
    }
    else
    {
        aircraft = "Rascal110-JSBSim";
    }

#ifdef Q_OS_MACX
    processFgfs = "/Applications/FlightGear.app/Contents/Resources/fgfs";
    processTerraSync = "/Applications/FlightGear.app/Contents/Resources/terrasync";
    fgRoot = "/Applications/FlightGear.app/Contents/Resources/data";
    //fgScenery = "/Applications/FlightGear.app/Contents/Resources/data/Scenery";
    fgScenery = "/Applications/FlightGear.app/Contents/Resources/data/Scenery-TerraSync";
    //   /Applications/FlightGear.app/Contents/Resources/data/Scenery:
#endif

#ifdef Q_OS_WIN32
    processFgfs = "C:\\Program Files\\FlightGear\\bin\\Win32\\fgfs.exe";
    fgRoot = "C:\\Program Files\\FlightGear\\data";
    fgScenery = "C:\\Program Files\\FlightGear\\data\\Scenery";
#endif

#ifdef Q_OS_LINUX
    processFgfs = "/usr/games/fgfs";
    fgRoot = "/usr/share/games/FlightGear";
    fgScenery = "/usr/share/games/FlightGear/Scenery";
#endif

    // Sanity checks
    bool sane = true;
    QFileInfo executable(processFgfs);
    if (!executable.isExecutable())
    {
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Failed to Start"), tr("FlightGear was not found at %1").arg(processFgfs));
        sane = false;
    }

    QFileInfo root(fgRoot);
    if (!root.isDir())
    {
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Failed to Start"), tr("FlightGear data directory was not found at %1").arg(fgRoot));
        sane = false;
    }

    QFileInfo scenery(fgScenery);
    if (!scenery.isDir())
    {
        MainWindow::instance()->showCriticalMessage(tr("FlightGear Failed to Start"), tr("FlightGear scenery directory was not found at %1").arg(fgScenery));
        sane = false;
    }

    if (!sane) return false;

    // --atlas=socket,out,1,localhost,5505,udp
    // terrasync -p 5505 -S -d /usr/local/share/TerraSync

    processCall << QString("--fg-root=%1").arg(fgRoot);
    processCall << QString("--fg-scenery=%1").arg(fgScenery);
    if (mav->getSystemType() == MAV_TYPE_QUADROTOR)
    {
        // FIXME ADD QUAD-Specific protocol here
        processCall << QString("--generic=socket,out,5,127.0.0.1,%1,udp,qgroundcontrol").arg(port);
        processCall << QString("--generic=socket,in,10,127.0.0.1,%1,udp,qgroundcontrol").arg(currentPort);
    }
    else
    {
        processCall << QString("--generic=socket,out,5,127.0.0.1,%1,udp,qgroundcontrol").arg(port);
        processCall << QString("--generic=socket,in,10,127.0.0.1,%1,udp,qgroundcontrol").arg(currentPort);
    }
    //processCall << "--atlas=socket,out,1,localhost,5505,udp";
    processCall << "--in-air";
    processCall << "--roll=0";
    processCall << "--pitch=0";
    processCall << "--vc=90";
    processCall << "--heading=300";
    processCall << "--timeofday=noon";
    processCall << "--disable-hud-3d";
    processCall << "--disable-fullscreen";
    processCall << "--geometry=400x300";
    processCall << "--disable-anti-alias-hud";
    processCall << "--wind=0@0";
    processCall << "--turbulence=0.0";
    processCall << "--prop:/sim/frame-rate-throttle-hz=30";
    processCall << "--control=mouse";
    processCall << "--disable-intro-music";
    processCall << "--disable-sound";
    processCall << "--disable-random-objects";
    processCall << "--shading-flat";
    processCall << "--fog-disable";
    processCall << "--disable-specular-highlight";
    processCall << "--disable-skyblend";
    processCall << "--disable-random-objects";
    processCall << "--disable-panel";
    processCall << "--disable-horizon-effect";
    processCall << "--disable-clouds";
	processCall << "--enable-ai-models";
    processCall << "--fdm=jsb";
    processCall << "--units-meters";
	processCall << "--multiplay=out,10,mpserver13.flightgear.org,5000";
	processCall << QString("--callsign=%1").arg(mav->getUASName());
	processCall << QString("--multiplay=in,10,,%1").arg((29000 + mav->getUASID()));

    if (mav->getSystemType() == MAV_TYPE_QUADROTOR)
    {
        // Start all engines of the quad
        processCall << "--prop:/engines/engine[0]/running=true";
        processCall << "--prop:/engines/engine[1]/running=true";
        processCall << "--prop:/engines/engine[2]/running=true";
        processCall << "--prop:/engines/engine[3]/running=true";
    }
    else
    {
        processCall << "--prop:/engines/engine/running=true";
    }
    processCall << QString("--lat=%1").arg(UASManager::instance()->getHomeLatitude());
    processCall << QString("--lon=%1").arg(UASManager::instance()->getHomeLongitude());
    processCall << QString("--altitude=%1").arg(UASManager::instance()->getHomeAltitude());
    // Add new argument with this: processCall << "";
    processCall << QString("--aircraft=%1").arg(aircraft);




    //QStringList terraSyncArguments;
    //terraSyncArguments << "-p 5505";
    //terraSyncArguments << "-S";
    //terraSyncArguments << QString("-d=%1").arg(fgScenery);

    //terraSync->start(processTerraSync, terraSyncArguments);
    process->start(processFgfs, processCall);



    emit flightGearConnected(connectState);
    if (connectState) {
        emit flightGearConnected();
        connectionStartTime = QGC::groundTimeUsecs()/1000;
    }
    qDebug() << "STARTING SIM";

        qDebug() << "STARTING: " << processFgfs << processCall;

    start(LowPriority);
    return connectState;
}

/**
 * @brief Check if connection is active.
 *
 * @return True if link is connected, false otherwise.
 **/
bool QGCFlightGearLink::isConnected()
{
    return connectState;
}

QString QGCFlightGearLink::getName()
{
    return name;
}

void QGCFlightGearLink::setName(QString name)
{
    this->name = name;
    //    emit nameChanged(this->name);
}
