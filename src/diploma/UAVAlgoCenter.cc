/**
 * @file
 *   @brief UI module for algorithm control implementation
 *
 *   @author Matei Chiperi <matei.chiperi@gmail.com>
 *
 */

//#include "UAS.h"
//#include "UASView.h"
#include "UAVAlgoCenter.h"
#include <QMessageBox>
#include "MAVLinkProtocol.h"
#include "UASManager.h"


UAVAlgoCenter::UAVAlgoCenter(QWidget* parent) : QWidget(parent), m_ui(new Ui::AlgoCenter)
{

	m_ui->setupUi(this);

	setMinimumSize(600, 80);

    listLayout = new QVBoxLayout(m_ui->uavInstanceList);
	listLayout->setMargin(0);
	listLayout->setSpacing(3);
	listLayout->setAlignment(Qt::AlignTop);
    m_ui->uavInstanceList->setLayout(listLayout);
	setObjectName("UAV Algo Center");

	uavInstances = QMap<UASInterface*, UAVInstance*>();

	this->setVisible(true);

	connect(UASManager::instance(), SIGNAL(UASCreated(UASInterface*)), this, SLOT(addUAV(UASInterface*)));

    connect(m_ui->sendSpeed, SIGNAL(clicked()), this, SLOT(sendSpeedParams()));

	listLayout->addWidget(new UAVInstance(NULL, this));

    m_ui->sendSpeed->setEnabled(true);

}

UAVAlgoCenter::~UAVAlgoCenter()
{
	delete m_ui;
}

void UAVAlgoCenter::changeEvent(QEvent *e)
{
	QWidget::changeEvent(e);
}

void UAVAlgoCenter::addUAV(UASInterface* uav)
{
    this->uas = dynamic_cast<UAS*>(uav);

    if (uavInstances.size() == 0) {
		connect(this, SIGNAL(setUAVControl(double,double,double,double)), this->uas, SLOT(setManualControlCommands(double, double, double, double)));
		connect(this, SIGNAL(setUAVTarget(float, float, float, float)), this->uas, SLOT(setTargetPosition(float,float,float,float)));
    }

	if (!uavInstances.contains(uav))
	{
		uavInstances.insert(uav, new UAVInstance(uav, this));
		listLayout->addWidget(uavInstances.value(uav));

        //m_ui->sendSpeed->setEnabled(true);
	}
}

void UAVAlgoCenter::sendSpeedParams()
{
    //double thrust = uavInstances.constBegin().value()->getThrust();
    //emit setUAVControl(m_ui->set_vroll->value(), m_ui->set_vpitch->value(), m_ui->set_vyaw->value(), thrust);
    //emit setUAVTarget((float)m_ui->set_vx->value(), (float)m_ui->set_vy->value(),
    //                  (float)m_ui->set_vz->value(), (float)m_ui->set_vyaw->value());

	//this->uas->executeCommand(MAV_CMD_NAV_WAYPOINT, 0, 0, 20, 20, 0, (float)m_ui->set_vx->value(), 
	//							(float)m_ui->set_vy->value(), (float)m_ui->set_vz->value(), 0);

	MAVLinkProtocol *mavLinkProt = new MAVLinkProtocol();
	UASManager::instance()->addUAS(new UAS(mavLinkProt, 123));
}

void UAVAlgoCenter::removeUAV(UASInterface* uav)
{
	Q_UNUSED(uav);
}
