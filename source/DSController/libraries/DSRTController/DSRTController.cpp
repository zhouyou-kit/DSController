/*
 * This file is part of ArmarX.
 *
 * ArmarX is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ArmarX is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    DSController::ArmarXObjects::DSRTController
 * @author     Mahdi Khoramshahi ( m80 dot khoramshahi at gmail dot com )
 * @date       2018
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "DSRTController.h"


using namespace armarx;

NJointControllerRegistration<DSRTController> registrationControllerDSRTController("DSRTController");

void DSRTController::onInitComponent()
{
    ARMARX_INFO << "init ...";
    controllerTask = new PeriodicTask<DSRTController>(this, &DSRTController::controllerRun, 0.3);
    controllerTask->start();
}

void DSRTController::onDisconnectComponent()
{
    ARMARX_INFO << "stopped ...";
    controllerTask->stop();
}


DSRTController::DSRTController(NJointControllerDescriptionProviderInterfacePtr prov, const NJointControllerConfigPtr& config, const VirtualRobot::RobotPtr&)
{
    DSControllerConfigPtr cfg = DSControllerConfigPtr::dynamicCast(config);

    ARMARX_CHECK_EXPRESSION(cfg);
    ARMARX_CHECK_EXPRESSION(prov);
    RobotUnitPtr robotUnit = RobotUnitPtr::dynamicCast(prov);
    ARMARX_CHECK_EXPRESSION(robotUnit);
    ARMARX_CHECK_EXPRESSION(!cfg->nodeSetName.empty());
    VirtualRobot::RobotNodeSetPtr rns = robotUnit->getRtRobot()->getRobotNodeSet(cfg->nodeSetName);

    ARMARX_CHECK_EXPRESSION_W_HINT(rns, cfg->nodeSetName);
    for (size_t i = 0; i < rns->getSize(); ++i)
    {
        std::string jointName = rns->getNode(i)->getName();
        ControlTargetBase* ct = prov->getControlTarget(jointName, ControlModes::Torque1DoF);
        ARMARX_CHECK_EXPRESSION(ct);
        const SensorValueBase* sv = prov->getSensorValue(jointName);
        ARMARX_CHECK_EXPRESSION(sv);
        auto casted_ct = ct->asA<ControlTarget1DoFActuatorTorque>();
        ARMARX_CHECK_EXPRESSION(casted_ct);
        targets.push_back(casted_ct);

        const SensorValue1DoFActuatorTorque* torqueSensor = sv->asA<SensorValue1DoFActuatorTorque>();
        const SensorValue1DoFGravityTorque* gravityTorqueSensor = sv->asA<SensorValue1DoFGravityTorque>();
        if (!torqueSensor)
        {
            ARMARX_WARNING << "No Torque sensor available for " << jointName;
        }
        if (!gravityTorqueSensor)
        {
            ARMARX_WARNING << "No Gravity Torque sensor available for " << jointName;
        }

        torqueSensors.push_back(torqueSensor);
        gravityTorqueSensors.push_back(gravityTorqueSensor);
    };
    ARMARX_INFO << "Initialized " << targets.size() << " targets";
    tcp = (cfg->tcpName.empty()) ? rns->getTCP() : robotUnit->getRtRobot()->getRobotNode(cfg->tcpName);
    ARMARX_CHECK_EXPRESSION_W_HINT(tcp, cfg->tcpName);


    ik.reset(new VirtualRobot::DifferentialIK(rns, rns->getRobot()->getRootNode(), VirtualRobot::JacobiProvider::eSVDDamped));

    DSRTControllerSensorData initSensorData;
    initSensorData.tcpPose = tcp->getPoseInRootFrame();
    initSensorData.currentTime = 0;
    controllerSensorData.reinitAllBuffers(initSensorData);


    oldPosition = tcp->getPositionInRootFrame();

    std::vector<float> desiredPositionVec = cfg->desiredPosition;
    for (size_t i = 0; i < 3; ++i)
    {
        desiredPosition(i) = desiredPositionVec[i];
    }

    DSRTControllerControlData initData;
    for (size_t i = 0; i < 3; ++i)
    {
        initData.tcpDesiredLinearVelocity(i) = 0;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        initData.tcpDesiredAngularVelocity(i) = 0;
    }
    reinitTripleBuffer(initData);

    // initial filter
    currentTCPLinearVelocity_filtered.setZero();
    filterTimeConstant = cfg->filterTimeConstant;
    kp = cfg->kp;
    v_max = cfg->v_max;
    Damping = cfg->D;
    torqueLimit = cfg->torqueLimit;
    ARMARX_INFO << "Initialization done";
}


void DSRTController::controllerRun()
{
    if (!controllerSensorData.updateReadBuffer())
    {
        return;
    }

    Eigen::Matrix4f currentTCPPose = controllerSensorData.getReadBuffer().tcpPose;
    Eigen::Vector3f currentTCPPosition;
    currentTCPPosition << currentTCPPose(0, 3), currentTCPPose(1, 3), currentTCPPose(2, 3);

    Eigen::Vector3f tcpDesiredLinearVelocity = kp * (desiredPosition - currentTCPPosition);
    float lenVec = tcpDesiredLinearVelocity.norm();
    if (lenVec > v_max)
    {
        tcpDesiredLinearVelocity = (v_max / lenVec) * tcpDesiredLinearVelocity;
    }

    // ToDo: angular velocity
    Eigen::Vector3f tcpDesiredAngularVelocity;
    tcpDesiredAngularVelocity << 0, 0, 0;

    // ToDo: GMM velocity calculation



    getWriterControlStruct().tcpDesiredLinearVelocity = tcpDesiredLinearVelocity;
    getWriterControlStruct().tcpDesiredAngularVelocity = tcpDesiredAngularVelocity;

    writeControlStruct();
}


void DSRTController::rtRun(const IceUtil::Time& sensorValuesTimestamp, const IceUtil::Time& timeSinceLastIteration)
{
    double deltaT = timeSinceLastIteration.toSecondsDouble();
    if (deltaT != 0)
    {

        Eigen::Matrix4f currentTCPPose = tcp->getPoseInRootFrame();

        controllerSensorData.getWriteBuffer().tcpPose = currentTCPPose;
        controllerSensorData.getWriteBuffer().currentTime += deltaT;
        controllerSensorData.commitWrite();

        Eigen::Vector3f currentTCPRPY = VirtualRobot::MathTools::eigen4f2rpy(currentTCPPose);
        Eigen::Vector3f currentTCPPosition;
        currentTCPPosition << currentTCPPose(0, 3), currentTCPPose(1, 3), currentTCPPose(2, 3);

        Eigen::Vector3f currentTCPLinearVelocity_raw = (currentTCPPosition - oldPosition) / deltaT;

        oldPosition = currentTCPPosition;


        double filterFactor = deltaT / (filterTimeConstant + deltaT);
        currentTCPLinearVelocity_filtered = (1 - filterFactor) * currentTCPLinearVelocity_filtered + filterFactor * currentTCPLinearVelocity_raw;

        ARMARX_INFO << "real velocity: " << currentTCPLinearVelocity_filtered;


        //TODO: orientation velocity
        Eigen::Vector3f tcpAngularVelocity;
        tcpAngularVelocity.setZero();

        Eigen::Vector3f tcpDesiredLinearVelocity = rtGetControlStruct().tcpDesiredLinearVelocity;
        Eigen::Vector3f tcpDesiredAngularVelocity = rtGetControlStruct().tcpDesiredAngularVelocity;

        Eigen::Vector3f tcpDesiredForce = -Damping * (currentTCPLinearVelocity_filtered - tcpDesiredLinearVelocity);


        ARMARX_INFO << "desired velocity: " << tcpDesiredLinearVelocity;
        ARMARX_INFO << "desired force: " << tcpDesiredForce;


        Eigen::Vector3f tcpDesiredTorque;
        tcpDesiredTorque.setZero();

        jacobip = ik->getJacobianMatrix(tcp, VirtualRobot::IKSolver::CartesianSelection::Position);
        jacobio = ik->getJacobianMatrix(tcp, VirtualRobot::IKSolver::CartesianSelection::Orientation);
        Eigen::VectorXf jointDesiredTorques = jacobip.transpose() * tcpDesiredForce + jacobio.transpose() * tcpDesiredTorque;
        ARMARX_INFO << "desired Torque: " << jointDesiredTorques;



        for (size_t i = 0; i < targets.size(); ++i)
        {
            float desiredTorque = jointDesiredTorques(i);
            if (abs(desiredTorque) > torqueLimit)
            {
                desiredTorque = sign(desiredTorque) * torqueLimit;
            }

            targets.at(i)->torque =  0;
        }

    }
    else
    {
        for (size_t i = 0; i < targets.size(); ++i)
        {
            targets.at(i)->torque = 0;

        }
    }



}

void DSRTController::onPublish(const SensorAndControl&, const DebugDrawerInterfacePrx&, const DebugObserverInterfacePrx& debugObs)
{

}

void DSRTController::rtPreActivateController()
{

}

void DSRTController::rtPostDeactivateController()
{

}
