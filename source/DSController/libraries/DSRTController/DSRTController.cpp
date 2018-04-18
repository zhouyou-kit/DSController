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

    jointNames.clear();
    ARMARX_CHECK_EXPRESSION_W_HINT(rns, cfg->nodeSetName);
    for (size_t i = 0; i < rns->getSize(); ++i)
    {
        std::string jointName = rns->getNode(i)->getName();
        jointNames.push_back(jointName);
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
    oldOrientation = tcp->getPoseInRootFrame().block<3, 3>(0, 0);

    std::vector<float> desiredPositionVec = cfg->desiredPosition;
    for (size_t i = 0; i < 3; ++i)
    {
        desiredPosition(i) = desiredPositionVec[i];
    }

    std::vector<float> desiredQuaternionVec = cfg->desiredQuaternion;
    desiredQuaternion.x() = desiredQuaternionVec[0];
    desiredQuaternion.y() = desiredQuaternionVec[1];
    desiredQuaternion.z() = desiredQuaternionVec[2];
    desiredQuaternion.w() = desiredQuaternionVec[3];



    DSRTControllerControlData initData;
    for (size_t i = 0; i < 3; ++i)
    {
        initData.tcpDesiredLinearVelocity(i) = 0;
    }

    for (size_t i = 0; i < 3; ++i)
    {
        initData.tcpDesiredAngularError(i) = 0;
    }
    reinitTripleBuffer(initData);

    // initial filter
    currentTCPLinearVelocity_filtered.setZero();
    filterTimeConstant = cfg->filterTimeConstant;
    posiKp = cfg->posiKp;
    v_max = cfg->v_max;
    posiDamping = cfg->posiDamping;
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

    Eigen::Vector3f tcpDesiredLinearVelocity = posiKp * (desiredPosition - currentTCPPosition);
    float lenVec = tcpDesiredLinearVelocity.norm();
    if (lenVec > v_max)
    {
        tcpDesiredLinearVelocity = (v_max / lenVec) * tcpDesiredLinearVelocity;
    }

    // ToDo: angular velocity
    Eigen::Vector3f tcpDesiredAngularError;
    tcpDesiredAngularError << 0, 0, 0;

    Eigen::Matrix3f currentOrientation = currentTCPPose.block<3, 3>(0, 0);
    Eigen::Matrix3f desiredOrientation = desiredQuaternion.normalized().toRotationMatrix();
    Eigen::Matrix3f orientationError = currentOrientation * desiredOrientation.inverse();
    Eigen::Quaternionf diffQuaternion(orientationError);
    Eigen::AngleAxisf angleAxis(diffQuaternion);
    tcpDesiredAngularError = angleAxis.angle() * angleAxis.axis();


    // ToDo: GMM velocity calculation

    getWriterControlStruct().tcpDesiredLinearVelocity = tcpDesiredLinearVelocity;
    getWriterControlStruct().tcpDesiredAngularError = tcpDesiredAngularError;

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

        // calculate linear velocity
        Eigen::Vector3f currentTCPPosition;
        currentTCPPosition << currentTCPPose(0, 3), currentTCPPose(1, 3), currentTCPPose(2, 3);
        Eigen::Vector3f currentTCPLinearVelocity_raw = (currentTCPPosition - oldPosition) / deltaT;
        double filterFactor = deltaT / (filterTimeConstant + deltaT);
        currentTCPLinearVelocity_filtered = (1 - filterFactor) * currentTCPLinearVelocity_filtered + filterFactor * currentTCPLinearVelocity_raw;
        oldPosition = currentTCPPosition;

        // calculate angular velocity
        Eigen::Matrix3f currentTCPOrientation = currentTCPPose.block<3, 3>(0, 0);
        Eigen::Matrix3f currentTCPDiff = currentTCPOrientation * oldOrientation.inverse();
        Eigen::AngleAxisf currentAngleAxisDiff(currentTCPDiff);
        Eigen::Vector3f currentTCPAngularVelocity_raw = currentAngleAxisDiff.angle() * currentAngleAxisDiff.axis();
        currentTCPAngularVelocity_filtered = (1 - filterFactor) * currentTCPAngularVelocity_filtered + filterFactor * currentTCPAngularVelocity_raw;
        oldOrientation = currentTCPOrientation;


        Eigen::Vector3f tcpDesiredLinearVelocity = rtGetControlStruct().tcpDesiredLinearVelocity;
        Eigen::Vector3f tcpDesiredAngularError = rtGetControlStruct().tcpDesiredAngularError;

        // calculate desired tcp force
        Eigen::Vector3f tcpDesiredForce = -posiDamping * (currentTCPLinearVelocity_filtered - tcpDesiredLinearVelocity);

        // calculate desired tcp torque
        Eigen::Vector3f tcpDesiredTorque = - oriKp * tcpDesiredAngularError - oriDamping * currentTCPAngularVelocity_filtered;

        // calculate desired joint torque
        jacobip = ik->getJacobianMatrix(tcp, VirtualRobot::IKSolver::CartesianSelection::Position);
        jacobio = ik->getJacobianMatrix(tcp, VirtualRobot::IKSolver::CartesianSelection::Orientation);
        Eigen::VectorXf jointDesiredTorques = 0.001 * jacobip.transpose() * tcpDesiredForce + jacobio.transpose() * tcpDesiredTorque;



        for (size_t i = 0; i < targets.size(); ++i)
        {
            float desiredTorque = jointDesiredTorques(i);
            if (abs(desiredTorque) > torqueLimit)
            {
                desiredTorque = sign(desiredTorque) * torqueLimit;
            }

            debugDataInfo.getWriteBuffer().desired_torques[jointNames[i]] = desiredTorque;

            targets.at(i)->torque = desiredTorque;
        }

        debugDataInfo.getWriteBuffer().desiredForce_x = tcpDesiredForce(0);
        debugDataInfo.getWriteBuffer().desiredForce_y = tcpDesiredForce(1);
        debugDataInfo.getWriteBuffer().desiredForce_z = tcpDesiredForce(2);

        debugDataInfo.commitWrite();

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

    StringVariantBaseMap datafields;
    auto values = debugDataInfo.getUpToDateReadBuffer().desired_torques;
    for (auto& pair : values)
    {
        datafields[pair.first] = new Variant(pair.second);
    }

    datafields["desiredForce_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_x);
    datafields["desiredForce_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_y);
    datafields["desiredForce_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_z);

    debugObs->setDebugChannel("DSControllerOutput", datafields);

}

void DSRTController::rtPreActivateController()
{

}

void DSRTController::rtPostDeactivateController()
{

}
