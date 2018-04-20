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
        const SensorValue1DoFActuatorVelocity* velocitySensor = sv->asA<SensorValue1DoFActuatorVelocity>();
        const SensorValue1DoFGravityTorque* gravityTorqueSensor = sv->asA<SensorValue1DoFGravityTorque>();
        const SensorValue1DoFActuatorPosition* positionSensor = sv->asA<SensorValue1DoFActuatorPosition>();
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
        velocitySensors.push_back(velocitySensor);
        positionSensors.push_back(positionSensor);
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
    ARMARX_INFO << "ik reseted ";

    std::vector<float> desiredQuaternionVec = cfg->desiredQuaternion;
    ARMARX_CHECK_EQUAL(desiredQuaternionVec.size(), 4);

    desiredQuaternion.x() = desiredQuaternionVec.at(0);
    desiredQuaternion.y() = desiredQuaternionVec.at(1);
    desiredQuaternion.z() = desiredQuaternionVec.at(2);
    desiredQuaternion.w() = desiredQuaternionVec.at(3);



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
    currentTCPAngularVelocity_filtered.setZero();
    filterTimeConstant = cfg->filterTimeConstant;
    posiKp = cfg->posiKp;
    v_max = cfg->v_max;
    posiDamping = cfg->posiDamping;
    torqueLimit = cfg->torqueLimit;
    oriKp = cfg->oriKp;
    oriDamping  = cfg->oriDamping;


    std::vector<float> qnullspaceVec = cfg->qnullspaceVec;

    qnullspace.resize(qnullspaceVec.size());

    for (size_t i = 0; i < qnullspaceVec.size(); ++i)
    {
        qnullspace(i) = qnullspaceVec[i];
    }

    nullspaceKp = cfg->nullspaceKp;
    nullspaceDamping = cfg->nullspaceDamping;


    //set GMM parameters ...
    std::vector<std::string> gmmParamsFiles = cfg->gmmParamsFiles;
    if (gmmParamsFiles.size() == 0)
    {
        ARMARX_ERROR << "no gmm found ... ";
    }

    gmmMotionGenList.clear();
    for (size_t i = 0; i < gmmParamsFiles.size(); ++i)
    {
        gmmMotionGenList.push_back(GMMMotionGenPtr(new GMMMotionGen(gmmParamsFiles.at(i))));
    }


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

    Eigen::Vector3f PositionError = currentTCPPosition - desiredPosition;
    if (PositionError.norm() < 100)
    {
        PositionError.setZero();
    }

    MathLib::Vector position_error;
    position_error.Resize(3);

    for (int i = 0; i < 3; ++i)
    {
        position_error(i) = 0.001 * PositionError(i);
    }

    GMMMotionGenPtr gmmMotionGen = gmmMotionGenList[0];
    MathLib::Vector desired_vel;
    desired_vel.Resize(3);
    desired_vel = gmmMotionGen->gmm->getVelocity(position_error);

    Eigen::Vector3f tcpDesiredLinearVelocity;
    tcpDesiredLinearVelocity << desired_vel(0), desired_vel(1), desired_vel(2);
    tcpDesiredLinearVelocity = posiKp * tcpDesiredLinearVelocity;
    //    Eigen::Vector3f tcpDesiredLinearVelocity = posiKp * PositionError;
    float lenVec = tcpDesiredLinearVelocity.norm();
    if (std::isnan(lenVec))
    {
        ARMARX_WARNING << "nan value from desired velocity ...  ";
        tcpDesiredLinearVelocity.setZero();
    }

    if (lenVec > v_max)
    {
        tcpDesiredLinearVelocity = (v_max / lenVec) * tcpDesiredLinearVelocity;
    }

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

        Eigen::MatrixXf jacobi = ik->getJacobianMatrix(tcp, VirtualRobot::IKSolver::CartesianSelection::All);

        Eigen::VectorXf qpos;
        Eigen::VectorXf qvel;
        qpos.resize(positionSensors.size());
        qvel.resize(velocitySensors.size());

        int jointDim = positionSensors.size();

        for (size_t i = 0; i < velocitySensors.size(); ++i)
        {
            qpos(i) = positionSensors[i]->position;
            qvel(i) = velocitySensors[i]->velocity;
        }

        // calculate nullspace torque


        Eigen::VectorXf tcptwist = jacobi * qvel;

        Eigen::Vector3f currentTCPLinearVelocity;
        currentTCPLinearVelocity << 0.001 * tcptwist(0),  0.001 * tcptwist(1), 0.001 * tcptwist(2);
        double filterFactor = deltaT / (filterTimeConstant + deltaT);
        currentTCPLinearVelocity_filtered = (1 - filterFactor) * currentTCPLinearVelocity_filtered + filterFactor * currentTCPLinearVelocity;


        Eigen::Vector3f currentTCPAngularVelocity;
        currentTCPAngularVelocity << tcptwist(3), tcptwist(4), tcptwist(5);
        //        // calculate linear velocity
        //        Eigen::Vector3f currentTCPPosition;
        //        currentTCPPosition << currentTCPPose(0, 3), currentTCPPose(1, 3), currentTCPPose(2, 3);
        //        Eigen::Vector3f currentTCPLinearVelocity_raw = (currentTCPPosition - oldPosition) / deltaT;
        //        oldPosition = currentTCPPosition;

        //        // calculate angular velocity
        //        Eigen::Matrix3f currentTCPOrientation = currentTCPPose.block<3, 3>(0, 0);
        //        Eigen::Matrix3f currentTCPDiff = currentTCPOrientation * oldOrientation.inverse();
        //        Eigen::AngleAxisf currentAngleAxisDiff(currentTCPDiff);
        //        Eigen::Vector3f currentTCPAngularVelocity_raw = currentAngleAxisDiff.angle() * currentAngleAxisDiff.axis();
        //        oldOrientation = currentTCPOrientation;
        //        currentTCPAngularVelocity_filtered = (1 - filterFactor) * currentTCPAngularVelocity_filtered + filterFactor * currentTCPAngularVelocity_raw;


        Eigen::Vector3f tcpDesiredLinearVelocity = rtGetControlStruct().tcpDesiredLinearVelocity;
        Eigen::Vector3f tcpDesiredAngularError = rtGetControlStruct().tcpDesiredAngularError;

        // calculate desired tcp force
        Eigen::Vector3f tcpDesiredForce = -posiDamping * (currentTCPLinearVelocity_filtered - tcpDesiredLinearVelocity);

        // calculate desired tcp torque
        Eigen::Vector3f tcpDesiredTorque = - oriKp * tcpDesiredAngularError - oriDamping * currentTCPAngularVelocity;

        Eigen::Vector6f tcpDesiredWrench;
        tcpDesiredWrench << 0.001 * tcpDesiredForce, tcpDesiredTorque;

        // calculate desired joint torque
        Eigen::MatrixXf I = Eigen::MatrixXf::Identity(jointDim, jointDim);

        float lambda = 2;
        Eigen::MatrixXf jtpinv = ik->computePseudoInverseJacobianMatrix(jacobi.transpose(), lambda);

        Eigen::VectorXf nullqerror = qpos - qnullspace;

        for (int i = 0; i < nullqerror.rows(); ++i)
        {
            if (fabs(nullqerror(i)) < 0.09)
            {
                nullqerror(i) = 0;
            }
        }
        Eigen::VectorXf nullspaceTorque = - nullspaceKp * nullqerror - nullspaceDamping * qvel;

        Eigen::VectorXf jointDesiredTorques = jacobi.transpose() * tcpDesiredWrench + (I - jacobi.transpose() * jtpinv) * nullspaceTorque;

        for (size_t i = 0; i < targets.size(); ++i)
        {
            float desiredTorque = jointDesiredTorques(i);

            desiredTorque = (desiredTorque >  torqueLimit) ? torqueLimit : desiredTorque;
            desiredTorque = (desiredTorque < -torqueLimit) ? -torqueLimit : desiredTorque;

            debugDataInfo.getWriteBuffer().desired_torques[jointNames[i]] = jointDesiredTorques(i);

            targets.at(i)->torque = desiredTorque;
        }


        debugDataInfo.getWriteBuffer().desiredForce_x = tcpDesiredForce(0);
        debugDataInfo.getWriteBuffer().desiredForce_y = tcpDesiredForce(1);
        debugDataInfo.getWriteBuffer().desiredForce_z = tcpDesiredForce(2);


        debugDataInfo.getWriteBuffer().tcpDesiredTorque_x = tcpDesiredTorque(0);
        debugDataInfo.getWriteBuffer().tcpDesiredTorque_y = tcpDesiredTorque(1);
        debugDataInfo.getWriteBuffer().tcpDesiredTorque_z = tcpDesiredTorque(2);

        debugDataInfo.getWriteBuffer().tcpDesiredAngularError_x = tcpDesiredAngularError(0);
        debugDataInfo.getWriteBuffer().tcpDesiredAngularError_y = tcpDesiredAngularError(1);
        debugDataInfo.getWriteBuffer().tcpDesiredAngularError_z = tcpDesiredAngularError(2);

        debugDataInfo.getWriteBuffer().currentTCPAngularVelocity_x = currentTCPAngularVelocity(0);
        debugDataInfo.getWriteBuffer().currentTCPAngularVelocity_y = currentTCPAngularVelocity(1);
        debugDataInfo.getWriteBuffer().currentTCPAngularVelocity_z = currentTCPAngularVelocity(2);

        debugDataInfo.getWriteBuffer().currentTCPLinearVelocity_x = currentTCPLinearVelocity_filtered(0);
        debugDataInfo.getWriteBuffer().currentTCPLinearVelocity_y = currentTCPLinearVelocity_filtered(1);
        debugDataInfo.getWriteBuffer().currentTCPLinearVelocity_z = currentTCPLinearVelocity_filtered(2);

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

    //    std::string nameJacobi = "jacobiori";
    //    std::string nameJacobipos = "jacobipos";

    //    std::vector<float> jacobiVec = debugDataInfo.getUpToDateReadBuffer().jacobiVec;
    //    std::vector<float> jacobiPos = debugDataInfo.getUpToDateReadBuffer().jacobiPos;

    //    for (size_t i = 0; i < jacobiVec.size(); ++i)
    //    {
    //        std::stringstream ss;
    //        ss << i;
    //        std::string name = nameJacobi + ss.str();
    //        datafields[name] = new Variant(jacobiVec[i]);
    //        std::string namepos = nameJacobipos + ss.str();
    //        datafields[namepos] = new Variant(jacobiPos[i]);

    //    }



    datafields["desiredForce_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_x);
    datafields["desiredForce_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_y);
    datafields["desiredForce_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().desiredForce_z);

    datafields["tcpDesiredTorque_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredTorque_x);
    datafields["tcpDesiredTorque_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredTorque_y);
    datafields["tcpDesiredTorque_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredTorque_z);

    datafields["tcpDesiredAngularError_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredAngularError_x);
    datafields["tcpDesiredAngularError_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredAngularError_y);
    datafields["tcpDesiredAngularError_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().tcpDesiredAngularError_z);

    datafields["currentTCPAngularVelocity_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPAngularVelocity_x);
    datafields["currentTCPAngularVelocity_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPAngularVelocity_y);
    datafields["currentTCPAngularVelocity_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPAngularVelocity_z);


    datafields["currentTCPLinearVelocity_x"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPLinearVelocity_x);
    datafields["currentTCPLinearVelocity_y"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPLinearVelocity_y);
    datafields["currentTCPLinearVelocity_z"] = new Variant(debugDataInfo.getUpToDateReadBuffer().currentTCPLinearVelocity_z);

    debugObs->setDebugChannel("DSControllerOutput", datafields);

}

void DSRTController::rtPreActivateController()
{

}

void DSRTController::rtPostDeactivateController()
{

}
