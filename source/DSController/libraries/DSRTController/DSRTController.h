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

#ifndef _ARMARX_LIB_DSController_DSRTController_H
#define _ARMARX_LIB_DSController_DSRTController_H

#include <RobotAPI/components/units/RobotUnit/NJointControllers/NJointController.h>
#include <VirtualRobot/Robot.h>
#include <RobotAPI/components/units/RobotUnit/RobotUnit.h>
#include <RobotAPI/components/units/RobotUnit/ControlTargets/ControlTarget1DoFActuator.h>
#include <RobotAPI/components/units/RobotUnit/SensorValues/SensorValue1DoFActuator.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/Tools/Gravity.h>

#include <DSController/interface/DSControllerBase.h>
#include "GMRDynamics.h"
#include <ArmarXCore/util/json/JSONObject.h>

#include "MathLib.h"

namespace armarx
{
    class DSRTControllerControlData
    {
    public:
        Eigen::Vector3f tcpDesiredLinearVelocity;
        Eigen::Vector3f tcpDesiredAngularError;
    };

    typedef boost::shared_ptr<GMRDynamics> GMMPtr;

    struct GMRParameters
    {
        int K_gmm_;
        int dim_;
        std::vector<double> Priors_;
        std::vector<double> Mu_;
        std::vector<double> Sigma_;
        std::vector<double> attractor_;
        double dt_;
    };

    class GMMMotionGen
    {
    public:
        GMMMotionGen();

        GMMMotionGen(const std::string& fileName)
        {
            getGMMParamsFromJsonFile(fileName);
        }

        GMMPtr  gmm;
        GMRParameters gmmParas;

        void getGMMParamsFromJsonFile(const std::string& fileName)
        {
            std::ifstream infile { fileName };
            std::string objDefs = { std::istreambuf_iterator<char>(infile), std::istreambuf_iterator<char>() };
            JSONObjectPtr json = new JSONObject();
            json->fromString(objDefs);


            gmmParas.K_gmm_ = json->getInt("K");
            gmmParas.dim_ = json->getInt("dim");
            json->getArray<double>("Priors", gmmParas.Priors_);
            json->getArray<double>("Mu", gmmParas.Mu_);
            json->getArray<double>("attractor", gmmParas.attractor_);
            json->getArray<double>("Sigma", gmmParas.Sigma_);

            gmm.reset(new GMRDynamics(gmmParas.K_gmm_, gmmParas.dim_, gmmParas.dt_, gmmParas.Priors_, gmmParas.Mu_, gmmParas.Sigma_));
            gmm->initGMR(0, 2, 3, 5);
        }
    };

    typedef boost::shared_ptr<GMMMotionGen> GMMMotionGenPtr;


    /**
    * @defgroup Library-DSRTController DSRTController
    * @ingroup DSController
    * A description of the library DSRTController.
    *
    * @class DSRTController
    * @ingroup Library-DSRTController
    * @brief Brief description of class DSRTController.
    *
    * Detailed description of class DSRTController.
    */

    class DSRTController : public NJointControllerWithTripleBuffer<DSRTControllerControlData>, public DSControllerInterface
    {

        // ManagedIceObject interface
    protected:
        void onInitComponent();
        void onDisconnectComponent();


        void controllerRun();



        // NJointControllerInterface interface
    public:
        using ConfigPtrT = DSControllerConfigPtr;

        DSRTController(NJointControllerDescriptionProviderInterfacePtr prov, const NJointControllerConfigPtr& config, const VirtualRobot::RobotPtr&);


        std::string getClassName(const Ice::Current&) const
        {
            return "DSRTController";
        }

        // NJointController interface
        void rtRun(const IceUtil::Time& sensorValuesTimestamp, const IceUtil::Time& timeSinceLastIteration);

    private:
        PeriodicTask<DSRTController>::pointer_type controllerTask;

        struct DSRTControllerSensorData
        {
            Eigen::Matrix4f tcpPose;
            double currentTime;

        };
        TripleBuffer<DSRTControllerSensorData> controllerSensorData;

        struct DSRTDebugInfo
        {
            StringFloatDictionary desired_torques;
            float desiredForce_x;
            float desiredForce_y;
            float desiredForce_z;
            float tcpDesiredTorque_x;
            float tcpDesiredTorque_y;
            float tcpDesiredTorque_z;

            float tcpDesiredAngularError_x;
            float tcpDesiredAngularError_y;
            float tcpDesiredAngularError_z;

            float currentTCPAngularVelocity_x;
            float currentTCPAngularVelocity_y;
            float currentTCPAngularVelocity_z;

            float currentTCPLinearVelocity_x;
            float currentTCPLinearVelocity_y;
            float currentTCPLinearVelocity_z;
        };
        TripleBuffer<DSRTDebugInfo> debugDataInfo;

        std::vector<const SensorValue1DoFActuatorTorque*> torqueSensors;
        std::vector<const SensorValue1DoFGravityTorque*> gravityTorqueSensors;
        std::vector<const SensorValue1DoFActuatorVelocity*> velocitySensors;
        std::vector<const SensorValue1DoFActuatorPosition*> positionSensors;

        std::vector<ControlTarget1DoFActuatorTorque*> targets;


        VirtualRobot::RobotNodePtr tcp;

        VirtualRobot::DifferentialIKPtr ik;
        Eigen::MatrixXf jacobip;
        Eigen::MatrixXf jacobio;

        Eigen::Vector3f desiredPosition;

        Eigen::Quaternionf desiredQuaternion;
        Eigen::Vector3f oldPosition;

        Eigen::Matrix3f oldOrientation;

        Eigen::Vector3f currentTCPLinearVelocity_filtered;
        Eigen::Vector3f currentTCPAngularVelocity_filtered;

        float filterTimeConstant;

        std::vector<std::string> jointNames;

        float posiKp;
        float v_max;
        float posiDamping;
        float torqueLimit;

        float oriKp;
        float oriDamping;

        float nullspaceKp;
        float nullspaceDamping;

        Eigen::VectorXf qnullspace;

        std::vector<GMMMotionGenPtr> gmmMotionGenList;

        float positionErrorTolerance;


        // NJointController interface
    protected:
        void onPublish(const SensorAndControl&, const DebugDrawerInterfacePrx&, const DebugObserverInterfacePrx&);

        // NJointController interface
    protected:
        void rtPreActivateController();
        void rtPostDeactivateController();
    };

}

#endif
