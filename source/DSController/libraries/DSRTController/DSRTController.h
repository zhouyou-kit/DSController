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

namespace armarx
{
    class DSRTControllerControlData
    {
    public:
        Eigen::Vector3f tcpDesiredLinearVelocity;
        Eigen::Vector3f tcpDesiredAngularVelocity;
    };

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
        };
        TripleBuffer<DSRTDebugInfo> debugDataInfo;

        std::vector<const SensorValue1DoFActuatorTorque*> torqueSensors;
        std::vector<const SensorValue1DoFGravityTorque*> gravityTorqueSensors;
        std::vector<ControlTarget1DoFActuatorTorque*> targets;


        VirtualRobot::RobotNodePtr tcp;

        VirtualRobot::DifferentialIKPtr ik;
        Eigen::MatrixXf jacobip;
        Eigen::MatrixXf jacobio;

        Eigen::Vector3f desiredPosition;
        Eigen::Vector3f oldPosition;

        Eigen::Vector3f currentTCPLinearVelocity_filtered;
        float filterTimeConstant;

        std::vector<std::string> jointNames;

        float kp;
        float v_max;
        float Damping;
        float torqueLimit;

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
