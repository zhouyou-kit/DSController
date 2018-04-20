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
        GMMMotionGen() {}

        GMMMotionGen(const std::string& fileName)
        {
            getGMMParamsFromJsonFile(fileName);
        }

        GMMPtr  gmm;
        GMRParameters gmmParas;
        Eigen::Vector3f desiredDefaultTarget;
        float scaling;

        float belief;
        float v_max;

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

            scaling = json->getDouble("Scaling");
            belief = json->getDouble("InitBelief");
            belief = 0;
            v_max = json->getDouble("MaxVelocity");

            gmm.reset(new GMRDynamics(gmmParas.K_gmm_, gmmParas.dim_, gmmParas.dt_, gmmParas.Priors_, gmmParas.Mu_, gmmParas.Sigma_));
            gmm->initGMR(0, 2, 3, 5);

            if (gmmParas.attractor_.size() < 3)
            {
                ARMARX_ERROR << "attractor in json file should be 6 dimension vector ... ";
            }

            for (int i = 0; i < 3; ++i)
            {
                desiredDefaultTarget(i) = gmmParas.attractor_.at(i);
            }
        }

        void updateDesiredVelocity(const Eigen::Vector3f& currentPositionInMeter, float positionErrorToleranceInMeter)
        {
            Eigen::Vector3f PositionError = currentPositionInMeter - desiredDefaultTarget;
            if (PositionError.norm() < positionErrorToleranceInMeter)
            {
                PositionError.setZero();
            }

            MathLib::Vector position_error;
            position_error.Resize(3);

            for (int i = 0; i < 3; ++i)
            {
                position_error(i) = PositionError(i);
            }

            MathLib::Vector desired_vel;
            desired_vel.Resize(3);
            desired_vel = gmm->getVelocity(position_error);

            Eigen::Vector3f tcpDesiredLinearVelocity;
            tcpDesiredLinearVelocity << desired_vel(0), desired_vel(1), desired_vel(2);

            currentDesiredVelocity = scaling * tcpDesiredLinearVelocity;


            float lenVec = tcpDesiredLinearVelocity.norm();
            if (std::isnan(lenVec))
            {
                tcpDesiredLinearVelocity.setZero();
            }

            if (lenVec > v_max)
            {
                tcpDesiredLinearVelocity = (v_max / lenVec) * tcpDesiredLinearVelocity;
            }
        }



        Eigen::Vector3f currentDesiredVelocity;
    };

    typedef boost::shared_ptr<GMMMotionGen> GMMMotionGenPtr;

    class DSAdaptor
    {
    public:
        float task0_belief;
        float epsilon;
        DSAdaptor() {}

        DSAdaptor(std::vector<GMMMotionGenPtr> gmmMotionGenList, float epsilon)
        {
            task0_belief = 1;
            this->gmmMotionGenList = gmmMotionGenList;

            ARMARX_INFO << "epsilon: " << epsilon;
            this->epsilon = epsilon;

            totalDesiredVelocity.setZero();
        }

        Eigen::Vector3f totalDesiredVelocity;
        std::vector<GMMMotionGenPtr> gmmMotionGenList;


        void updateDesiredVelocity(const Eigen::Vector3f& currentPositionInMeter, float positionErrorToleranceInMeter)
        {
            totalDesiredVelocity.setZero();
            for (size_t i = 0; i < gmmMotionGenList.size(); ++i)
            {
                gmmMotionGenList[i]->updateDesiredVelocity(currentPositionInMeter, positionErrorToleranceInMeter);
                totalDesiredVelocity +=  gmmMotionGenList[i]->belief * gmmMotionGenList[i]->currentDesiredVelocity;
            }
        }

        void updateBelief(const Eigen::Vector3f& realVelocity)
        {
            std::vector<float> beliefUpdate;
            beliefUpdate.resize(gmmMotionGenList.size());

            float nullInnerSimilarity = 0;
            for (size_t i = 0; i < gmmMotionGenList.size(); ++i)
            {

                GMMMotionGenPtr currentGMM = gmmMotionGenList[i];

                float belief = currentGMM->belief;
                Eigen::Vector3f OtherTasks = totalDesiredVelocity - belief * currentGMM->currentDesiredVelocity;
                float innerSimilarity = 2 * OtherTasks.dot(currentGMM->currentDesiredVelocity);
                float outerDisSimilarity = (realVelocity - currentGMM->currentDesiredVelocity).squaredNorm();

                if (innerSimilarity > nullInnerSimilarity)
                {
                    nullInnerSimilarity = innerSimilarity;
                }

                beliefUpdate[i] = - outerDisSimilarity - innerSimilarity;
            }

            float nullOuterSimilarity = realVelocity.squaredNorm();
            float zeroTaskRawBeliefUpdate = - nullInnerSimilarity - nullOuterSimilarity;

            if (zeroTaskRawBeliefUpdate < 0.2)
            {
                zeroTaskRawBeliefUpdate -= 1000;
            }


            beliefUpdate.insert(beliefUpdate.begin(), zeroTaskRawBeliefUpdate);

            WinnerTakeAll(beliefUpdate);

            task0_belief += epsilon * beliefUpdate[0];
            float beliefSum = task0_belief;

            for (size_t i = 0; i < gmmMotionGenList.size(); ++i)
            {
                gmmMotionGenList[i]->belief += epsilon * beliefUpdate[i + 1];
                beliefSum += gmmMotionGenList[i]->belief;
            }

            for (size_t i = 0; i < gmmMotionGenList.size(); ++i)
            {
                gmmMotionGenList[i]->belief /= beliefSum;
            }

            task0_belief /= beliefSum;
        }

    private:

        void WinnerTakeAll(std::vector<float>& UpdateBeliefs_)
        {
            //            std::fill(UpdateBeliefs_.begin(), UpdateBeliefs_.end(), 0);

            int winner_index = 0;

            for (size_t i = 1; i < UpdateBeliefs_.size(); i++)
            {
                if (UpdateBeliefs_[i] > UpdateBeliefs_[winner_index])
                {
                    winner_index = i;
                }
            }

            float winner_belief = task0_belief;

            if (winner_index != 0)
            {
                winner_belief = gmmMotionGenList[winner_index - 1]->belief;
            }

            if (winner_belief == 1)
            {
                return;
            }

            int runnerUp_index = 0;

            if (winner_index == 0)
            {
                runnerUp_index = 1;
            }

            for (size_t i = 0; i < UpdateBeliefs_.size(); i++)
            {
                if (i ==  winner_index)
                {
                    continue;
                }

                if (UpdateBeliefs_[i] > UpdateBeliefs_[runnerUp_index])
                {
                    runnerUp_index = i;
                }
            }

            float offset = 0.5 * (UpdateBeliefs_[winner_index] + UpdateBeliefs_[runnerUp_index]);

            for (size_t i = 0; i < UpdateBeliefs_.size(); i++)
            {
                UpdateBeliefs_[i] -= offset;
            }

            float UpdateSum = 0;

            for (size_t i = 0; i < UpdateBeliefs_.size(); i++)
            {
                float belief = task0_belief;
                if (i != 0)
                {
                    belief = gmmMotionGenList[i - 1]->belief;
                }

                if (belief != 0 || UpdateBeliefs_[i] > 0)
                {
                    UpdateSum += UpdateBeliefs_[i];
                }
            }

            UpdateBeliefs_[winner_index] -= UpdateSum;
        }
    };

    typedef boost::shared_ptr<DSAdaptor> DSAdaptorPtr;



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

            Eigen::Vector3f linearVelocity;

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

            float belief0;
            float belief1;
            float belief2;
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

        DSAdaptorPtr dsAdaptorPtr;

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
