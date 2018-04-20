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
 * @package    DSController::DSControllerGroup
 * @author     Mahdi Khoramshahi ( m80 dot khoramshahi at gmail dot com )
 * @date       2018
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "DSControllerTest.h"

//#include <ArmarXCore/core/time/TimeUtil.h>
//#include <ArmarXCore/observers/variant/DatafieldRef.h>

#include <DSController/interface/DSControllerBase.h>

using namespace armarx;
using namespace DSControllerGroup;

// DO NOT EDIT NEXT LINE
DSControllerTest::SubClassRegistry DSControllerTest::Registry(DSControllerTest::GetName(), &DSControllerTest::CreateInstance);



void DSControllerTest::onEnter()
{
    // put your user code for the enter-point here
    // execution time should be short (<100ms)
}

void DSControllerTest::run()
{
    getRobotUnit()->loadLibFromPackage("DSController", "DSRTController");

    float kp = in.getKp();
    float v_max = in.getVmax();
    float D = in.getDamping();
    std::string nodeSetName = in.getNodeSetName();
    std::string tcpName = getRobot()->getRobotNodeSet(nodeSetName)->getTCP()->getName();

    std::vector<float> desiredTarget = in.getDesiredTarget();
    std::vector<float> desiredQuaternion = in.getDesiredQuaternion();

    float filterTimeConstant = in.getFilterTimeConstant();
    float torqueLimit = in.getTorqueLimit();

    float oriKp = in.getOriKp();
    float oriDamping = in.getOriDamping();
    std::vector<float> nullspaceVec = in.getNullSpaceJoint();

    float nullspaceKp = in.getNullSpaceKp();
    float nullspaceDamping = in.getNullSpaceDamping();
    float positionErrorTolerance = in.getPositionErrorTolerance();
    std::vector<std::string> gmmfilestrings = in.getGMMParamsFiles();
    float dsAdaptorEpsilon = in.getDsAdaptorEpsilon();
    DSControllerConfigPtr config = new DSControllerConfig(kp,
            v_max,
            D,
            oriDamping,
            oriKp,
            filterTimeConstant,
            torqueLimit,
            nodeSetName,
            tcpName,
            desiredTarget,
            desiredQuaternion,
            nullspaceVec,
            nullspaceKp,
            nullspaceDamping,
            gmmfilestrings,
            positionErrorTolerance,
            dsAdaptorEpsilon
                                                         );


    DSControllerInterfacePrx dsController
        = DSControllerInterfacePrx::checkedCast(getRobotUnit()->createNJointController("DSRTController", "dsController", config));

    dsController->activateController();

    while (!isRunningTaskStopped())
    {
    }

    dsController->deactivateController();
    while (dsController->isControllerActive())
    {
        usleep(10000);
    }
    dsController->deleteController();
}

//void DSControllerTest::onBreak()
//{
//    // put your user code for the breaking point here
//    // execution time should be short (<100ms)
//}

void DSControllerTest::onExit()
{
    // put your user code for the exit point here
    // execution time should be short (<100ms)
}


// DO NOT EDIT NEXT FUNCTION
XMLStateFactoryBasePtr DSControllerTest::CreateInstance(XMLStateConstructorParams stateData)
{
    return XMLStateFactoryBasePtr(new DSControllerTest(stateData));
}

