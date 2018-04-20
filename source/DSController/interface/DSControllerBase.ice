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
 * @package    RobotAPI::NJointControllerInterface
 * @author     Mirko Waechter ( mirko dot waechter at kit dot edu )
 * @date       2017
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#pragma once

#include <RobotAPI/interface/units/RobotUnit/NJointController.ice>

module armarx
{
    class DSControllerConfig extends NJointControllerConfig
    {
        float posiKp = 5;
        float v_max = 0.15;
        float posiDamping = 10;

        float oriDamping;
        float oriKp;

        float filterTimeConstant = 0.01;
        float torqueLimit = 1;


        string nodeSetName = "";
        string tcpName = "";

        Ice::FloatSeq desiredPosition;
        Ice::FloatSeq desiredQuaternion;

        Ice::FloatSeq qnullspaceVec;

        float nullspaceKp;
        float nullspaceDamping;


        Ice::StringSeq gmmParamsFiles;
    };

    interface DSControllerInterface extends NJointControllerInterface
    {

    };

};
