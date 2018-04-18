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

#ifndef _ARMARX_XMLUSERCODE_DSController_DSControllerGroup_DSControllerTest_H
#define _ARMARX_XMLUSERCODE_DSController_DSControllerGroup_DSControllerTest_H

#include <DSController/statecharts/DSControllerGroup/DSControllerTest.generated.h>

namespace armarx
{
    namespace DSControllerGroup
    {
        class DSControllerTest :
            public DSControllerTestGeneratedBase < DSControllerTest >
        {
        public:
            DSControllerTest(const XMLStateConstructorParams& stateData):
                XMLStateTemplate < DSControllerTest > (stateData), DSControllerTestGeneratedBase < DSControllerTest > (stateData)
            {
            }

            // inherited from StateBase
            void onEnter() override;
            void run() override;
            // void onBreak() override;
            void onExit() override;

            // static functions for AbstractFactory Method
            static XMLStateFactoryBasePtr CreateInstance(XMLStateConstructorParams stateData);
            static SubClassRegistry Registry;

            // DO NOT INSERT ANY CLASS MEMBERS,
            // use stateparameters instead,
            // if classmember are neccessary nonetheless, reset them in onEnter
        };
    }
}

#endif
