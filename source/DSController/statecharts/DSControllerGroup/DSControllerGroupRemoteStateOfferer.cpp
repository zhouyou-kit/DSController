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
 * @package    DSController::DSControllerGroup::DSControllerGroupRemoteStateOfferer
 * @author     Mahdi Khoramshahi ( m80 dot khoramshahi at gmail dot com )
 * @date       2018
 * @copyright  http://www.gnu.org/licenses/gpl-2.0.txt
 *             GNU General Public License
 */

#include "DSControllerGroupRemoteStateOfferer.h"

using namespace armarx;
using namespace DSControllerGroup;

// DO NOT EDIT NEXT LINE
DSControllerGroupRemoteStateOfferer::SubClassRegistry DSControllerGroupRemoteStateOfferer::Registry(DSControllerGroupRemoteStateOfferer::GetName(), &DSControllerGroupRemoteStateOfferer::CreateInstance);



DSControllerGroupRemoteStateOfferer::DSControllerGroupRemoteStateOfferer(StatechartGroupXmlReaderPtr reader) :
    XMLRemoteStateOfferer < DSControllerGroupStatechartContext > (reader)
{
}

void DSControllerGroupRemoteStateOfferer::onInitXMLRemoteStateOfferer()
{

}

void DSControllerGroupRemoteStateOfferer::onConnectXMLRemoteStateOfferer()
{

}

void DSControllerGroupRemoteStateOfferer::onExitXMLRemoteStateOfferer()
{

}

// DO NOT EDIT NEXT FUNCTION
std::string DSControllerGroupRemoteStateOfferer::GetName()
{
    return "DSControllerGroupRemoteStateOfferer";
}

// DO NOT EDIT NEXT FUNCTION
XMLStateOffererFactoryBasePtr DSControllerGroupRemoteStateOfferer::CreateInstance(StatechartGroupXmlReaderPtr reader)
{
    return XMLStateOffererFactoryBasePtr(new DSControllerGroupRemoteStateOfferer(reader));
}



