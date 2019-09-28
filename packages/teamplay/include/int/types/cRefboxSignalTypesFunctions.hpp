 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRefboxSignalTypesFunctions.hpp
 * Contains simple conversion functions that are only used in some callers, to avoid w_unused function from many includes
 *
 *  Created on: July 7, 2016
 *      Author: Martijn van Veen
 */

#ifndef CREFBOXSIGNALTYPESFUNCTIONS_HPP_
#define CREFBOXSIGNALTYPESFUNCTIONS_HPP_

#include <iostream>
#include <string>
#include <map>

#include "cRefboxSignalTypes.hpp"

static refboxSignalEnum refBoxSignalStrToEnum(const std::string enumString)
{
	if(refboxSignalMapping.find(enumString) == refboxSignalMapping.end())
	{
		std::cout << enumString << std::endl;
	}
	return	refboxSignalMapping.at(enumString);
}

/*
 // this looked useful but actually it's not, apparently
static std::string refBoxSignalEnumToStr(const  refboxSignalEnum RefBoxSigEnum)
{
	 try
	 {
		std::map<std::string, refboxSignalEnum>::const_iterator it;
		for(it = refboxSignalMapping.begin(); it != refboxSignalMapping.end(); ++it)
		{
			if(it->second == RefBoxSigEnum)
			{
				return it->first;
			}
		}
	}
	catch (std::exception &e)
		{
			TRACE_ERROR("Caught exception: %s.", e.what());
			std::cout << "Caught exception: " << e.what() << std::endl;
		}
	TRACE("Unknown RefBox Signal received!");
	return "INVALID";
}
*/

#endif /* CREFBOXSIGNALTYPESFUNCTIONS_HPP_ */


