 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cByteArray.cpp
 *
 *  Created on: Aug 11, 2015
 *      Author: Tim Kouters
 */

#include "ext/cByteArray.hpp"

#include <stdlib.h>
#include <string.h>
#include "falconsCommon.hpp"


using namespace Facilities;


/*!
 * \brief Constructor for the Byte Array class
 *
 * This class is a representation of a byte array
 *
 */
cByteArray::cByteArray()
{
}

/*!
 * \brief Destructor for the byte array class
 *
 * \note Ensures the data within this class is freed
 */
cByteArray::~cByteArray()
{
	// no action needed here due to using vector
}

/*!
 * \brief Get the size of the byte array in this class
 *
 * \retval size_t size of the array in bytes
 *
 * \note Optional note
 */
size_t cByteArray::getSize()
{
	return _data.size();
}


/*!
 * \brief Get the data
 *
 * \param[out] std::vector<uint8_t> &data Reference to the buffer container
 *
 * \retval void None
 *
 * \note The output data itself is a pointer to the real data.
 *       Data is not copied.
 */
void cByteArray::getData(std::vector<uint8_t> &data)
{
    data = _data;
}

/*!
 * \brief Set byte array data
 *
 * This function copies the data.
 *
 * \param[in] std::vector<uint8_t> &data Reference to the buffer container
 */
void cByteArray::setData(std::vector<uint8_t> const &data)
{
    _data = data;
}


