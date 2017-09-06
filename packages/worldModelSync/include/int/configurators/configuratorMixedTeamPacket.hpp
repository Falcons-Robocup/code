 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * configuratorMixedTeamPacket.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: robocup
 */

#ifndef CONFIGURATORMIXEDTEAMPACKET_HPP_
#define CONFIGURATORMIXEDTEAMPACKET_HPP_

#include <string>

class configuratorMixedTeamPacket
{
	public:
		/*!
		 * \brief Obtain an instance of this singleton class
		 */
		static configuratorMixedTeamPacket& getInstance()
		{
			static configuratorMixedTeamPacket instance;
			return instance;
		}

		void setMulticastAddress(const std::string address);
		void setPortNumber(const unsigned int number);
		void setLoopIsEnabled(const bool isEnabled);
		void setMaxHops(const unsigned int nrHops);
		void setGroupID(const unsigned int groupID);
		void setMixedTeamEnabled(const bool isEnabled);

		std::string getMulticastAddress() const;
		unsigned int getPortNumber() const;
		bool getLoopIsEnabled() const;
		unsigned int getMaxHops() const;
		unsigned int getGroupID() const;
		bool getMixedTeamIsEnabled() const;

	private:
		std::string _multicastAddress;
        unsigned int _portNumber;
        bool _loopEnabled;
        unsigned int _maxHops;
        unsigned int _groupID;
        bool _mixedIsEnabled;

		configuratorMixedTeamPacket();
		~configuratorMixedTeamPacket();
};

#endif /* CONFIGURATORMIXEDTEAMPACKET_HPP_ */
