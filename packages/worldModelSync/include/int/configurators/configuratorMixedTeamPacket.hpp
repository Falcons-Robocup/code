// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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
