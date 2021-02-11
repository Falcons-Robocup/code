// Copyright 2016 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * configuratorWorldModelPacket.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#ifndef CONFIGURATORWORLDMODELPACKET_HPP_
#define CONFIGURATORWORLDMODELPACKET_HPP_

#include <string>

class configuratorWorldModelPacket
{
	public:
		/*!
		 * \brief Obtain an instance of this singleton class
		 */
		static configuratorWorldModelPacket& getInstance()
		{
			static configuratorWorldModelPacket instance;
			return instance;
		}

		void setMulticastAddress(const std::string address);
		void setPortNumber(const unsigned int number);
		void setLoopIsEnabled(const bool isEnabled);
		void setMaxHops(const unsigned int nrHops);
		void setGroupID(const unsigned int groupID);

		std::string getMulticastAddress() const;
		unsigned int getPortNumber() const;
		bool getLoopIsEnabled() const;
		unsigned int getMaxHops() const;
		unsigned int getGroupID() const;

	private:
		std::string _multicastAddress;
        unsigned int _portNumber;
        bool _loopEnabled;
        unsigned int _maxHops;
        unsigned int _groupID;

		configuratorWorldModelPacket();
		~configuratorWorldModelPacket();
};

#endif /* CONFIGURATORWORLDMODELPACKET_HPP_ */
