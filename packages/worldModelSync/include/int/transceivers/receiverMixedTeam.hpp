// Copyright 2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * receiverMixedTeam.hpp
 *
 *  Created on: Jan 21, 2017
 *      Author: robocup
 */

#ifndef RECEIVERMIXEDTEAM_HPP_
#define RECEIVERMIXEDTEAM_HPP_

#include <boost/function.hpp>

#include "cReceiverUDP.hpp"
#include "cAbstractObserverByteArray.hpp"
#include "cByteArray.hpp"

class receiverMixedTeam : public Facilities::cAbstractObserverByteArray
{
	public:
		receiverMixedTeam();
		~receiverMixedTeam();

		void reconnect();
		virtual void notifyNewPacket(Facilities::cByteArray &data);

		void setNotifyNewPacket(boost::function<void(Facilities::cByteArray)> func)
		{
		  _fncNotifyNewPacket = func;
		}
	private:
		boost::function<void(Facilities::cByteArray)> _fncNotifyNewPacket;
		Facilities::Network::cReceiverUDP *_udpReceiver;

		void cleanUpReceiver();
};

#endif /* RECEIVERMIXEDTEAM_HPP_ */
