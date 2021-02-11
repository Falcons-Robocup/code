// Copyright 2016-2017 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * receiverWorldModel.hpp
 *
 *  Created on: Oct 11, 2016
 *      Author: Tim Kouters
 */

#ifndef RECEIVERWORLDMODEL_HPP_
#define RECEIVERWORLDMODEL_HPP_

#include <boost/function.hpp>

#include "cReceiverUDP.hpp"
#include "cAbstractObserverByteArray.hpp"
#include "cByteArray.hpp"

class receiverWorldModel : public Facilities::cAbstractObserverByteArray
{
	public:
		receiverWorldModel();
		~receiverWorldModel();

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

#endif /* RECEIVERWORLDMODEL_HPP_ */
