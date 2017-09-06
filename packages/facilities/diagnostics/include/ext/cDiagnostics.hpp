 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnostics.hpp
 *
 *  Created on: Jan 05, 2016
 *      Author: Jan Feitsma
 */

#ifndef CDIAGNOSTICS_HPP_
#define CDIAGNOSTICS_HPP_

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <string>
#include <vector>
#include <boost/thread.hpp>
#include "ext/cDiagnosticsEnum.hpp"
#include "int/cDiagnosticsUdpToTopic.hpp"
#include "FalconsCommon.h" 
#include "ports.hpp"
#include "addresses.hpp"
#include "cTransmitterUDP.hpp"
#include "boost/thread/mutex.hpp"


// buffer size is limited by cTransmitterUDP BUFSIZE
#define MAX_PACKET_SIZE 65536


namespace diagnostics
{


/*
 * Sender class to be run on robot.
 * It will send the given packet via UDP to any coach laptop.
 * Examples: see robotControl/cDiagnosticsAdapter
 */
template<typename T>
class cDiagnosticsSender
{
    public:
        cDiagnosticsSender(diagEnum id, double freq, bool sendAfterSet = true)
        {
            TRACE("constructing cDiagnosticsSender with id=%d, sendAfterSet=%d, freq=%e", (int)id, (int)sendAfterSet, freq);
            // setup UDP transmitter
            int robotNum = getRobotNumber();
            _port = Facilities::Network::getPort(Facilities::Network::PORT_DIAG_BASE, robotNum) + (int)id;
            _sendAfterSet = sendAfterSet;
            std::string multicastAddress = Facilities::Network::getMulticastAddress();
            _transmitterUDP = new Facilities::Network::cTransmitterUDP(multicastAddress, _port, true, getHops());
            _freq = freq;
            _timerThread = new boost::thread(boost::bind(&cDiagnosticsSender<T>::timer, this));
            //TRACE("constructed cDiagnosticsSender with port=%d", _port);
            _connectionDevice = GetPrimaryConnectionType();
        }
        
        ~cDiagnosticsSender()
        {   
            //TRACE("destructing cDiagnosticsSender");
            _timerThread->join();
            delete _timerThread;
            delete _transmitterUDP;
            //TRACE("destructing complete for cDiagnosticsSender");
        }

        /*
         * Send a dedicated data packet on robot.
         */
        void send()
        {
            try
            {
            	_sendDiagMtx.lock();

                // check serialization length
	            uint32_t serial_size = ros::serialization::serializationLength(_msg);
	            if (serial_size >= MAX_PACKET_SIZE) 
	            {
	                TRACE("diagnostics message of size %d exceeds maximum length of %d", serial_size, MAX_PACKET_SIZE);
	                ROS_ERROR("diagnostics message of size %d exceeds maximum length of %d", serial_size, MAX_PACKET_SIZE);
            		return;
            	}
                // convert message to byte array
		        ros::serialization::OStream stream(_buf, MAX_PACKET_SIZE);
		        ros::serialization::serialize(stream, _msg);
                Facilities::Network::cByteArray packet;
                std::vector<uint8_t> v(_buf, _buf + serial_size);
           		packet.setData(v);
		        // send UDP packet
           		if((!_transmitterUDP->isSocketOpen()) ||
           		  (GetPrimaryConnectionType() != _connectionDevice))
           		{
           			reconnect();
           		}
		        _transmitterUDP->sendPacket(&packet);

		        _sendDiagMtx.unlock();
            }
            catch (std::exception &e)
            {
                TRACE("failed to send message over UDP, reconnecting");
                ROS_ERROR("failed to send message over UDP, reconnecting");

                reconnect();
                _sendDiagMtx.unlock();
                return;
            }            
            
        }
        
        /*
         * Get / set latest message.
         * Useful when filling in only parts.
         */
        T &get()
        {
            boost::lock_guard<boost::mutex> guard(_mtx);
            return _msg;
        }
        void set(T const &data)
        {
            boost::lock_guard<boost::mutex> guard(_mtx);
            _msg = data;
            if (_sendAfterSet)
            {
                send();
            }
        }
        
        /*
         * Optionally install a callback to be called from the timer, just before sending.
         */
        void installCallback(boost::function<void()> const &f)
        {
            _preSend = f;
        }

    private:
        T       _msg;
        int     _port;
        uint8_t _buf[MAX_PACKET_SIZE];
        bool    _sendAfterSet;
        double  _freq;
        boost::function<void(void)> _preSend;
        Facilities::Network::cTransmitterUDP *_transmitterUDP;
        boost::thread *_timerThread;
        boost::mutex _mtx;
        boost::mutex _sendDiagMtx;
        connectionType _connectionDevice;
        
        void timer()
        {
            if (_freq > 0.0)
            {
                while (true)
                {
                    if (!_preSend.empty())
                    {
                        _preSend();
                    }
                    send();
                    boost::this_thread::sleep(boost::posix_time::milliseconds(1000.0 / _freq));
                    // TODO this timer slightly drifts. Use something more stable like ros::rate.sleep or boost asio timer
                }
            }
        }
        
        void reconnect()
		{
			TRACE("reconnect");
			try
			{
				TRACE("cleanUpTransmitter");
				cleanUpTransmitter();

				TRACE("construct");
				std::string multicastAddress = Facilities::Network::getMulticastAddress();
				_transmitterUDP = new Facilities::Network::cTransmitterUDP(multicastAddress, _port, true, getHops());
				TRACE("construct succeeded");

				if(!_transmitterUDP->isSocketOpen())
				{
					TRACE("recurse");
					reconnect();
				}

				_connectionDevice = GetPrimaryConnectionType();

				TRACE("reconnected");
			}
			catch(std::exception &e)
			{
				TRACE("Caught exception: %s", e.what());
				std::cout << "Caught exception: " << e.what() << std::endl;
				throw std::runtime_error(std::string("Linked to: ") + e.what());
			}
		}

        void cleanUpTransmitter()
		{
			try
			{
				if(_transmitterUDP != NULL)
				{
					delete _transmitterUDP;
					_transmitterUDP = NULL;

					/*
					 * We need to sleep in order to avoid hang ups due to Linux socket
					 */
					TRACE("sleep");
					sleep(20.0);
				}
			}
			catch(std::exception &e)
			{
				TRACE("Caught exception: %s", e.what());
				std::cout << "Caught exception: " << e.what() << std::endl;
				throw std::runtime_error(std::string("Linked to: ") + e.what());
			}
		}

};


/*
 * Receiver class to be run on (any) coach laptop.
 * It will listen on all ports for all robots and relay data to topics, so other coach software can process it.
 *
 * NOTE: JFEI: I am not so sure how well this will scale threading wise. Perhaps we need to use multiple threads in a 
 * round-robin fashion or something in order to avoid latency or even saturation. 
 */
class cDiagnosticsReceiver
{
    public:
        cDiagnosticsReceiver();
        ~cDiagnosticsReceiver();
        
    private:
        ros::NodeHandle _nh;
        std::vector<Facilities::Network::cAbstractObserverByteArray *> _converters;
        
        /*
         * Internal helper to define a diagnostics channel and listen for each robot.
         * Called from cDiagnosticsAdmin constructor.
         *
         * Templates are required in order to generate all code compile-time in the cDiagnosticsReceiver constructor.
         * (Runtime topic definition is not supported by ROS - it is also template-based).
         */
        template<typename T>
        void add(diagEnum id, std::string const &topicName, bool withCoach=false)
        {
            // for each robot 1..6
            std::string m = Facilities::Network::getMulticastAddress();
            for (int robotNum = 1; robotNum <= 6; ++robotNum)
            {
                std::string topic = "robot" + boost::lexical_cast<std::string>(robotNum) + "/" + topicName;
                int port = Facilities::Network::getPort(Facilities::Network::PORT_DIAG_BASE, robotNum) + (int)id;
                TRACE("setting up UDP2topic relay with topic %s for diagnostics id %d at port %d", topic.c_str(), (int)id, port);
                _converters.push_back(new cDiagnosticsUdpToTopic<T>(m, port, topic));
            }
            if (withCoach)
            {
                std::string topic = topicName;
                int port = Facilities::Network::getPort(Facilities::Network::PORT_DIAG_BASE, 0) + (int)id;
                TRACE("setting up @coach UDP2topic relay with topic %s for diagnostics id %d at port %d", topic.c_str(), (int)id, port);
                _converters.push_back(new cDiagnosticsUdpToTopic<T>(m, port, topic));
            }
        }
        
};

} // end of namespace diagnostics

#endif /* CDIAGNOSTICS_HPP_ */
