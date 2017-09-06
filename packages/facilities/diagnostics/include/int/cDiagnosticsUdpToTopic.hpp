 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDiagnosticsUdpToTopic.hpp
 *
 *  Created on: Jan 10, 2016
 *      Author: Jan Feitsma
 */

#ifndef CDIAGNOSTICS_UDP2TOPIC_HPP_
#define CDIAGNOSTICS_UDP2TOPIC_HPP_

#include <ros/ros.h>
#include <exception>
#include <string>
#include "FalconsCommon.h" 
#include "timeConvert.hpp" 
#include "cReceiverUDP.hpp"


namespace diagnostics
{

template <typename T>
class cDiagnosticsUdpToTopic : public Facilities::Network::cAbstractObserverByteArray
{
    public:
        cDiagnosticsUdpToTopic(std::string const &addr, int port, std::string const &topic)
        {
            TRACE("constructing cDiagnosticsUdpToTopic with addr=%s port=%d topic=%s", addr.c_str(), port, topic.c_str());
            // advertise the topic
            _topic = topic;
            _pub = _nh.advertise<T>(_topic, 20.0);
            // construct a UDP receiver
            _recv = new Facilities::Network::cReceiverUDP(addr, port);
            _recv->attachObserver(this);
            _good = true;
            TRACE("construction of cDiagnosticsUdpToTopic complete");
        }
        
        ~cDiagnosticsUdpToTopic()
        {
        	delete(_recv);
        	_recv = NULL;
        }

    private:
        ros::NodeHandle _nh;
        ros::Publisher  _pub;
        std::string     _topic;
        bool            _good;
        Facilities::Network::cReceiverUDP *_recv;
        T _lastMsg;
        void notifyNewPacket(Facilities::Network::cByteArray &data)
        {
            if (!_good) return;
            // in principal NO tracing here!!! this function is called far too often
            //TRACE("got packet, publishing to %s", _topic.c_str());
            try
            {
                // convert to message
                std::vector<uint8_t> v;
        		data.getData(v);
        		assert(v.size() < 1e6);
                uint8_t buf[v.size()];
                for (size_t u = 0; u < v.size(); ++u)
                {
                    buf[u] = v[u];
                }
                
                // statistics for analyzing network load: report only each 1000 packets 
                // use maps with topic as key to distinguish between all channels 
                static std::map<std::string, int> counter;
                static std::map<std::string, double> tlast;
                static std::map<std::string, int> numBytes;
                numBytes[_topic] += v.size();
                int N = 1000;
                if ((counter[_topic] % N) == 0)
                {
                    double elapsed = getTimeNow() - tlast[_topic];
                    double rate = numBytes[_topic] / elapsed / 1024.0;
                    double freq = N / elapsed;
                    double avgSize = numBytes[_topic] * 1.0 / N;
                    TRACE("topic=%s counter=%d freq=%.1fHz rate=%.1fKB/s avgSize=%.1fB", _topic.c_str(), counter[_topic], freq, rate, avgSize);
                    // reset
                    tlast[_topic] = getTimeNow();
                    numBytes[_topic] = 0;
                }
                counter[_topic]++;
                
                // TODO: in case message structure is changed, then buffer overrun may occur, which can lockup your laptop - how to be robust? 
		        ros::serialization::IStream stream(buf, v.size());
		        ros::serialization::deserialize(stream, _lastMsg);
                // publish to the topic
                _pub.publish(_lastMsg);
            }
            catch (std::exception &e)
            {
                _good = false;
                TRACE("failed to serialize/publish message to topic %s, reason: %s", _topic.c_str(), e.what());
                ROS_ERROR("failed to serialize/publish message to topic %s, reason: %s", _topic.c_str(), e.what());
            }
            
        }
};

} // end of namespace diagnostics

#endif /* CDIAGNOSTICS_UDP2TOPIC_HPP_ */
