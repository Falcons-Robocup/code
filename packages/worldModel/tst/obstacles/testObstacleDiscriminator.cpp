 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 
#include "int/administrators/IobstacleDiscriminator.hpp"
#include "int/administrators/obstacleDiscriminator.hpp"
#include "obstacleMeasurement.hpp"
#include "int/types/robot/robotType.hpp"
#include "cDiagnostics.hpp"
#include "int/adapters/RTDB/RTDBOutputAdapter.hpp"
#include "RtDB2.h"

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cstdio>
#include <vector>
#include <map>
#include <fstream>
#include <queue>

class TraceLoader
{
public:
    static std::list<std::vector<obstacleMeasurement> > loadMeasurementsFromPTraceFile(char* filename)
    {
        std::list<std::vector<obstacleMeasurement> > fullMeasurements;
        std::vector<obstacleMeasurement> singleMeasurement;

        std::ifstream infile(filename);

        std::string line;
        while (std::getline(infile, line))
        {
            try
            {
                // trim
                boost::trim_if(line, boost::is_any_of("\t "));

                // ignore comment line
                if (line.size() > 0 && line[0] == '#')
                {
                    continue;
                }

                // split words
                std::vector<std::string> words;
                boost::split(words, line, boost::is_any_of("\t \n"), boost::token_compress_on);

                if (words.size() >= 2)
            	{
            		std::string traceMarker = words[1];

            		if(	traceMarker == "OM")
            		{
            			if (words.size() != 15)
            			{
            				printf("Error processing line: %s\n", line.c_str());
            			}
            			else
            			{
            				int index		  = boost::lexical_cast<int>   (words[2]);
            				int robotId	      = boost::lexical_cast<int>   (words[3]);
            				int uniqueId	  = boost::lexical_cast<int>   (words[4]);
            				double timestamp  = boost::lexical_cast<double>(words[5]);
            				//int cameraType	  = boost::lexical_cast<int>   (words[6]);
            				double confidence = boost::lexical_cast<double>(words[7]);
            				double azimuth	  = boost::lexical_cast<double>(words[8]);
            				double elevation  = boost::lexical_cast<double>(words[9]);
            				double radius	  = boost::lexical_cast<double>(words[10]);
            				double cameraX	  = boost::lexical_cast<double>(words[11]);
            				double cameraY	  = boost::lexical_cast<double>(words[12]);
            				double cameraZ	  = boost::lexical_cast<double>(words[13]);
            				double cameraPhi  = boost::lexical_cast<double>(words[14]);

            				obstacleMeasurement obstacleMeasurement;

            				obstacleMeasurement.identifier = uniqueObjectID(robotId, uniqueId);
            				obstacleMeasurement.timestamp = timestamp;
            				obstacleMeasurement.confidence = confidence;
            				obstacleMeasurement.cameraX = cameraX;
            				obstacleMeasurement.cameraY = cameraY;
            				obstacleMeasurement.cameraZ = cameraZ;
            				obstacleMeasurement.cameraPhi = cameraPhi;
            				obstacleMeasurement.azimuth = azimuth;
            				obstacleMeasurement.elevation = elevation;
            				obstacleMeasurement.radius = radius;

            				if(index == 0)
            				{
            					if(singleMeasurement.size() > 0)
            					{
            						fullMeasurements.push_back(singleMeasurement);
            					}
            					singleMeasurement.clear();
            				}

            				singleMeasurement.push_back(obstacleMeasurement);

            			}
            		}
            	}
            }
            catch (std::exception &e)
            {
                printf("ERROR: could not parse line: %s\n", line.c_str());
                break;
            }
        }

        return fullMeasurements;
    }

private:
};

class CommandReceiver
{
public:

    enum CommandEnum
    {
        NO_COMMAND,
        PLAY,
        PAUSE,
        REWIND,
        STEP_FORWARD,
        STEP_BACK
    };

    CommandReceiver() :
        socket(boost::asio::ip::udp::socket(io_service, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), 7060)))
    {
        boost::thread thrd( boost::bind( &CommandReceiver::receiveLoop, this));

        commandMap["PLAY"] = PLAY;
        commandMap["PAUSE"] = PAUSE;
        commandMap["REWIND"] = REWIND;
        commandMap["STEP_FORWARD"] = STEP_FORWARD;
        commandMap["STEP_BACK"] = STEP_BACK;
    }

    CommandEnum getCommand()
    {
        CommandEnum command = NO_COMMAND;

        if(!commandQueue.empty())
        {
            command = commandQueue.front();
            commandQueue.pop();
        }

        return command;
    }

private:

    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket;
    std::queue<CommandEnum> commandQueue;
    std::map<std::string, CommandEnum> commandMap;

    void receiveLoop()
    {
        char reply[1024];
        boost::asio::ip::udp::endpoint sender_endpoint;

        while(1)
        {
            size_t reply_length = socket.receive_from(boost::asio::buffer(reply, 1024), sender_endpoint);
            commandQueue.push(commandMap[std::string(reply, reply_length)]);
        }
    }
};


class TracePlayback
{
public:
    TracePlayback(char* filename, int type) :
    	filename(filename),
    	playbackState(CommandReceiver::PAUSE),
    	initialTime(0),
    	lastStepTime(0),
    	stepDelay(DEFAULT_STEP_SPEED),
    	currentTraceTime(-std::numeric_limits<double>::infinity()),
    	_rtdb(RtDB2Store::getInstance().getRtDB2(getRobotNumber()))
    {
    	if(type==0)
    	{
    	    ; // old implementation was removed
    	}
    	else
    	{
    		discriminator = new obstacleDiscriminator();
    		// this used to map to obstacleDiscriminatorGaussian, but it was later renamed to obstacleDiscriminator
    	}
    }

    ~TracePlayback()
    {
    	delete discriminator;
    }

    void runPlayback()
    {
    	while(1)
    	{
    		switch(playbackState)
    		{
    			case CommandReceiver::NO_COMMAND:
    				usleep(100 * 1000);
    				break;

    			case CommandReceiver::PAUSE:
    				stepDelay = DEFAULT_STEP_SPEED;
    				usleep(100 * 1000);
    				break;

    			case CommandReceiver::PLAY:
    				sleepUntilNextStep();
    				stepAdvanceObstacles();
    				break;

    			case CommandReceiver::REWIND:
    				sleepUntilNextStep();
    				stepRewindObstacles();
    				break;

    			case CommandReceiver::STEP_FORWARD:
    				stepAdvanceObstacles();
    				playbackState = CommandReceiver::PAUSE;
    				break;

    			case CommandReceiver::STEP_BACK:
    				stepRewindObstacles();
    				playbackState = CommandReceiver::PAUSE;
    				break;
    		}

    		processCommand();
    	}
    }

    void processCommand()
    {
    	CommandReceiver::CommandEnum newCommand = commandReceiver.getCommand();

    	switch(newCommand)
    	{
    		case CommandReceiver::NO_COMMAND:
    			break;

    		case CommandReceiver::PAUSE:
    			playbackState = newCommand;
    			break;

    		case CommandReceiver::PLAY:
    			if(playbackState == CommandReceiver::PLAY)
    			{
    				stepDelay /= 2;
    			}
    			else if(playbackState == CommandReceiver::REWIND)
    			{
    				stepDelay *= 2;
    			}
    			else
    			{
    				playbackState = newCommand;
    			}
    			break;

    		case CommandReceiver::REWIND:
    			if(playbackState == CommandReceiver::REWIND)
    			{
    				stepDelay /= 2;
    			}
    			else if(playbackState == CommandReceiver::PLAY)
    			{
    				stepDelay *= 2;
    			}
    			else
    			{
    				playbackState = newCommand;
    			}
    			break;

    		case CommandReceiver::STEP_FORWARD:
    			playbackState = newCommand;
    			break;

    		case CommandReceiver::STEP_BACK:
    			playbackState = newCommand;
    			break;
    	}


    }

    void startPlayback()
    {
    	loadFile();

    	runPlayback();
    }

private:

    char* filename;
    CommandReceiver commandReceiver;
    CommandReceiver::CommandEnum playbackState;
    double initialTime;
    double lastStepTime;
    double stepDelay;
    double currentTraceTime;
    RtDB2* _rtdb;

    static constexpr double DEFAULT_STEP_SPEED = 0.01;

    IobstacleDiscriminator* discriminator;

    // These data structures must be lists to keep the iterators valid while
    // data is added to the list
    std::list<std::vector<obstacleMeasurement> > measurementsList;
    std::list<std::vector<obstacleClass_t> > resultObstaclesList;
    std::vector<robotClass_t> teamMembersInclSelf;

    std::list<std::vector<obstacleMeasurement> >::iterator measurementsIterator;
    std::list<std::vector<obstacleClass_t> >::iterator obstaclesIterator;

    void loadFile()
    {
    	measurementsList = TraceLoader::loadMeasurementsFromPTraceFile(filename);
    	printf("Loaded %lu measurements\n", measurementsList.size());

    	measurementsIterator = measurementsList.begin();
    	obstaclesIterator = resultObstaclesList.begin();

    	initialTime = (*measurementsIterator)[0].timestamp;
    }

    void sleepUntilNextStep()
    {
    	double missing_dt = stepDelay - (rtime::now().toDouble() - lastStepTime);

    	if(missing_dt > 0)
    	{
    		usleep(missing_dt * 1000000);
    	}
    }

    void stepAdvanceObstacles()
    {
    	if(obstaclesIterator == resultObstaclesList.end())
    	{
    		stepAdvanceMeasurements();
    		obstaclesIterator = resultObstaclesList.end();
    		obstaclesIterator--;
    	}

    	sendDiagnostics(*obstaclesIterator);
    	lastStepTime = rtime::now().toDouble();
    	obstaclesIterator++;
    }

    void stepRewindObstacles()
    {
    	if(obstaclesIterator == resultObstaclesList.end())
    	{
    		obstaclesIterator = resultObstaclesList.end();
    		obstaclesIterator--;
    	}

    	sendDiagnostics(*obstaclesIterator);
    	lastStepTime = rtime::now().toDouble();

    	if(obstaclesIterator != resultObstaclesList.begin())
    	{
    		obstaclesIterator--;
    	}
    	else
    	{
    		playbackState = CommandReceiver::PAUSE;
    	}

    }

    void stepAdvanceMeasurements()
    {
        if(measurementsIterator != measurementsList.end())
    	{
    		addMeasurementToDiscriminator(*measurementsIterator);
    		measurementsIterator++;
    	}
    	else
    	{
    		playbackState = CommandReceiver::PAUSE;
    	}
    }

    void removeDuplicateMeasurements(std::vector<obstacleMeasurement>& measurement)
    {
    	for(auto it=measurement.begin(); it!=measurement.end(); it++)
    	{
    		for(auto jt=it+1; jt!=measurement.end();)
    		{
    			double delta = 0.0;
    			delta += pow(jt->cameraX - it->cameraX, 2);
    			delta += pow(jt->cameraY - it->cameraY, 2);
    			delta += pow(jt->cameraZ - it->cameraZ, 2);
    			delta += pow(jt->azimuth - it->azimuth, 2);
    			delta += pow(jt->cameraPhi - it->cameraPhi, 2);
    			delta += pow(jt->radius - it->radius, 2);

    			if(delta < 0.001)
    			{
    				jt = measurement.erase(jt);
    			}
    			else
    			{
    				jt++;
    			}
    		}
    	}
    }

    void addMeasurementToDiscriminator(std::vector<obstacleMeasurement> measurement)
    {
    	teamMembersInclSelf.clear();

    	removeDuplicateMeasurements(measurement);

    	for(auto it=measurement.begin(); it!=measurement.end(); it++)
    	{
    		discriminator->addMeasurement(*it);
    		robotClass_t teamRobot;
    		teamRobot.setCoordinates(it->cameraX, it->cameraY, 0.0);
    		teamRobot.setTimestamp(it->timestamp.toDouble());
    		teamRobot.setRobotID(it->identifier.robotID);

    		currentTraceTime = std::max(currentTraceTime, it->timestamp.toDouble());

    		if(teamMembersInclSelf.empty())
    		{
    			teamMembersInclSelf.push_back(teamRobot);
    		}
    	}

    	discriminator->performCalculation(currentTraceTime, teamMembersInclSelf);
    	resultObstaclesList.push_back(discriminator->getObstacles());
    }

//    void sendDiagnostics(const std::vector<obstacleClass_t>& obstacles)
//    {
//        if(obstacles.size() > 0)
//        {
//    	    printf("t=%f obstacles=%ld\n", (obstacles[0].getTimestamp()-initialTime), obstacles.size());
//    	    fflush(stdout);
//
//    	    rosMsgs::t_diag_wm_obstacles diagMsg;
//    	    diagMsg.numTrackers = (int)obstacles.size();
//    	    for(auto it = obstacles.begin(); it != obstacles.end(); it++)
//    	    {
//    		    rosMsgs::t_obstacle obstMsg;
//    		    obstacleClass_t obstacle = *it;
//    		    obstMsg.id = obstacle.getId();
//    		    obstMsg.x = obstacle.getX();
//    		    obstMsg.y = obstacle.getY();
//    		    obstMsg.vx = obstacle.getVX();
//    		    obstMsg.vy = obstacle.getVY();
//    		    obstMsg.confidence = obstacle.getConfidence();
//    		    diagMsg.obstacles.push_back(obstMsg);
//
//    		    //printf("sending: %d %f %f %f %f %f\n", obstMsg.id, obstMsg.x, obstMsg.y, obstMsg.vx, obstMsg.vy, obstMsg.confidence);
//    	    }
//    	    diagSender.set(diagMsg);
//    	    diagSender.send();
//        }
//    }

    void sendDiagnostics(const std::vector<obstacleClass_t>& obstacles)
    {
    	if(obstacles.size() > 0)
    	{
    		printf("t=%f obstacles=%ld\n", (obstacles[0].getTimestamp()-initialTime), obstacles.size());

    		T_OBSTACLES obstacleResults;
    		for (auto it = obstacles.begin(); it != obstacles.end(); it++)
    		{
    			obstacleResult obst;
    			obst.position.x = it->getX();
    			obst.position.y = it->getY();
    			obst.velocity.x = it->getVX();
    			obst.velocity.y = it->getVY();
    			obst.confidence = it->getConfidence();
    			obst.id = 0;
    			obstacleResults.push_back(obst);
    		}

    		_rtdb->put(OBSTACLES, &obstacleResults);
    	}
    }
};


void setEnvironment(int argc, char **argv)
{
    setenv("ROS_NAMESPACE","/teamA/robot2", 1);
    setenv("TURTLE5K_ROBOTNUMBER","2", 1);
}

int main(int argc, char **argv)
{
    setEnvironment(argc, argv);

    obstacleDiscriminator g;

    if(argc < 2)
    {
        printf("Error: expected trace file name as argument\n");
    }
    else
    {
        int type = 0;
        if(argc > 2)
        {
            if(argv[2][0] == 'g')
            {
            	type = 1;
            }
        }

        TracePlayback tracePlayback(argv[1], type);

        tracePlayback.startPlayback();
    }

    printf("testObstacleDiscriminator finished\n");

    return 0;
}
