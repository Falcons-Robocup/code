 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 * File: fsm.hpp
 * Author: Jan Feitsma
 * Creation: 2014-12-27 based on private FSM
 * Description: FiniteStateMachine.
 * 
 */

#ifndef _INCLUDED_FSM_HPP_
#define _INCLUDED_FSM_HPP_


// required headers
#include <vector>
#include <string>
#include <map>
#include <boost/function.hpp>


// define an Event? Internal vs External? 
// define base class StateData?


struct State 
{
	int id;
	std::string name;
	// optional: execute when entering or leaving state
	boost::function<void(void)> entryAction; // not yet used
	boost::function<void(void)> exitAction; // not yet used
	// constructor(s)
	State(int i, std::string s);
};

struct Transition
{
	int fromState;
	int toState;
	// required: test if this Transition is to be executed
	boost::function<bool(void)> condition;
	// optional: if condition holds, execute this action (normally to update some secondary data)
	boost::function<void(void)> action; // not yet used
	// constructor(s)
	Transition(int from, int to, boost::function<bool()> cond);
};

// TODO EVENT


class FiniteStateMachine
{

    private:

	// state during FSM simulation
	int m_current_state;

	// list of states
	std::vector<State> m_states;

	// reverse lookup on the state list
	std::map<std::string, int> m_state2idx;

	// all allowed transitions, indexed at the source state
	std::vector< std::vector<Transition> > m_transitions;

    public:
 
	// construction / destruction
	FiniteStateMachine();
	~FiniteStateMachine();
	void reset();

	// queries
	bool hasState(std::string statename) const; 
	std::string getCurrentState() const;
	std::vector<State> const & getStates() const;

	// FSM setup
	void registerState(std::string statename);
	void registerTransition(std::string from, std::string to, boost::function<bool()> transitionCondition);
	void registerEntryAction(std::string state_name, boost::function<void(void)> action);
	void registerExitAction(std::string state_name, boost::function<void(void)> action);
   
	// FSM iteration
	void iterate();

// auxiliary stuff
    private:
	void destroy();
	
}; // end FiniteStateMachine


/* EXAMPLE


#include <boost/bind.hpp>
#include <boost/ref.hpp>
#include <boost/function.hpp>
#include "fsm.hpp"
using namespace std;

int main() 
{
	FiniteStateMachine testFSM;
	// some random state changes between A and B, with Final as sink
	testFSM.registerState("stateA");
	testFSM.registerState("stateB");
	testFSM.registerState("stateFinal");
	testFSM.registerTransition("stateA", "stateB", boost::bind(randBool, 0.5));
	testFSM.registerTransition("stateB", "stateA", boost::bind(randBool, 0.5));
	testFSM.registerTransition("stateB", "stateFinal", boost::bind(randBool, 0.1));
	for (int i = 0; i < 10; i++) 
	{
		testFSM.iterate();
	}
	return 0;
}
*/


#endif // _INCLUDED_FSM_HPP_

