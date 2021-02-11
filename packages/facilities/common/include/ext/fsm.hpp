// Copyright 2015 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
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

