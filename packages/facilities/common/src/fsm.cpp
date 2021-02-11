// Copyright 2015-2018 Tim Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/**
 * File: fsm.cpp
 * Author: Jan Feitsma
 * Creation: 2014-12-27
 * Description: FiniteStateMachine implementations.
 *
 * 
 */

#include "ext/fsm.hpp"
#include "tracing.hpp"



State::State(int i, std::string s)
{
	id = i;
	name = s;
	entryAction = 0;
	exitAction = 0;
	TRACE("registered state %d with name %s", i, s.c_str());
}

Transition::Transition(int from, int to, boost::function<bool()> cond)
{
	fromState = from;
	toState = to;
	condition = cond;
	TRACE("registered statetransition between %d and %d", from, to);
}

FiniteStateMachine::FiniteStateMachine()
{ 
	TRACEF();
	reset();
}

FiniteStateMachine::~FiniteStateMachine()
{
	TRACEF();
	destroy();
}
   
void FiniteStateMachine::destroy()
{
	TRACEF();
	reset();
}
   
void FiniteStateMachine::reset()
{
	TRACEF();
	m_current_state = 0;
	m_states.clear();
	m_transitions.clear();
	m_state2idx.clear();
}
   
void FiniteStateMachine::registerState(std::string statename)
{
	TRACE("statename=%s", statename.c_str());
	if (hasState(statename))
	{
		TRACE("already exists");
		return;
	}
	int id = m_states.size();
	TRACE("assigning id=%d", id);
	m_states.push_back(State(id, statename));
	m_state2idx[statename] = id;
	// add empty transition list
	m_transitions.push_back(std::vector<Transition>());
}

bool FiniteStateMachine::hasState(std::string statename) const
{
	return m_state2idx.count(statename);
}

std::string FiniteStateMachine::getCurrentState() const
{
	return m_states[m_current_state].name;
}

std::vector<State> const & FiniteStateMachine::getStates() const
{
	return m_states;
}

void FiniteStateMachine::registerTransition(std::string from, std::string to, boost::function<bool()> transitionCondition)
{
	TRACEF("from=%s to=%s", from.c_str(), to.c_str());
	// register if not existing
	registerState(from);
	registerState(to);
	m_transitions[m_state2idx[from]].push_back(Transition(m_state2idx[from], m_state2idx[to], transitionCondition));
}

void FiniteStateMachine::registerEntryAction(std::string state_name, boost::function<void(void)> action)
{
	TRACEF("start state_name%s", state_name.c_str());
	registerState(state_name);
	m_states[m_state2idx[state_name]].entryAction = action;
}

void FiniteStateMachine::registerExitAction(std::string state_name, boost::function<void(void)> action)
{
	TRACEF("start state_name%s", state_name.c_str());
	registerState(state_name);
	m_states[m_state2idx[state_name]].exitAction = action;
}
   
void FiniteStateMachine::iterate()
{
	TRACEF("start");
	// check if FSM has been initialized, i.e. if states have been defined
	if (!m_states.size()) return;
	TRACE("current state %s (%d)", m_states[m_current_state].name.c_str(), m_current_state);
	// check conditions
	TRACE("checking %d conditions", (int)m_transitions[m_current_state].size());
	for (int i = 0; i < (int)m_transitions[m_current_state].size(); ++i)
	{
		if (m_transitions[m_current_state][i].condition())
		{
			int newState = m_transitions[m_current_state][i].toState;
			TRACE("transition i=%d: changing state to %s (%d)", i, m_states[newState].name.c_str(), newState);
			if (m_states[m_current_state].exitAction) m_states[m_current_state].exitAction();
			m_current_state = newState;
			if (m_states[m_current_state].entryAction) m_states[m_current_state].entryAction();
			break;
		}
	}
}



