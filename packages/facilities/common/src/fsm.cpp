 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /**
 * File: fsm.cpp
 * Author: Jan Feitsma
 * Creation: 2014-12-27
 * Description: FiniteStateMachine implementations.
 *
 * 
 */

#include "ext/fsm.hpp"
#include "ext/tracer.hpp"



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



