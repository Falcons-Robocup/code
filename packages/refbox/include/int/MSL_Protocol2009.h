// Copyright 2015-2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0

/********************************************************************************
 *  FILE: MSL-commands.h
 *
 * PURPOSE: This file contains the constants that define the various 
 *      commands the referee box can send to the clients
 *
 *  written by: Jurge van Eijck, contact: <jveijck@users.sourceforge.net>
 *  original written by: Brett Browning
 ********************************************************************************
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *******************************************************************************/

#ifndef __COMMANDS_H__
#define __COMMANDS_H__

//only for serial connection
/* Baud rate */
#define COMM_BAUD_RATE      9600

// play commands
#define COMM_STOP             'S'
#define COMM_START            's'

// game flow commands
#define COMM_FIRST_HALF       '1'
#define COMM_HALF_TIME        'h'
#define COMM_SECOND_HALF      '2'
#define COMM_END_GAME         'e'
#define COMM_OT_FIRST_HALF    '3'
#define COMM_OT_SECOND_HALF   '4'
#define COMM_GAMEOVER         'z'
#define COMM_REQWORLDSTATE    'w'
#define COMM_TESTMODEON       'V'
#define COMM_TESTMODEOFF      'v'
#define COMM_PARK             'L'

// goal status
#define COMM_GOAL_MAGENTA       'a'
#define COMM_GOAL_CYAN          'A'
#define COMM_SUBGOAL_MAGENTA    'd'
#define COMM_SUBGOAL_CYAN       'D'

/* game flow commands */
#define COMM_RESTART           'Z'

#define COMM_KICKOFF_MAGENTA   'k'
#define COMM_KICKOFF_CYAN      'K'

#define COMM_FREEKICK_MAGENTA  'f'
#define COMM_FREEKICK_CYAN     'F'

#define COMM_GOALKICK_MAGENTA  'g'
#define COMM_GOALKICK_CYAN     'G'

#define COMM_THROWIN_MAGENTA   't'
#define COMM_THROWIN_CYAN      'T'

#define COMM_CORNER_MAGENTA    'c'
#define COMM_CORNER_CYAN       'C'

#define COMM_PENALTY_MAGENTA   'p'
#define COMM_PENALTY_CYAN      'P'

#define COMM_DROPPED_BALL      'N'
#define COMM_DUMMY             '*'

#define COMM_REPAIR_OUT_CYAN   'O'
#define COMM_REPAIR_OUT_MAGENTA 'o'
#define COMM_REPAIR_IN_CYAN    'I'
#define COMM_REPAIR_IN_MAGENTA 'i'

#define COMM_REDCARD_CYAN      'R'
#define COMM_REDCARD_MAGENTA   'r'
#define COMM_YELLOWCARD_CYAN   'Y'
#define COMM_YELLOWCARD_MAGENTA 'y'
#define COMM_DOUBLE_YELLOW_IN_CYAN   'J'
#define COMM_DOUBLE_YELLOW_IN_MAGENTA 'j'
#define COMM_DOUBLE_YELLOW_OUT_CYAN   'B'
#define COMM_DOUBLE_YELLOW_OUT_MAGENTA 'b'

// acceptable referee commands
#define COMM_CMD_STRING      "SsNhezZWwVv1234LKkFfGgTtCcPpAaDdOoIiRrYyJjBb"

#endif /* __COMMANDS_H__ */
