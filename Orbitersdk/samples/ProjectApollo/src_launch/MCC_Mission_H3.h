/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2026

MCC for Mission H3 (Header)

Project Apollo is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

Project Apollo is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Project Apollo; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

See http://nassp.sourceforge.net/license/ for more details.

**************************************************************************/

#pragma once

// MISSION STATES: MISSION H3 (Apollo 14)
//
// Same ground-loop shape as Mission H1 (Apollo 12): liftoff initialization,
// TLI simulation, TLI pad, S-IVB evasive/TB8, then state-vector uplinks.
// State numbers are interpreted only while MissionType is MTP_H3.
//
// Not implemented here (need Apollo 14 flight-plan data, not the H1 constants):
// TLI+90 abort pad, P37 block data, PTC REFSMMAT epoch, MCC-1 through MCC-4,
// lunar flyby pad, LOI/DOI/PDI/ascent/TEI pads, map updates, landmark and
// photography pads, and the lunar-surface timeline. See MCC_Mission_H3.cpp.

//Ground liftoff time update to TLI simulation
#define MST_H3_INSERTION			10
//TLI simulation to the skipped earth-orbit abort pads
#define MST_H3_EPO1					11
//Skipped TLI+90 / block data, then TLI PAD
#define MST_H3_EPO2					12
//TLI PAD to TLI evaluation
#define MST_H3_EPO4					14
//TLI evaluation to S-IVB evasive maneuver
#define MST_H3_TRANSLUNAR1			20
#define MST_H3_TRANSLUNAR2			21
//S-IVB evasive maneuver to TB8 enable
#define MST_H3_TRANSLUNAR3			22
//TB8 enable to CSM state-vector uplink
#define MST_H3_TRANSLUNAR4			23
#define MST_H3_TRANSLUNAR5			24
//CSM state vector to PTC quad decision
#define MST_H3_TRANSLUNAR_SV		25
//PTC quad decision to the coast placeholder
#define MST_H3_TRANSLUNAR_QUADS		26
#define MST_H3_COAST_NOTE			27
//Repeating CSM state-vector uplinks for the rest of the flight
#define MST_H3_COAST				28

//ABORTS
//Earth-orbit abort. Ends at entry, same terminal condition as Mission H1.
#define MST_H3_ABORT_ORBIT			500
//Post-insertion abort. Entry-interface evaluation only; no H1 block-data schedule.
#define MST_H3_ABORT				501
