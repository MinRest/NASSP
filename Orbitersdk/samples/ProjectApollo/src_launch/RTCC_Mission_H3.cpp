/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2026

RTCC Calculations for Mission H3 (Apollo 14)

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

#include "Orbitersdk.h"
#include "soundlib.h"
#include "apolloguidance.h"
#include "mcc.h"
#include "rtcc.h"

bool RTCC::CalculationMTP_H3(int fcn, LPVOID& pad, char* upString, char* upDesc, char* upMessage)
{
	switch (fcn)
	{
	// Updates that are computed from the live vehicle and the loaded mission
	// files (Apollo 14 Constants / TLI / SFP). They do not embed Apollo 12
	// flight-plan constants.
	//
	// TODO(A14): update 11 still builds the CSM/LM separation maneuver with the
	// H1 inertial attitude (48.6, -130.9, -139.1 deg). The H1 source already
	// notes that this attitude should be launch-day specific. TLI targeting
	// itself comes from the Apollo 14 TLI file via M68.
	case 1: //CMC CSM state vector
	case 2: //CMC LM state vector
	case 3: //LGC CSM state vector
	case 4: //LGC LM state vector
	case 5: //CMC CSM state vector with V66
	case 6: //LGC LM state vector with V66
	case 7: //CSM DAP data
	case 700: //CSM DAP data, docked
	case 8: //LM DAP data
	case 9: //LM DAP data with V42
	case 10: //Ground liftoff time update
	case 11: //TLI simulation
	case 14: //TLI PAD
	case 15: //TLI evaluation
	case 140: //PTC quad decision
	case 200: //TEI evaluation
	case 205: //EI evaluation
		return CalculationMTP_H1(fcn, pad, upString, upDesc, upMessage);
	default:
		// TODO(A14): the H1 calculator for this update number contains Apollo 12
		// flight-plan constants (abort and block-data TIGs, the PTC REFSMMAT MJD,
		// the 83:25:18 LOI target, Surveyor landing-site coordinates, TEI revolution
		// numbers, or photography targets). Do not reuse those values for Apollo 14.
		if (upMessage != NULL)
		{
			sprintf(upMessage, "A14 update %d needs flight-plan data", fcn);
		}
		return false;
	}
}
