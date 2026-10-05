/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2026

MCC sequencing for Mission H3 (Apollo 14)

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
#include "saturn.h"
#include "sivb.h"
#include "mcc.h"
#include "rtcc.h"
#include "MCC_Mission_H3.h"
#include "iu.h"
#include "nassputils.h"

using namespace nassp;

// TODO(A14): every GET below is copied from the Mission H1 (Apollo 12) sequence
// so the ground loop has the same shape. These are not Apollo 14 flight-plan times.
static const double H3_LIFTOFF_UPDATE_GET = 1.0 * 3600.0 + 30.0 * 60.0;
static const double H3_TLI_PAD_DELAY = 18.0;
static const double H3_POST_TLI_CALLOUT = 1.0 * 3600.0 + 20.0 * 60.0;
static const double H3_EVASIVE_YAW_DELAY = 3.5 * 60.0;
static const double H3_EVASIVE_MIN_AFTER_YAW = 8.0 * 60.0;
static const double H3_POST_EVASIVE_GET = 1.0 * 3600.0 + 31.0 * 60.0 + 40.0;
static const double H3_TB8_TO_SV = 2.0 * 3600.0 + 29.0 * 60.0;
static const double H3_PTC_QUADS_GET = 7.0 * 3600.0 + 15.0 * 60.0;
static const double H3_LATE_TRANSLUNAR_GET = 39.0 * 3600.0 + 1.0 * 60.0;
static const double H3_SECOND_TLI_OPP = 3.0 * 3600.0;
// Placeholder so CSM state vectors keep being offered after the coded sequence.
// Not an Apollo 14 update cycle. TODO(A14): replace with flight-plan SV passes.
static const double H3_COAST_SV_INTERVAL = 8.0 * 3600.0;

void MCC::MissionSequence_H3()
{
	switch (MissionState)
	{
	case MST_H3_INSERTION: //Ground liftoff time update to TLI Simulation
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(H3_LIFTOFF_UPDATE_GET), 10, MST_H3_EPO1);
		break;
	case MST_H3_EPO1: //TLI Simulation
		UpdateMacro(UTP_NONE, PT_NONE, true, 11, MST_H3_EPO2);
		break;
	case MST_H3_EPO2:
		// TODO(A14): H1 issues a TLI+90 maneuver pad (update 12, abort landing time
		// 16:46:00) and a liftoff+8 / block-data P37 pad (updates 13, 16, 17).
		// Those times are Apollo 12 data and are not uplinked here.
		if (SubState == 0)
		{
			addMessage("TLI+90 PAD skipped; A14 abort time is not coded.");
			addMessage("Block data skipped; A14 splash times are not coded.");
			setSubState(1);
		}
		else
		{
			setState(MST_H3_EPO4);
		}
		break;
	case MST_H3_EPO4: //TLI PAD to TLI Evaluation
		UpdateMacro(UTP_PADONLY, PT_TLIPAD, mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_TLI_PAD_DELAY), 14, MST_H3_TRANSLUNAR1);
		break;
	case MST_H3_TRANSLUNAR1: //TLI Evaluation to SIVB Evasive Maneuver
		UpdateMacro(UTP_NONE, PT_NONE, true, 15, MST_H3_TRANSLUNAR2, scrubbed, mcc_calcs.GETEval(H3_SECOND_TLI_OPP), MST_H3_EPO1);
		break;
	case MST_H3_TRANSLUNAR2:
		switch (SubState) {
		case 0:
		{
			addMessage("TLI");
			MissionPhase = MMST_TL_COAST;
			setSubState(1);
		}
		break;
		case 1:
		{
			// TODO(A14): H1 template, TLI + 1:20.
			if (mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_POST_TLI_CALLOUT))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR3);
			}
		}
		break;
		}
		break;
	case MST_H3_TRANSLUNAR3: //SIVB Evasive Maneuver to TB8 Enable
		switch (SubState) {
		case 0:
		{
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
		}
		break;
		case 1:
		{
			if (sivb == NULL)
			{
				VESSEL* v;
				OBJHANDLE hLV;
				hLV = oapiGetVesselByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);

					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB*)v;
					}
				}
			}

			if (sivb == NULL)
			{
				addMessage("S-IVB not found; skipping evasive uplink.");
				setState(MST_H3_TRANSLUNAR4);
			}
			else if (sivb->DockingStatus(0) == 0)
			{
				setSubState(2);
			}
		}
		break;
		case 2:
		{
			// TODO(A14): H1 template, 3.5 minutes after LM ejection.
			if (SubStateTime > H3_EVASIVE_YAW_DELAY)
			{
				sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_EVASIVE_MANEUVER_ENABLE, NULL);
				setSubState(3);
			}
		}
		break;
		case 3:
			// TODO(A14): H1 template, not before 8 minutes after the yaw command and not before TLI + 1:31:40.
			if (SubStateTime >= H3_EVASIVE_MIN_AFTER_YAW && mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_POST_EVASIVE_GET))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR4);
			}
			break;
		}
		break;
	case MST_H3_TRANSLUNAR4: //TB8 enable
		switch (SubState) {
		case 0:
		{
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
		}
		break;
		case 1:
		{
			if (sivb == NULL)
			{
				VESSEL* v;
				OBJHANDLE hLV;
				hLV = oapiGetVesselByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);

					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB*)v;
					}
				}
			}

			if (sivb == NULL)
			{
				addMessage("S-IVB not found; skipping TB8 uplink.");
				setState(MST_H3_TRANSLUNAR5);
			}
			else
			{
				sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_TIMEBASE_8_ENABLE, NULL);
				setSubState(2);
			}
		}
		break;
		case 2:
			// TODO(A14): H1 template, TLI + 2:29. H1 uplinks the PTC REFSMMAT here.
			// That update uses an Apollo 12 MJD, so H3 uplinks a CSM state vector instead.
			if (mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_TB8_TO_SV))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR5);
			}
			break;
		}
		break;
	case MST_H3_TRANSLUNAR5:
		if (SubState == 0)
		{
			addMessage("PTC REFSMMAT skipped; A14 time is not coded.");
			setSubState(1);
		}
		else
		{
			setState(MST_H3_TRANSLUNAR_SV);
		}
		break;
	case MST_H3_TRANSLUNAR_SV: //CSM state vector, then PTC quad decision
		// TODO(A14): advance time is the H1 PTC-quad GET (TLI + 7:15).
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_PTC_QUADS_GET), 5, MST_H3_TRANSLUNAR_QUADS);
		break;
	case MST_H3_TRANSLUNAR_QUADS: //PTC quads from live RCS quantities
		// TODO(A14): hold until the H1 late-translunar GET (TLI + 39:01).
		// H1 runs MCC-1/2, block data 2, and the lunar flyby pad in this gap.
		UpdateMacro(UTP_PADONLY, PT_GENERIC, mcc_calcs.GETEval(rtcc->calcParams.TLI + H3_LATE_TRANSLUNAR_GET), 140, MST_H3_COAST_NOTE);
		break;
	case MST_H3_COAST_NOTE:
		if (SubState == 0)
		{
			addMessage("A14 flight-plan pads are not coded. State vectors continue.");
			setSubState(1);
		}
		else
		{
			setState(MST_H3_COAST);
		}
		break;
	case MST_H3_COAST:
		// Generic CSM state vector (update 5). Repeats after H3_COAST_SV_INTERVAL
		// once the previous uplink is finished. TODO(A14): flight-plan SV passes.
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > H3_COAST_SV_INTERVAL, 5, MST_H3_COAST);
		break;
	case MST_H3_ABORT_ORBIT:
	{
		if (AbortMode == 5)
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
	}
	break;
	case MST_H3_ABORT:
		// TODO(A14): Mission H1 steps through fixed abort TIGs (TLI+90, 8:00, 15:00,
		// 25:00, 35:00, 45:00, 60:00). Those are Apollo 12 block-data times.
		// H3 only evaluates entry interface from the current state vector.
		switch (SubState) {
		case 0:
		{
			addMessage("A14 abort pads are not coded. Evaluating EI.");
			startSubthread(205, UTP_NONE);
			setSubState(1);
		}
		break;
		case 1:
			if (subThreadStatus == DONE)
			{
				addMessage("EI evaluation complete. Abort PAD is not coded.");
				setSubState(2);
			}
			break;
		case 2:
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
			break;
		}
		break;
	}
}
