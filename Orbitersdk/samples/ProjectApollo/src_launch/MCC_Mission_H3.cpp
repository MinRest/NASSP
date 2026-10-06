/****************************************************************************
This file is part of Project Apollo - NASSP
Copyright 2026

MCC sequencing for Mission H3

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

// Apollo 14 (Mission H3) ground loop.
//
// Same state machine shape as Mission H1. An update runs when its state is
// entered; the GET on that line is when the state hands off to the next update.
// Times are the 18 January 1971 final flight plan (HSI-209261) and the Apollo 14
// press kit. Separation and deorbit axes are the flight-plan notes and the
// October 1970 LM-impact note. A few minutes between an evaluation and its pad
// is the same processing gap the H1 script uses, not a second flight-plan time.
//
// Table I-5: TLI 02:30:38, MCC-1 11:36:33, MCC-2 30:36:07, MCC-3 60:38:14,
// MCC-4 77:38:14, LOI 82:38:14, DOI 86:56:57, undock/sep 104:27:31,
// circ 105:46:48, PC-1 118:09:40, CSM sep 146:28:31, TEI 149:14:50,
// MCC-5 166:14:50, MCC-6 194:26:59, MCC-7 213:26:59.
// Table I-6: PDI 108:42:01, ascent 142:24:29, LM deorbit 147:52:58.9.
// Table I-7 pass times gate the block-data and abort pads.
// Press kit: S-IVB evasive 04:19, propulsive dump 04:42, GET sync about 55 h.
// Table I-3 fuel-cell times, Table I-7 pass times, and Table I-11 P23 times gate the
// translunar coast. There is no repeating state-vector cycle in the flight plan.

static double A14GET(int h, int m, int s)
{
	return OrbMech::HHMMSSToSS(h, m, s);
}

void MCC::MissionSequence_H3()
{
	switch (MissionState)
	{
	case MST_H3_INSERTION: //Liftoff initialization, then TLI sim at the 1:40 block-data pass
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(A14GET(1, 40, 0)), 10, MST_H3_EPO1);
		break;
	case MST_H3_EPO1: //TLI simulation from the Apollo 14 TLI file. Sep attitude is not published.
		UpdateMacro(UTP_NONE, PT_NONE, true, 11, MST_H3_EPO2);
		break;
	case MST_H3_EPO2: //TLI+90, Table I-7 passed at 1:40
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 12, MST_H3_EPO3);
		break;
	case MST_H3_EPO3: //L/O+8, Table I-7 passed at 1:40
		UpdateMacro(UTP_PADONLY, PT_P37PAD, SubStateTime > 3.0 * 60.0, 13, MST_H3_EPO4);
		break;
	case MST_H3_EPO4: //Pre-burn TLI pad, held until burnout + 18 s. SEP and extraction FDAI are on the pad.
		UpdateMacro(UTP_PADONLY, PT_TLIPAD, mcc_calcs.GETEval(rtcc->calcParams.TLI + 18.0), 14, MST_H3_TRANSLUNAR_DAY1_1);
		break;
	case MST_H3_TRANSLUNAR_DAY1_1: //TLI evaluation. Second opportunity is the in-tree TLI file, GET 3:00
		UpdateMacro(UTP_NONE, PT_NONE, true, 15, MST_H3_TRANSLUNAR_DAY1_2, scrubbed, mcc_calcs.GETEval(3.0 * 3600.0), MST_H3_EPO1);
		break;
	case MST_H3_TRANSLUNAR_DAY1_2: //Coast to the press-kit evasive time
		switch (SubState) {
		case 0:
			addMessage("TLI");
			MissionPhase = MMST_TL_COAST;
			setSubState(1);
			break;
		case 1:
			if (mcc_calcs.GETEval(A14GET(4, 19, 0)))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR_DAY1_3);
			}
			break;
		}
		break;
	case MST_H3_TRANSLUNAR_DAY1_3: //S-IVB evasive at 04:19, after LM ejection
		switch (SubState) {
		case 0:
			if (cm->GetStage() >= CSM_LEM_STAGE)
			{
				setSubState(1);
			}
			else if (mcc_calcs.GETEval(A14GET(4, 42, 0)))
			{
				setState(MST_H3_TRANSLUNAR_DAY1_4);
			}
			break;
		case 1:
			if (sivb == NULL)
			{
				VESSEL *v;
				OBJHANDLE hLV = oapiGetVesselByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);
					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB *)v;
					}
				}
			}
			if (sivb == NULL || sivb->DockingStatus(0) != 0)
			{
				if (mcc_calcs.GETEval(A14GET(4, 42, 0)))
				{
					setState(MST_H3_TRANSLUNAR_DAY1_4);
				}
			}
			else
			{
				setSubState(2);
			}
			break;
		case 2:
			if (sivb != NULL && mcc_calcs.GETEval(A14GET(4, 19, 0)))
			{
				sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_EVASIVE_MANEUVER_ENABLE, NULL);
				setSubState(3);
			}
			else if (mcc_calcs.GETEval(A14GET(4, 42, 0)))
			{
				setState(MST_H3_TRANSLUNAR_DAY1_4);
			}
			break;
		case 3:
			if (mcc_calcs.GETEval(A14GET(4, 42, 0)))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR_DAY1_4);
			}
			break;
		}
		break;
	case MST_H3_TRANSLUNAR_DAY1_4: //TB8 / propulsive dump at 04:42, then L/O+15 at 06:00
		switch (SubState) {
		case 0:
			if (sivb == NULL)
			{
				VESSEL *v;
				OBJHANDLE hLV = oapiGetVesselByName(LVName);
				if (hLV != NULL)
				{
					v = oapiGetVesselInterface(hLV);
					if (utils::IsVessel(v, utils::SaturnV_SIVB)) {
						sivb = (SIVB *)v;
					}
				}
			}
			if (sivb != NULL)
			{
				sivb->GetIU()->GetDCS()->Uplink(DCSUPLINK_TIMEBASE_8_ENABLE, NULL);
			}
			setSubState(1);
			break;
		case 1:
			if (mcc_calcs.GETEval(A14GET(6, 0, 0)))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSLUNAR_DAY1_5);
			}
			break;
		}
		break;
	case MST_H3_TRANSLUNAR_DAY1_5: //L/O+15 passed at 06:00. PTC REFSMMAT at the 09:30 P23
		UpdateMacro(UTP_PADONLY, PT_P37PAD, mcc_calcs.GETEval(A14GET(9, 30, 0)), 16, MST_H3_TRANSLUNAR_DAY1_6);
		break;
	case MST_H3_TRANSLUNAR_DAY1_6: //PTC REFSMMAT at the 09:30 P23. Epoch is 166:10:30, not TEI.
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(A14GET(11, 30, 0)), 18, MST_H3_TRANSLUNAR_DAY1_7);
		break;
	case MST_H3_TRANSLUNAR_DAY1_7: //MCC-1 decision at the Table I-3 11:30 purge, before TIG 11:36:33
		UpdateMacro(UTP_NONE, PT_NONE, true, 19, MST_H3_TRANSLUNAR_DAY1_8);
		break;
	case MST_H3_TRANSLUNAR_DAY1_8: //MCC-1 pad, then block data 2 at 14:00
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(A14GET(14, 0, 0)), 21, MST_H3_TRANSLUNAR_DAY1_9);
		break;
	case MST_H3_TRANSLUNAR_DAY1_9: //L/O+25 through L/O+60
		UpdateMacro(UTP_PADONLY, PT_P37PAD, SubStateTime > 5.0 * 60.0, 17, MST_H3_TRANSLUNAR_DAY1_10);
		break;
	case MST_H3_TRANSLUNAR_DAY1_10: //PTC quads, then MCC-2 evaluation at the 28:30 P23
		UpdateMacro(UTP_PADONLY, PT_GENERIC, mcc_calcs.GETEval(A14GET(28, 30, 0)), 140, MST_H3_TRANSLUNAR_DAY1_11);
		break;
	case MST_H3_TRANSLUNAR_DAY1_11:
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 5.0 * 60.0, 20, MST_H3_TRANSLUNAR_DAY1_12);
		break;
	case MST_H3_TRANSLUNAR_DAY1_12: //MCC-2 pad. Flyby data is passed at 35:00
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(A14GET(35, 0, 0)), 22, MST_H3_TRANSLUNAR_DAY2_1);
		break;
	case MST_H3_TRANSLUNAR_DAY2_1: //Flyby pad, then the 55 h GET-sync window
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, mcc_calcs.GETEval(A14GET(55, 0, 0)), 23, MST_H3_TRANSLUNAR_DAY2_2);
		break;
	case MST_H3_TRANSLUNAR_DAY2_2: //GET sync note from 55:00 until the MCC-3 pass at 60:20
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(A14GET(60, 20, 0)), 501, MST_H3_TRANSLUNAR_DAY3_1);
		break;
	case MST_H3_TRANSLUNAR_DAY2_3: //Saved state. No separate coast SV; MCC-3 carries its own.
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(A14GET(60, 20, 0)), 501, MST_H3_TRANSLUNAR_DAY3_1);
		break;
	case MST_H3_TRANSLUNAR_DAY2_4:
		if (mcc_calcs.GETEval(A14GET(60, 20, 0)))
		{
			setState(MST_H3_TRANSLUNAR_DAY3_1);
		}
		break;
	case MST_H3_TRANSLUNAR_DAY3_1: //MCC-3. A scrub still reaches the 76:00 PC+2 pass
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 24, MST_H3_TRANSLUNAR_DAY3_2, scrubbed, mcc_calcs.GETEval(A14GET(76, 0, 0)), MST_H3_TRANSLUNAR_DAY4_1);
		break;
	case MST_H3_TRANSLUNAR_DAY3_2:
		UpdateMacro(UTP_PADONLY, PT_GENERIC, mcc_calcs.GETEval(A14GET(76, 0, 0)), 140, MST_H3_TRANSLUNAR_DAY4_1);
		break;
	case MST_H3_TRANSLUNAR_DAY4_1: //MCC-4 decision at the 76:00 PC+2 pass
		UpdateMacro(UTP_NONE, PT_NONE, true, 25, MST_H3_TRANSLUNAR_DAY4_2, scrubbed, true, MST_H3_TRANSLUNAR_NO_MCC4_2);
		break;
	case MST_H3_TRANSLUNAR_NO_MCC4_1: //Saved state. PC+2 does not wait on an extra SV.
		if (mcc_calcs.GETEval(A14GET(76, 0, 0)))
		{
			setState(MST_H3_TRANSLUNAR_NO_MCC4_2);
		}
		break;
	case MST_H3_TRANSLUNAR_NO_MCC4_2: //PC+2 without MCC-4, passed at 76:00, held to the 79:30 TEI-4
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, mcc_calcs.GETEval(A14GET(79, 30, 0)), 28, MST_H3_TRANSLUNAR_NO_MCC4_3);
		break;
	case MST_H3_TRANSLUNAR_NO_MCC4_3:
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 29, MST_H3_TRANSLUNAR_DAY4_5);
		break;
	case MST_H3_TRANSLUNAR_DAY4_2: //MCC-4 pad, then PC+2 in the same 76:00 pass
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, true, 26, MST_H3_TRANSLUNAR_DAY4_3);
		break;
	case MST_H3_TRANSLUNAR_DAY4_3:
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, mcc_calcs.GETEval(A14GET(79, 30, 0)), 27, MST_H3_TRANSLUNAR_DAY4_4);
		break;
	case MST_H3_TRANSLUNAR_DAY4_4: //Single LOI pad. No second final-update time was in the tables
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 30, MST_H3_TRANSLUNAR_DAY4_5);
		break;
	case MST_H3_TRANSLUNAR_DAY4_5: //TEI-4, passed at 79:30
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 41, MST_H3_TRANSLUNAR_DAY4_6);
		break;
	case MST_H3_TRANSLUNAR_DAY4_6: //Rev 1 map, held through the LOI burn
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, mcc_calcs.GETEval(rtcc->TimeofIgnition + 7.0 * 60.0), 60, MST_H3_TRANSLUNAR_DAY4_8);
		break;
	case MST_H3_TRANSLUNAR_DAY4_7:
		setState(MST_H3_TRANSLUNAR_DAY4_8);
		break;
	case MST_H3_TRANSLUNAR_DAY4_8: //LOI evaluation after the solved ignition
		UpdateMacro(UTP_NONE, PT_NONE, true, 33, MST_H3_LUNAR_ORBIT_LOI_DAY_1, scrubbed, true, MST_H3_ABORT);
		break;
	case MST_H3_TRANSLUNAR_DAY4_9:
		setState(MST_H3_LUNAR_ORBIT_LOI_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_1:
		switch (SubState) {
		case 0:
			MissionPhase = MMST_LUNAR_ORBIT;
			setSubState(1);
			break;
		case 1:
			// DOI pad is due at 84:50 even if the revolution count has not stepped.
			if ((MoonRev >= 2 && MoonRevTime > 10.0 * 60.0) || mcc_calcs.GETEval(A14GET(84, 50, 0)))
			{
				SlowIfDesired();
				setState(MST_H3_LUNAR_ORBIT_LOI_DAY_2);
			}
			break;
		}
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_2:
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, mcc_calcs.GETEval(A14GET(84, 50, 0)), 600, MST_H3_LUNAR_ORBIT_LOI_DAY_3);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_3: //CSM SPS DOI, Table I-5 86:56:57
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(A14GET(85, 5, 0)), 31, MST_H3_LUNAR_ORBIT_LOI_DAY_4);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_4: //TEI-5 passed at 85:05, assumes DOI
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 42, MST_H3_LUNAR_ORBIT_LOI_DAY_5);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_5: //Mosting A, rev 2
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, MoonRev >= 3 || mcc_calcs.GETEval(A14GET(89, 20, 0)), 61, MST_H3_LUNAR_ORBIT_LOI_DAY_6);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_6: //H-3, rev 3
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, mcc_calcs.GETEval(A14GET(89, 20, 0)), 62, MST_H3_LUNAR_ORBIT_LOI_DAY_7);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_7: //TEI-12 passed at 89:20
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 43, MST_H3_LUNAR_ORBIT_LOI_DAY_8);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_8:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(A14GET(100, 45, 0)), 5, MST_H3_LUNAR_ORBIT_LOI_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_9: //TEI-19 passed at 100:45
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, mcc_calcs.GETEval(A14GET(103, 0, 0)), 44, MST_H3_LUNAR_ORBIT_LOI_DAY_10);
		break;
	case MST_H3_LUNAR_ORBIT_LOI_DAY_10:
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, true, 600, MST_H3_LUNAR_ORBIT_PDI_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_1: //14-1 through 14-4, revs 12/13/15
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, mcc_calcs.GETEval(A14GET(104, 20, 0)), 67, MST_H3_LUNAR_ORBIT_PDI_DAY_2);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_2: //Undock/sep 104:27:31, radial, CSM below
		UpdateMacro(UTP_PADONLY, PT_AP12SEPPAD, mcc_calcs.GETEval(A14GET(105, 40, 0)), 37, MST_H3_LUNAR_ORBIT_PDI_DAY_3);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_3: //Circ 105:46:48, perilune 56.04 nm
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 3.0 * 60.0, 502, MST_H3_LUNAR_ORBIT_PDI_DAY_4);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_4:
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, mcc_calcs.GETEval(A14GET(107, 0, 0)), 7, MST_H3_LUNAR_ORBIT_PDI_DAY_5);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_5: //Landing-site REFSMMAT before PDI 108:42:01
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0 * 60.0, 32, MST_H3_LUNAR_ORBIT_PDI_DAY_6);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_6:
		UpdateMacro(UTP_PADONLY, PT_LMACTDATA, SubStateTime > 3.0 * 60.0, 9, MST_H3_LUNAR_ORBIT_PDI_DAY_7);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_7:
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, SubStateTime > 5.0 * 60.0, 35, MST_H3_LUNAR_ORBIT_PDI_DAY_8);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_8:
		UpdateMacro(UTP_PADONLY, PT_AP11AGSACT, SubStateTime > 5.0 * 60.0, 36, MST_H3_LUNAR_ORBIT_PDI_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_9:
		UpdateMacro(UTP_PADONLY, PT_AP11PDIPAD, SubStateTime > 3.0 * 60.0, 70, MST_H3_LUNAR_ORBIT_PDI_DAY_10);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_10:
		UpdateMacro(UTP_PADONLY, PT_AP12PDIABORTPAD, SubStateTime > 3.0 * 60.0, 71, MST_H3_LUNAR_ORBIT_PDI_DAY_11);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_11:
		UpdateMacro(UTP_PADONLY, PT_AP12LUNSURFPAD, SubStateTime > 2.0 * 60.0, 73, MST_H3_LUNAR_ORBIT_PRE_PDI_2);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_12:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_PDI_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_PDI_DAY_13:
	case MST_H3_LUNAR_ORBIT_PDI_DAY_14:
	case MST_H3_LUNAR_ORBIT_PDI_DAY_15:
	case MST_H3_LUNAR_ORBIT_PDI_DAY_16:
	case MST_H3_LUNAR_ORBIT_PDI_DAY_17:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_PDI_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_PRE_DOI_1:
	case MST_H3_LUNAR_ORBIT_PRE_DOI_2:
	case MST_H3_LUNAR_ORBIT_PRE_DOI_3:
	case MST_H3_LUNAR_ORBIT_PRE_DOI_4:
	case MST_H3_LUNAR_ORBIT_PRE_PDI_1:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_PRE_PDI_2);
		break;
	case MST_H3_LUNAR_ORBIT_PRE_PDI_2:
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(A14GET(108, 44, 0)), 77, MST_H3_LUNAR_ORBIT_PRE_PDI_3);
		break;
	case MST_H3_LUNAR_ORBIT_PRE_PDI_3: //LM state vector and RLS during PDI
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_GENERIC, SubStateTime > 3.0 * 60.0, 75, MST_H3_LUNAR_ORBIT_PRE_LANDING_1);
		break;
	case MST_H3_LUNAR_ORBIT_PRE_LANDING_1:
		UpdateMacro(UTP_NONE, PT_NONE, rtcc->calcParams.tgt->GroundContact(), 78, MST_H3_LUNAR_ORBIT_POST_LANDING_1, scrubbed, SubStateTime > 3.0 * 60.0, MST_H3_LUNAR_ORBIT_NO_PDI);
		break;
	case MST_H3_LUNAR_ORBIT_NO_PDI:
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_AP11AGSACT, SubStateTime > 3.0 * 60.0, 170, MST_H3_LUNAR_ORBIT_PDI_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_1:
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 3.0 * 60.0, 79, MST_H3_LUNAR_ORBIT_POST_LANDING_2);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_2:
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 3.0 * 60.0, 80, MST_H3_LUNAR_ORBIT_POST_LANDING_3);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_3:
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 3.0 * 60.0, 81, MST_H3_LUNAR_ORBIT_POST_LANDING_4);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_4:
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_LMP22ACQPAD, MoonRev >= 15 || mcc_calcs.GETEval(A14GET(115, 0, 0)), 74, MST_H3_LUNAR_ORBIT_POST_LANDING_5);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_5: //Rev 15 landmarks
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, MoonRev >= 17 || mcc_calcs.GETEval(A14GET(115, 0, 0)), 68, MST_H3_LUNAR_ORBIT_POST_LANDING_6);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_6: //Landing site, rev 17
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0 * 60.0, 63, MST_H3_LUNAR_ORBIT_POST_LANDING_7);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_7:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 3.0 * 60.0, 1, MST_H3_LUNAR_ORBIT_POST_LANDING_8);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_8:
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, MoonRev >= 18 || mcc_calcs.GETEval(A14GET(115, 0, 0)), 7, MST_H3_LUNAR_ORBIT_POST_LANDING_9);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_9:
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, SubStateTime > 3.0 * 60.0, 85, MST_H3_LUNAR_ORBIT_POST_LANDING_10);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_10:
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0 * 60.0, 66, MST_H3_LUNAR_ORBIT_POST_LANDING_11);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_11:
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, SubStateTime > 3.0 * 60.0, 600, MST_H3_LUNAR_ORBIT_POST_LANDING_12);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_12: //Rev 18 landmark set
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, mcc_calcs.GETEval(A14GET(115, 0, 0)), 64, MST_H3_LUNAR_ORBIT_POST_LANDING_13);
		break;
	case MST_H3_LUNAR_ORBIT_POST_LANDING_13: //TEI-34 preliminary, passed at 115:00
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 45, MST_H3_LUNAR_ORBIT_PLANE_CHANGE_1);
		break;
	case MST_H3_LUNAR_ORBIT_PLANE_CHANGE_1: //PC-1. Scrub after the planned 118:09:40 TIG
		UpdateMacro(UTP_NONE, PT_NONE, true, 93, MST_H3_LUNAR_ORBIT_PLANE_CHANGE_2, scrubbed, mcc_calcs.GETEval(A14GET(119, 0, 0)), MST_H3_LUNAR_ORBIT_NO_PLANE_CHANGE_1);
		break;
	case MST_H3_LUNAR_ORBIT_PLANE_CHANGE_2:
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(rtcc->TimeofIgnition + 5.0 * 60.0), 94, MST_H3_LUNAR_ORBIT_EVA_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_NO_PLANE_CHANGE_1:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0 * 60.0, 1, MST_H3_LUNAR_ORBIT_EVA_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_1: //Table I-5 has PC-1 only
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 1.0 * 60.0, 95, MST_H3_LUNAR_ORBIT_EVA_DAY_2);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_2:
		UpdateMacro(UTP_PADONLY, PT_LIFTOFFTIMES, SubStateTime > 3.0 * 60.0, 86, MST_H3_LUNAR_ORBIT_EVA_DAY_3);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_3: //Liftoff REFSMMAT after PC-1
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0 * 60.0, 96, MST_H3_LUNAR_ORBIT_EVA_DAY_4);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_4:
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, MoonRev >= 29 || mcc_calcs.GETEval(rtcc->calcParams.LunarLiftoff - 90.0 * 60.0), 600, MST_H3_LUNAR_ORBIT_EVA_DAY_5);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_5: //Rev 29 landmarks, then ascent targeting 90 min before liftoff
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, mcc_calcs.GETEval(rtcc->calcParams.LunarLiftoff - 90.0 * 60.0), 65, MST_H3_LUNAR_ORBIT_ASCENT_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_EVA_DAY_6:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_7:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_8:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_9:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_10:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_11:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_12:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_13:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_14:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_15:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_16:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_17:
	case MST_H3_LUNAR_ORBIT_EVA_DAY_18:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_ASCENT_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_1:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, SubStateTime > 5.0 * 60.0, 100, MST_H3_LUNAR_ORBIT_ASCENT_DAY_2);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_2: //Table I-6 has no H/V split, so the ascent pad is not filled with Apollo 12 components
		UpdateMacro(UTP_PADONLY, PT_AP12LMASCPAD, mcc_calcs.GETEval(rtcc->calcParams.LunarLiftoff - 45.0 * 60.0), 105, MST_H3_LUNAR_ORBIT_ASCENT_DAY_3);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_3: //Direct rendezvous: Table I-6 has TPI, not a CSI
		UpdateMacro(UTP_PADONLY, PT_AP10CSI, SubStateTime > 3.0 * 60.0, 106, MST_H3_LUNAR_ORBIT_ASCENT_DAY_4);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_4:
		UpdateMacro(UTP_PADONLY, PT_AP11LMARKTRKPAD, SubStateTime > 3.0 * 60.0, 66, MST_H3_LUNAR_ORBIT_ASCENT_DAY_5);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_5:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(rtcc->calcParams.LunarLiftoff - 10.0 * 60.0), 101, MST_H3_LUNAR_ORBIT_ASCENT_DAY_6);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_6:
		UpdateMacro(UTP_LGCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(rtcc->calcParams.LunarLiftoff + 20.0), 102, MST_H3_LUNAR_ORBIT_ASCENT_DAY_7);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_7:
		UpdateMacro(UTP_NONE, PT_NONE, mcc_calcs.GETEval(rtcc->calcParams.Insertion + 120.0), 107, MST_H3_LUNAR_ORBIT_ASCENT_DAY_8, scrubbed, SubStateTime > 15.0 * 60.0, MST_H3_LUNAR_ORBIT_ASCENT_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_8:
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, rtcc->calcParams.src->DockingStatus(0) == 1, 2, MST_H3_LUNAR_ORBIT_ASCENT_DAY_9);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_9:
		UpdateMacro(UTP_PADONLY, PT_AP10DAPDATA, SubStateTime > 3.0 * 60.0, 700, MST_H3_LUNAR_ORBIT_ASCENT_DAY_10);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_10:
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, mcc_calcs.GETEval(A14GET(146, 20, 0)), 600, MST_H3_LUNAR_ORBIT_ASCENT_DAY_11);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_11: //CSM sep 146:28:31, 1 fps retrograde, +Z thrusters
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(A14GET(147, 40, 0)), 110, MST_H3_LUNAR_ORBIT_PC2_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_12:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_13:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_14:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_15:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_16:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_17:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_18:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_19:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_20:
	case MST_H3_LUNAR_ORBIT_ASCENT_DAY_21:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_PC2_DAY_1);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_1: //TEI-34 nominal, passed at 147:40, to EOM
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 50, MST_H3_LUNAR_ORBIT_PC2_DAY_2);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_2: //TEI-35
		UpdateMacro(UTP_PADONLY, PT_AP11MNV, mcc_calcs.GETEval(A14GET(147, 50, 0)), 51, MST_H3_LUNAR_ORBIT_PC2_DAY_3);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_3: //LM deorbit 147:52:58.9, 180 fps retrograde, 36.5 fps north
		UpdateMacro(UTP_PADWITHLGCUPLINK, PT_AP11LMMNV, SubStateTime > 3.0 * 60.0, 111, MST_H3_LUNAR_ORBIT_PC2_DAY_4);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_4:
		UpdateMacro(UTP_PADONLY, PT_AP11P76PAD, mcc_calcs.GETEval(A14GET(149, 10, 0)), 112, MST_H3_LUNAR_ORBIT_PC2_DAY_27);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_5:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_6:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_7:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_8:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_9:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_10:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_11:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_12:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_13:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_14:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_15:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_16:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_17:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_18:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_19:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_20:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_21:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_22:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_23:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_24:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_25:
	case MST_H3_LUNAR_ORBIT_PC2_DAY_26:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_LUNAR_ORBIT_PC2_DAY_27);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_27: //Map before TEI 149:14:50
		UpdateMacro(UTP_PADONLY, PT_AP10MAPUPDATE, mcc_calcs.GETEval(rtcc->calcParams.TEI + 5.0 * 60.0), 601, MST_H3_LUNAR_ORBIT_PC2_DAY_28);
		break;
	case MST_H3_LUNAR_ORBIT_PC2_DAY_28:
		UpdateMacro(UTP_NONE, PT_NONE, true, 200, MST_H3_TRANSEARTH_DAY1_1, scrubbed, mcc_calcs.GETEval(rtcc->calcParams.TEI + 30.0 * 60.0), MST_H3_LUNAR_ORBIT_PC2_DAY_1);
		break;
	case MST_H3_TRANSEARTH_DAY1_1:
		switch (SubState)
		{
		case 0:
			MissionPhase = MMST_TE_COAST;
			setSubState(1);
			break;
		case 1:
			// Processing gap after TEI. The flight plan does not list a PTC uplink minute.
			if (mcc_calcs.GETEval(rtcc->calcParams.TEI + 20.0 * 60.0))
			{
				SlowIfDesired();
				setState(MST_H3_TRANSEARTH_DAY1_2);
			}
			break;
		}
		break;
	case MST_H3_TRANSEARTH_DAY1_2: //PTC REFSMMAT, then MCC-5 pad at the 164:00 P23
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(A14GET(164, 0, 0)), 18, MST_H3_TRANSEARTH_DAY1_3);
		break;
	case MST_H3_TRANSEARTH_DAY1_3:
		UpdateMacro(UTP_PADONLY, PT_GENERIC, SubStateTime > 5.0 * 60.0, 140, MST_H3_TRANSEARTH_DAY2_1);
		break;
	case MST_H3_TRANSEARTH_DAY2_1: //MCC-5. TIG 166:14:50
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, mcc_calcs.GETEval(A14GET(173, 0, 0)), 210, MST_H3_TRANSEARTH_DAY2_2);
		break;
	case MST_H3_TRANSEARTH_DAY2_2: //P23 at 173:00
		UpdateMacro(UTP_PADONLY, PT_GENERIC, mcc_calcs.GETEval(A14GET(188, 27, 0)), 140, MST_H3_TRANSEARTH_DAY2_3);
		break;
	case MST_H3_TRANSEARTH_DAY2_3: //P23 at 188:27
		UpdateMacro(UTP_CMCUPLINKONLY, PT_NONE, mcc_calcs.GETEval(A14GET(193, 10, 0)), 5, MST_H3_TRANSEARTH_DAY3_1);
		break;
	case MST_H3_TRANSEARTH_DAY3_1: //Fuel-cell time if MCC-6 is not performed
		UpdateMacro(UTP_PADONLY, PT_GENERIC, mcc_calcs.GETEval(A14GET(194, 20, 0)), 140, MST_H3_TRANSEARTH_DAY3_2);
		break;
	case MST_H3_TRANSEARTH_DAY3_2: //MCC-6 a few minutes before 194:26:59
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 212, MST_H3_TRANSEARTH_DAY3_3);
		break;
	case MST_H3_TRANSEARTH_DAY3_3:
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, mcc_calcs.GETEval(A14GET(213, 20, 0)), 216, MST_H3_TRANSEARTH_DAY4_2);
		break;
	case MST_H3_TRANSEARTH_DAY3_4:
	case MST_H3_TRANSEARTH_DAY3_5:
	case MST_H3_TRANSEARTH_DAY3_6:
	case MST_H3_TRANSEARTH_DAY4_1:
		UpdateMacro(UTP_NONE, PT_NONE, true, 500, MST_H3_TRANSEARTH_DAY4_2);
		break;
	case MST_H3_TRANSEARTH_DAY4_2: //MCC-7 decision, a few minutes before 213:26:59
		UpdateMacro(UTP_NONE, PT_NONE, SubStateTime > 60.0, 213, MST_H3_TRANSEARTH_DAY4_3);
		break;
	case MST_H3_TRANSEARTH_DAY4_3:
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11MNV, SubStateTime > 5.0 * 60.0, 214, MST_H3_TRANSEARTH_DAY4_4);
		break;
	case MST_H3_TRANSEARTH_DAY4_4:
		UpdateMacro(UTP_PADONLY, PT_AP11ENT, SubStateTime > 5.0 * 60.0, 217, MST_H3_TRANSEARTH_DAY4_5);
		break;
	case MST_H3_TRANSEARTH_DAY4_5: //Final entry update through CM/SM separation
		UpdateMacro(UTP_PADWITHCMCUPLINK, PT_AP11ENT, cm->GetStage() == CM_STAGE, 218, MST_ENTRY);
		break;
	case MST_ENTRY:
		switch (SubState) {
		case 0:
			MissionPhase = MMST_ENTRY;
			setSubState(1);
			break;
		case 1:
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
			break;
		}
		break;
	case MST_H3_ABORT_ORBIT:
		if (AbortMode == 5)
		{
			if (cm->GetStage() == CM_ENTRY_STAGE_SEVEN)
			{
				setState(MST_LANDING);
			}
		}
		break;
	case MST_H3_ABORT:
		if (AbortMode == 6)	//Translunar coast. Table I-7 GETIs
		{
			switch (SubState) {
			case 0:
			{
				if (mcc_calcs.GETEval(A14GET(77, 38, 0)))
				{
					setSubState(13);
				}
				else if (mcc_calcs.GETEval(A14GET(60, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(77, 38, 0);
					setSubState(1);
				}
				else if (mcc_calcs.GETEval(A14GET(45, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(60, 0, 0);
				}
				else if (mcc_calcs.GETEval(A14GET(35, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(45, 0, 0);
				}
				else if (mcc_calcs.GETEval(A14GET(25, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(35, 0, 0);
				}
				else if (mcc_calcs.GETEval(A14GET(15, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(25, 0, 0);
				}
				else if (mcc_calcs.GETEval(A14GET(8, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(15, 0, 0);
				}
				else if (mcc_calcs.GETEval(A14GET(4, 0, 0)))
				{
					rtcc->calcParams.TEI = A14GET(8, 0, 0);
				}
				else
				{
					rtcc->calcParams.TEI = A14GET(4, 0, 0);
				}

				if (SubState == 0)
				{
					setSubState(1);
				}
			}
			break;
			case 1:
				if (mcc_calcs.GETEval(rtcc->calcParams.TEI + 10.0 * 60.0))
				{
					startSubthread(205, UTP_NONE);
					setSubState(2);
				}
				break;
			case 2:
				if (rtcc->calcParams.TEI > rtcc->calcParams.EI - 12.0 * 60 * 60)
				{
					setSubState(3);
				}
				else
				{
					setSubState(4);
				}
				break;
			case 3:
				if (mcc_calcs.GETEval(rtcc->calcParams.EI - 4.0 * 3600.0 - 35.0 * 60.0))
				{
					SlowIfDesired();
					setState(MST_H3_TRANSEARTH_DAY4_3);
				}
				break;
			case 4:
				if (mcc_calcs.GETEval(rtcc->calcParams.TEI + 4.0 * 60 * 60))
				{
					SlowIfDesired();
					setSubState(5);
				}
				break;
			case 5:
				allocPad(8);
				if (padForm != NULL) {
					startSubthread(300, UTP_PADWITHCMCUPLINK);
				}
				setSubState(6);
			case 6:
				if (SubStateTime > 1 && padState > -1) {
					if (scrubbed)
					{
						if (upMessage[0] != 0)
						{
							addMessage(upMessage);
						}
						freePad();
						scrubbed = false;
						setSubState(11);
					}
					else
					{
						addMessage("You can has PAD");
						if (padAutoShow == true && padState == 0) { drawPad(); }
						addMessage("Ready for uplink?");
						sprintf(PCOption_Text, "Ready for uplink");
						PCOption_Enabled = true;
						setSubState(7);
					}
				}
				break;
			case 7:
			case 8:
				break;
			case 9:
				if (SubStateTime > 1 && padState > -1) {
					this->CM_uplink_buffer();
					PCOption_Enabled = false;
					if (upDescr[0] != 0)
					{
						addMessage(upDescr);
					}
					setSubState(10);
				}
				break;
			case 10:
				if (cm->pcm.mcc_size == 0) {
					addMessage("Uplink completed!");
					NCOption_Enabled = true;
					sprintf(NCOption_Text, "Repeat uplink");
					setSubState(11);
				}
				break;
			case 11:
				if (mcc_calcs.GETEval(rtcc->calcParams.EI - 4.0 * 3600.0 - 35.0 * 60.0))
				{
					SlowIfDesired();
					setState(MST_H3_TRANSEARTH_DAY4_3);
				}
				break;
			case 12:
				NCOption_Enabled = false;
				setSubState(5);
				break;
			case 13: //Past the flyby TIG: pericynthion from the current vector
				if (mcc_calcs.GETEval(A14GET(77, 38, 0)))
				{
					EphemerisData sv = rtcc->StateVectorCalcEphem(rtcc->calcParams.src);
					double dt = OrbMech::timetoperi(sv.R, sv.V, OrbMech::mu_Moon);
					rtcc->calcParams.LOI = rtcc->GETfromGMT(sv.GMT + dt);
					startSubthread(205, UTP_NONE);
					setSubState(14);
				}
				break;
			case 14:
				if (rtcc->calcParams.EI - rtcc->calcParams.LOI > 45.0 * 3600.0)
				{
					rtcc->calcParams.TEI = rtcc->calcParams.LOI;
					setState(MST_H3_TRANSEARTH_DAY1_1);
				}
				else
				{
					rtcc->calcParams.TEI = rtcc->calcParams.EI - 30.0 * 3600.0;
					setSubState(4);
				}
				break;
			}
		}
		else if (AbortMode == 7) //Lunar orbit
		{
			switch (SubState) {
			case 0:
				if (MoonRevTime > 900.0)
				{
					setSubState(1);
				}
				break;
			case 1:
			{
				EphemerisData sv = rtcc->StateVectorCalcEphem(rtcc->calcParams.src);
				OELEMENTS coe = OrbMech::coe_from_sv(sv.R, sv.V, OrbMech::mu_Moon);
				if (coe.e > 0.7)
				{
					double dt = OrbMech::timetoperi(sv.R, sv.V, OrbMech::mu_Moon);
					rtcc->calcParams.TEI = rtcc->GETfromGMT(sv.GMT + dt);
					setState(MST_H3_TRANSEARTH_DAY1_1);
				}
				else
				{
					setSubState(2);
				}
			}
			break;
			case 2:
				if (MoonRevTime < 100.0)
				{
					setSubState(0);
				}
				break;
			}
		}
		else if (AbortMode == 8)
		{
			//How to Abort?
		}
		break;
	}
}
