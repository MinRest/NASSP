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
#include "saturn.h"
#include "LEM.h"
#include "../src_rtccmfd/OrbMech.h"
#include "mcc.h"
#include "rtcc.h"

// Apollo 14 update codes that would copy Apollo 12 flight-plan constants
// (abort/block-data TIGs, PTC epoch, fixed TEI revolutions, landmark and
// photography targets, the Descartes plane-change site, the LM deorbit
// delta-V, or the hard-coded photography REFSMMAT). Returning true makes
// PAD macros drop the pad. Uplink-only script states for 18, 120-123, and
// 131 are UTP_NONE, so this message is shown and nothing is uplinked.
static void A14FlightPlanGap(int fcn, char* upMessage)
{
	if (upMessage != NULL)
	{
		sprintf(upMessage, "A14 update %d scrubbed: no flight-plan data", fcn);
	}
}

bool RTCC::CalculationMTP_H3(int fcn, LPVOID& pad, char* upString, char* upDesc, char* upMessage)
{
	char uplinkdata[1024 * 3];
	bool scrubbed = false;

	switch (fcn)
	{
	case 12: //TLI+90, Apollo 12 t_Z
	case 13: //Liftoff+8, Apollo 12 TIG and t_Z
	case 16: //Block data 1, Apollo 12 TIGs
	case 17: //Block data 2, Apollo 12 TIGs
	case 18: //PTC REFSMMAT, Apollo 12 absolute MJD
	case 40: //TEI-1 fixed revolution
	case 41:
	case 42:
	case 43:
	case 44:
	case 45:
	case 46:
	case 47:
	case 48:
	case 49: //TEI-45 preliminary. Final/next-rev (50, 51) use the live revolution.
	case 61: //Landmark pads with Apollo 12 coordinates
	case 62:
	case 63:
	case 64:
	case 65:
	case 95: //Plane change 2 writes Descartes into BZLAND
	case 110: //SEP / LM jettison, Apollo 12 times and delta-V
	case 111: //LM deorbit, Apollo 12 delta-V
	case 112: //P76 for that deorbit
	case 120: //P42 / PRO / ENTER / ullage-off for that deorbit
	case 121:
	case 122:
	case 123:
	case 130: //Photography REFSMMAT, Apollo 12 matrix
	case 131:
	case 602: //Named Apollo 12 photo targets
	case 603:
	case 604:
	case 605:
	case 607:
	case 608:
		A14FlightPlanGap(fcn, upMessage);
		return true;

	case 19: //MCC-1 evaluation
	case 21: //MCC-1 update
	case 20: //MCC-2 evaluation
	case 22: //MCC-2 update
	{
		// Same translunar midcourse processor as Mission H1. The LOI anchor is
		// the Apollo 14 skeleton flight plan, not the Apollo 12 ignition GET.
		double P30TIG, MCC1GET, MCC2GET, F23time;
		int engine, mccnum;
		VECTOR3 dV_LVLH;
		EphemerisData sv;
		PLAWDTOutput WeightsTable;
		char Buff[128];
		int hh, mm;
		double ss;

		double dt_lls = PZSFPTAB.blocks[0].dt_lls;
		if (dt_lls <= 0.0)
		{
			dt_lls = PZSFPTAB.blocks[1].dt_lls;
		}

		// dt_lls is time from LOI to landing on the SFP card. Subtracting it
		// from the launch-day TLAND (1971-01-31 Init.txt, also RTCC_TLAND in
		// the scenario) is the SFP pericynthion GET, not a published LOI TIG.
		// TODO(A14): replace LOIFP with a sourced LOI ignition GET.
		if (CZTDTGTU.GETTD <= 0.0 || dt_lls <= 0.0)
		{
			if (upMessage != NULL)
			{
				sprintf(upMessage, "A14 MCC scrubbed: SFP LOI time missing");
			}
			return true;
		}
		double LOIFP = CZTDTGTU.GETTD - dt_lls;

		bool IterateNodeGET = false;
		if (SystemParameters.MCLABN < 77.0 * RAD && calcParams.TLI < OrbMech::HHMMSSToSS(3, 0, 0))
		{
			IterateNodeGET = true;
		}

		// TODO(A14): TLI ignition + 9h / + 28h are the Apollo 12 script offsets.
		// The mission script waits on the same offsets, so the burn and the
		// timeline stay together. They are not a sourced Apollo 14 MCC schedule.
		double TLIbase = calcParams.TLI - 5.0 * 60.0 - 20.0;
		MCC1GET = TLIbase + 9.0 * 3600.0;
		MCC2GET = TLIbase + 28.0 * 3600.0;

		sv = StateVectorCalcEphem(calcParams.src);
		WeightsTable = GetWeightsTable(calcParams.src, true, true);

		PZMCCPLN.MidcourseGET = MCC2GET;
		PZMCCPLN.Config = true;
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 1;
		PZMCCPLN.Mode = 5;

		sprintf_s(Buff, "F23,0.0:0.0:0.0,0.0:0.0:0.0;");
		GMGMED(Buff);

		TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);

		if (IterateNodeGET)
		{
			bool init = true;
			int n = 0;
			F23time = LOIFP - 11.0 * 60.0;
			while ((PZMCCDIS.data[0].GET_LOI < LOIFP - 5.0 || init) && n < 400)
			{
				OrbMech::SStoHHMMSS(F23time, hh, mm, ss, 0.01);
				sprintf_s(Buff, "F23,%d:%d:%.2lf,%d:%d:%.2lf;", hh, mm, ss, hh, mm + 10, ss);
				GMGMED(Buff);
				TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
				F23time = F23time + 5.0;
				if (init) init = false;
				n++;
			}
		}

		if (fcn == 19 || fcn == 21)
		{
			mccnum = 1;

			if (length(PZMCCDIS.data[0].DV_MCC) < 120.0 * 0.3048)
			{
				scrubbed = true;
			}
			else
			{
				PZMCCPLN.MidcourseGET = MCC1GET;

				TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);

				if (IterateNodeGET)
				{
					bool init = true;
					int n = 0;
					F23time = LOIFP - 11.0 * 60.0;
					while ((PZMCCDIS.data[0].GET_LOI < LOIFP - 5.0 || init) && n < 400)
					{
						OrbMech::SStoHHMMSS(F23time, hh, mm, ss, 0.01);
						sprintf_s(Buff, "F23,%d:%d:%.2lf,%d:%d:%.2lf;", hh, mm, ss, hh, mm + 10, ss);
						GMGMED(Buff);
						TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
						F23time = F23time + 5.0;
						if (init) init = false;
						n++;
					}
				}
			}
		}
		else
		{
			mccnum = 2;

			if (length(PZMCCDIS.data[0].DV_MCC) < 1.0 * 0.3048)
			{
				scrubbed = true;
			}
		}

		if (PZMCCDIS.data[0].GET_LOI > 0.0)
		{
			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;
		}
		else
		{
			// Keep later LOI-relative states from firing immediately.
			calcParams.LOI = LOIFP;
			scrubbed = true;
		}
		GMGMED("F30,1;");

		if (!scrubbed)
		{
			engine = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, PZMCCDIS.data[0].DV_MCC);
			PoweredFlightProcessor(sv, WeightsTable.CSMWeight, PZMCCPLN.MidcourseGET, engine, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V, false, P30TIG, dV_LVLH);

			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
		}
		else
		{
			DeltaV_LVLH = _V(0.0, 0.0, 0.0);
		}

		if (fcn > 20)
		{
			if (scrubbed)
			{
				char buffer1[1000];

				if (upMessage != NULL)
				{
					sprintf(upMessage, "MCC-%d has been scrubbed.", mccnum);
				}
				if (upDesc != NULL)
				{
					sprintf(upDesc, "CSM state vector, V66");
				}

				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);

				sprintf(uplinkdata, "%s", buffer1);
				if (upString != NULL) {
					strncpy(upString, uplinkdata, 1024 * 3);
				}
			}
			else
			{
				char buffer1[1000];
				char buffer2[1000];
				AP11ManPADOpt manopt;

				AP11MNV* form = (AP11MNV*)pad;

				manopt.TIG = P30TIG;
				manopt.dV_LVLH = dV_LVLH;
				manopt.enginetype = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, dV_LVLH);
				manopt.HeadsUp = true;
				manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
				manopt.RV_MCC = sv;
				manopt.WeightsTable = WeightsTable;

				AP11ManeuverPAD(manopt, *form);
				sprintf(form->purpose, "MCC-%d", mccnum);
				sprintf(form->remarks, "LM weight is %.0f.", form->LMWeight);

				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
				CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);

				sprintf(uplinkdata, "%s%s", buffer1, buffer2);
				if (upString != NULL) {
					strncpy(upString, uplinkdata, 1024 * 3);
					if (upDesc != NULL)
					{
						sprintf(upDesc, "CSM state vector, V66, Target load");
					}
				}
			}
		}
	}
	break;
	case 32: //State vector and landing-site REFSMMAT
	{
		// The Apollo 12 update writes the Surveyor site here. Apollo 14 keeps
		// the SFP site already stored in BZLAND[RTCC_LMPOS_BEST] by F62.
		MATRIX3 REFSMMAT;
		SV sv;
		REFSMMATOpt opt;
		char buffer1[1000];
		char buffer2[1000];

		sv = StateVectorCalc(calcParams.tgt);

		GZGENCSN.LDPPPoweredDescentSimFlag = false;
		GZGENCSN.LDPPDwellOrbits = 0;
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		// TODO(A14): LOI+25.5h is the Apollo 12 descent-planning threshold.
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = calcParams.LOI + 25.5 * 3600.0;

		if (LunarDescentPlanningProcessor(ConvertSVtoEphemData(sv), 0.0) == 0)
		{
			calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
			calcParams.PDI = PZLDPDIS.PD_GETIG;
			CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;

			PoweredFlightProcessor(sv, calcParams.DOI, RTCC_ENGINETYPE_LMDPS, 0.0, PZLDPELM.V_man_after[0] - PZLDPELM.sv_man_bef[0].V, false, TimeofIgnition, DeltaV_LVLH);

			opt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
			opt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
			opt.REFSMMATopt = 5;
			opt.REFSMMATTime = CZTDTGTU.GETTD;
			opt.vessel = calcParams.src;

			REFSMMAT = REFSMMATCalc(&opt);
			EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
			GMGMED("G00,LEM,LLD,CSM,LCV;");

			AGCStateVectorUpdate(buffer1, sv, true, true);
			AGCDesiredREFSMMATUpdate(buffer2, REFSMMAT);

			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			if (upString != NULL) {
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector, V66, LS REFSMMAT");
			}
		}
		else
		{
			AGCStateVectorUpdate(buffer1, sv, true, true);
			sprintf(uplinkdata, "%s", buffer1);
			if (upString != NULL) {
				strncpy(upString, uplinkdata, 1024 * 3);
				sprintf(upDesc, "CSM state vector, V66");
			}
			if (upMessage != NULL)
			{
				sprintf(upMessage, "A14 LS REFSMMAT skipped: descent planning failed");
			}
		}
	}
	break;
	default:
		// Live-state and mission-file updates: state vectors, DAP, TLI from the
		// Apollo 14 TLI file, maps, LOI/DOI/PDI/ascent solved from the current
		// SV and BZLAND, liftoff-time pads, TEI for the current revolution
		// (updates 50 and 51), PTC quad decision, and transearth MCC/entry.
		//
		// TODO(A14): several of those H1 functions still carry Apollo 12 template
		// offsets or labels. They are not replaced here because the pad itself is
		// integrated from the live trajectory:
		//   11  CSM/LM separation attitude (48.6, -130.9, -139.1 deg)
		//   23  lunar-flyby search bound t_zmin = 145h, TIG guess LOI-5h
		//   31  LOI-2 threshold LOI+3.5h
		//   38  DOI threshold LOI+25.5h (same offset as update 32)
		//   70-73, 100, 105 descent/ascent geometry constants from the H1 script
		//   93  plane-change liftoff seed LOI+58.6h (site is BZLAND, not Surveyor)
		//   110 is scrubbed; 50/51 and 85-88 recompute from the live state
		//   216-218 entry area string is "MIDPAC"; coordinates come from targeting
		//   606, 609 stereo times are terminator crossings, not a named A12 site
		return CalculationMTP_H1(fcn, pad, upString, upDesc, upMessage);
	}

	return scrubbed;
}
