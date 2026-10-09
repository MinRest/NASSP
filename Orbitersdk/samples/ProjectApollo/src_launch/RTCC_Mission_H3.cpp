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

// Planned GET from the Apollo 14 final flight plan, 18 January 1971 (HSI-209261).
// Delta-V is solved on the live trajectory. Preflight delta-V is not copied in.
// Translunar MCC decisions are HSI-43756: MCC-1 was not computed when the predicted
// MCC-2 was 70-90 fps; MCC-3 was not computed when the predicted MCC-4 was 1.7-3.8 fps;
// MCC-2 and, on the flown trajectory, MCC-4 were the burns. Table I-5 lists MCC-1,
// MCC-3, and MCC-4 as nominally zero and MCC-2 as the SPS hybrid transfer.
// The LM-impact note (October 1970) gives the post-rendezvous separation axis and
// the deorbit components. MSC 71-FM54-41 gives the MCC-5 threshold of 1 fps.
// Table I-6 gives ascent 142:24:29 and TPI 143:09:40. It does not give the
// horizontal/vertical insertion split, so those Apollo 12 components are not used.
// MSC-04112: the rendezvous is direct, so there is no CSI pad.

static double A14SS(int h, int m, double s)
{
	return OrbMech::HHMMSSToSS((double)h, (double)m, s);
}

static const double A14_LOI = 82.0 * 3600.0 + 38.0 * 60.0 + 14.0;       // Table I-5
static const double A14_MCC1 = 11.0 * 3600.0 + 36.0 * 60.0 + 33.0;      // Table I-5, nominally zero
static const double A14_MCC2 = 30.0 * 3600.0 + 36.0 * 60.0 + 7.0;       // Table I-5 SPS hybrid transfer
static const double A14_MCC3 = 60.0 * 3600.0 + 38.0 * 60.0 + 14.0;      // Table I-5, nominally zero
static const double A14_MCC4 = 77.0 * 3600.0 + 38.0 * 60.0 + 14.0;      // Table I-5 and the flown GET
// Section E: X in the ecliptic, perpendicular to the earth-moon line at the
// average transearth injection of the monthly window. Not this mission's TEI.
// ARCore stores that Apollo 14 epoch as 166:10:30 GET of the nominal launch.
static const double A14_PTC_REFSMMAT_MJD = 40989.77326433333;
// 1971-01-31 Init.txt TLAND. Landing-site REFSMMAT epoch, section E.
static const double A14_TLAND = 108.8925 * 3600.0;
static const double A14_DOI = 86.0 * 3600.0 + 56.0 * 60.0 + 57.0;       // Table I-5
static const double A14_UNDOCK = 104.0 * 3600.0 + 27.0 * 60.0 + 31.0;   // Table I-5
static const double A14_CIRC = 105.0 * 3600.0 + 46.0 * 60.0 + 48.0;     // Table I-5
static const double A14_PDI = 108.0 * 3600.0 + 42.0 * 60.0 + 1.0;       // Table I-6
static const double A14_PC1 = 118.0 * 3600.0 + 9.0 * 60.0 + 40.0;       // Table I-5
static const double A14_LIFTOFF = 142.0 * 3600.0 + 24.0 * 60.0 + 29.0;  // Table I-6
static const double A14_TPI = 143.0 * 3600.0 + 9.0 * 60.0 + 40.0;       // Table I-6
static const double A14_SEP = 146.0 * 3600.0 + 28.0 * 60.0 + 31.0;      // Table I-5
static const double A14_DEORBIT = 147.0 * 3600.0 + 52.0 * 60.0 + 58.9;  // Table I-6
static const double A14_TEI = 149.0 * 3600.0 + 14.0 * 60.0 + 50.0;      // Table I-5
static const double A14_MCC5 = 166.0 * 3600.0 + 14.0 * 60.0 + 50.0;     // Table I-5
static const double A14_MCC6 = 194.0 * 3600.0 + 26.0 * 60.0 + 59.0;     // Table I-5
static const double A14_MCC7 = 213.0 * 3600.0 + 26.0 * 60.0 + 59.0;     // Table I-5
static const double A14_EOM_LNG = -171.53 * RAD;                        // 1971-01-31 Init.txt

static void A14Msg(char *upMessage, const char *text)
{
	if (upMessage != NULL)
	{
		sprintf(upMessage, "%s", text);
	}
}

static void A14GiveUplink(char *upString, char *upDesc, const char *data, const char *desc)
{
	if (upString != NULL)
	{
		strncpy(upString, data, 1024 * 3);
		upString[1024 * 3 - 1] = 0;
		if (upDesc != NULL && desc != NULL)
		{
			sprintf(upDesc, "%s", desc);
		}
	}
}

// TLI burnout is maneuver 0. H1 also stored a post-TLI separation burn in maneuver 1.
// Apollo 14 does not: that burn carried the unpublished Apollo 12 attitude.
// mantable is a deque, so an index past ManeuverNum is an access violation.
static bool A14MPTBurnout(const MissionPlanTable &mpt, unsigned index, double gmtbase, SV &sv)
{
	if (index >= mpt.ManeuverNum || index >= mpt.mantable.size())
		return false;

	const MPTManeuver &man = mpt.mantable[index];
	// S-IVB TLI leaves CSMMass at the preburn CSM weight. Stack mass is not a CSM pad weight.
	if (man.CommonBlock.CSMMass <= 0.0 || length(man.R_BO) < 1.0)
		return false;

	sv.mass = man.CommonBlock.CSMMass;
	sv.MJD = OrbMech::MJDfromGET(man.GMT_BO, gmtbase);
	sv.R = man.R_BO;
	sv.V = man.V_BO;
	return true;
}

static PLAWDTOutput A14LiveWeights(RTCC *rtcc, VESSEL *v);
static bool A14LMAttached(const PLAWDTOutput &tab);

static void A14P37Line(RTCC *rtcc, EntryOpt &entopt, P37PAD *form, int i, const SV &sv, double tig, double tz)
{
	EntryResults res;
	SV svUse = sv;
	PLAWDTOutput wt = A14LiveWeights(rtcc, entopt.vessel);

	// P37 has no printed weight. The solution mass is still the live configuration.
	if (wt.ConfigWeight > 0.0)
		svUse.mass = wt.ConfigWeight;
	entopt.csmlmdocked = A14LMAttached(wt);
	entopt.TIGguess = form->GETI[i] = tig;
	entopt.t_Z = tz;
	entopt.RV_MCC = svUse;
	rtcc->EntryTargeting(entopt, res);
	form->dVT[i] = length(res.dV_LVLH) / 0.3048;
	form->GET400K[i] = res.GET05G;
	form->lng[i] = round(res.longitude * DEG);
}

static void A14Landmark(LMARKTRKPADOpt &opt, AP11LMARKTRKPAD *form, int i, const char *name, double lat_deg, double lng_deg, double alt_nm, double get)
{
	// Table I-10 altitude is the difference from mean lunar radius 938.4935 nm,
	// which matches OrbMech::R_Moon. South and west are negative.
	sprintf(form->LmkID[i], "%s", name);
	opt.lat[i] = lat_deg * RAD;
	opt.lng[i] = lng_deg * RAD;
	opt.alt[i] = alt_nm * 1852.0;
	opt.LmkTime[i] = get;
}

// nasspdefs.h LBS converts grams to pounds, so LBS*1000 converts kilograms.
static double A14KgToLb(double kg)
{
	return kg * 0.0022046226218 * 1000.0;
}

static void A14FinishWeights(PLAWDTOutput &tab)
{
	const double area = 129.4 * 0.3048 * 0.3048;

	if (tab.CSMWeight > 0.0)
		tab.CSMArea = area;
	if (tab.LMAscWeight > 0.0)
		tab.LMAscArea = area;
	if (tab.LMDscWeight > 0.0)
		tab.LMDscArea = area;
	tab.ConfigArea = tab.CSMArea;
	if (tab.LMAscArea > tab.ConfigArea)
		tab.ConfigArea = tab.LMAscArea;
	if (tab.LMDscArea > tab.ConfigArea)
		tab.ConfigArea = tab.LMDscArea;
	// S-IVB mass is not a CSM or LM pad weight.
	tab.SIVBWeight = 0.0;
	tab.SIVBArea = 0.0;
	tab.CC[RTCC_CONFIG_S] = false;
	tab.ConfigWeight = tab.CSMWeight + tab.LMAscWeight + tab.LMDscWeight;
}

// 'L' sets both A and D. 'A' alone is the ascent stage. 'C' with neither is CSM only.
static bool A14LMAttached(const PLAWDTOutput &tab)
{
	return tab.CC[RTCC_CONFIG_A] || tab.CC[RTCC_CONFIG_D];
}

static bool A14AscentOnly(const PLAWDTOutput &tab)
{
	return tab.CC[RTCC_CONFIG_A] && !tab.CC[RTCC_CONFIG_D];
}

// Live spacecraft at the moment the pad is built. MPTMassUpdate is the same
// call the MPT uses: config "CL"/"CA"/"CSL" when the LM is with the CSM,
// "C"/"CS" when it is not, "L" for a full LM and "A" after staging. The
// S-IVB stays out, including while the stack has not yet reached CSM/LM stage.
static PLAWDTOutput A14LiveWeights(RTCC *rtcc, VESSEL *v)
{
	PLAWDTOutput tab;
	MED_M50 m50;
	MED_M55 m55;
	MED_M49 m49;
	double lm, asc;

	if (rtcc == NULL || v == NULL)
		return tab;

	rtcc->MPTMassUpdate(v, m50, m55, m49, true);
	rtcc->MPTGetConfigFromString(m55.ConfigCode, tab.CC);

	if (tab.CC[RTCC_CONFIG_C] && m50.CSMWT > 0.0)
		tab.CSMWeight = m50.CSMWT;

	lm = m50.LMWT;
	asc = m50.LMASCWT;
	if (!(asc > 0.0))
		asc = lm;
	if (lm > 0.0 && asc > lm)
		asc = lm;

	if (tab.CC[RTCC_CONFIG_D])
	{
		tab.LMAscWeight = asc;
		tab.LMDscWeight = (lm > asc) ? (lm - asc) : 0.0;
	}
	else if (tab.CC[RTCC_CONFIG_A])
	{
		tab.LMAscWeight = (asc > 0.0) ? asc : lm;
		tab.LMDscWeight = 0.0;
	}

	A14FinishWeights(tab);
	return tab;
}

// Post-TLI MPT block. Used when the pad is solved before TLI, while the live
// vessel is still the Earth-orbit stack. Same config bits, still no S-IVB.
static PLAWDTOutput A14WeightsFromBlock(const MPTVehicleDataBlock &cb)
{
	PLAWDTOutput tab;

	tab.CC = cb.ConfigCode;
	if (cb.CSMMass > 0.0)
	{
		tab.CC[RTCC_CONFIG_C] = true;
		tab.CSMWeight = cb.CSMMass;
	}
	if (cb.LMAscentMass > 0.0)
	{
		tab.CC[RTCC_CONFIG_A] = true;
		tab.LMAscWeight = cb.LMAscentMass;
	}
	if (cb.LMDescentMass > 0.0)
	{
		tab.CC[RTCC_CONFIG_D] = true;
		tab.LMDscWeight = cb.LMDescentMass;
	}
	A14FinishWeights(tab);
	return tab;
}

// AP11ManeuverPAD stores N47 from CSMWeight alone. A docked CSM pad prints
// CSM+LM. An undocked CSM pad prints the CSM. The burn table itself is unchanged.
static void A14ShowCSMWeight(AP11MNV *form, const PLAWDTOutput &tab)
{
	double kg;

	if (form == NULL)
		return;
	kg = tab.CSMWeight;
	if (A14LMAttached(tab))
		kg += tab.LMAscWeight + tab.LMDscWeight;
	form->Weight = A14KgToLb(kg);
	form->LMWeight = A14KgToLb(tab.LMAscWeight + tab.LMDscWeight);
}

static void A14LMWeightRemark(AP11MNV *form)
{
	if (form == NULL || !(form->LMWeight > 1.0))
		return;
	if (strstr(form->remarks, "LM weight") != NULL || strstr(form->remarks, "Includes LM") != NULL)
		return;

	char extra[48];
	sprintf(extra, " Includes LM %.0f.", form->LMWeight);
	if (strlen(form->remarks) + strlen(extra) < sizeof(form->remarks))
		strcat(form->remarks, extra);
}

bool RTCC::CalculationMTP_H3(int fcn, LPVOID &pad, char *upString, char *upDesc, char *upMessage)
{
	char uplinkdata[1024 * 3];
	bool scrubbed = false;

	// Flight-plan anchors used before the solved values exist. A later successful
	// solution overwrites them. Do not overwrite a value the processor already set.
	if (calcParams.LOI <= 0.0) calcParams.LOI = A14_LOI;
	if (calcParams.DOI <= 0.0) calcParams.DOI = A14_DOI;
	if (calcParams.PDI <= 0.0) calcParams.PDI = A14_PDI;
	if (calcParams.LunarLiftoff <= 0.0) calcParams.LunarLiftoff = A14_LIFTOFF;
	if (calcParams.SEP <= 0.0) calcParams.SEP = A14_SEP;

	switch (fcn)
	{
	case 11: //TLI simulation from the Apollo 14 TLI file. No separation attitude is added.
	{
		if (PZMPTCSM.ManeuverNum > 0)
		{
			GMGMED("M62,CSM,1,D;");
		}

		med_m55.Table = RTCC_MPT_CSM;
		MPTMassUpdate(calcParams.src, med_m50, med_m55, med_m49);
		PMMWTC(55);
		med_m50.Table = RTCC_MPT_CSM;
		med_m50.WeightGET = GETfromGMT(RTCCPresentTimeGMT());
		PMMWTC(50);

		StateVectorTableEntry sv0;
		sv0.Vector = StateVectorCalcEphem(calcParams.src);
		sv0.LandingSiteIndicator = false;
		sv0.VectorCode = "APIC001";
		PMSVCT(4, RTCC_MPT_CSM, sv0);

		// Second opportunity is the in-tree TLI file, GET 3:00.
		if (mcc->mcc_calcs.GETEval(A14SS(3, 0, 0.0)))
		{
			GMGMED("M68,CSM,2;");
		}
		else
		{
			GMGMED("M68,CSM,1;");
		}

		// Page 3-5 prints the sep FDAI (000, 158, 319) and page 3-6 prints the
		// dock FDAI (301, 338, 041). Those are pad angles, not the LVLH pitch/yaw/roll
		// M66 wants. The CSM separation itself is +X for 3 s (about 0.5 fps) at 03:01.
		// Only the TLI burn is stored, so later pads must use mantable[0].
		if (PZMPTCSM.ManeuverNum < 1 || PZMPTCSM.mantable.empty())
		{
			A14Msg(upMessage, "A14 TLI sim failed: no TLI maneuver in the MPT");
			break;
		}
		TimeofIgnition = GETfromGMT(PZMPTCSM.mantable[0].GMT_BI);
		calcParams.TLI = GETfromGMT(PZMPTCSM.mantable[0].GMT_BO);
		A14Msg(upMessage, "A14 TLI sim. FDAI sep and dock angles are on the TLI pad. No inertial sep attitude is printed, so no sep maneuver is added.");
	}
	break;
	case 12: //TLI+90. Table I-7: GETI 4:00, GETIL 12:12, AOL
	{
		EntryOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		double GMTSV;
		EphemerisData sv, sv_uplink;
		SV sv1;
		char buffer1[1000];

		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);

		// Pad is passed at 1:40, before TLI, so the live CSM state is still in earth orbit.
		// The predicted vector is TLI burnout. There is no second maneuver.
		if (!A14MPTBurnout(PZMPTCSM, 0, SystemParameters.GMTBASE, sv1))
		{
			scrubbed = true;
			A14Msg(upMessage, "TLI+90 skipped: no TLI burnout in the MPT");
			break;
		}
		sv1.gravref = hEarth;
		// TLI burnout config. By GET 4:00 the LM is docked and the S-IVB is gone.
		opt.WeightsTable = A14WeightsFromBlock(PZMPTCSM.mantable[0].CommonBlock);
		sv1.mass = opt.WeightsTable.ConfigWeight;

		entopt.entrylongmanual = false;
		entopt.ATPLine = 2; //Table I-7 note 1: TLI+90 is AOL
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.TIGguess = A14SS(4, 0, 0.0);
		entopt.t_Z = A14SS(12, 12, 0.0);
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		entopt.RV_MCC = sv1;

		EntryTargeting(entopt, res);

		opt.TIG = res.P30TIG;
		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.RV_MCC = ConvertSVtoEphemData(sv1);

		AP11ManeuverPAD(opt, *form);
		A14ShowCSMWeight(form, opt.WeightsTable);
		form->lat = res.latitude * DEG;
		form->lng = res.longitude * DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->GET05G = res.GET05G;
		sprintf(form->purpose, "TLI+90");
		sprintf(form->remarks, "AOL, GETI 4:00, TLI burnout SV");
		A14LMWeightRemark(form);

		GMTSV = PZMPTCSM.TimeToBeginManeuver[0] - 10.0 * 60.0;
		sv_uplink = coast(sv, GMTSV - sv.GMT, RTCC_MPT_CSM);
		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv_uplink, true);
		sprintf(uplinkdata, "%s", buffer1);
		A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
	}
	break;
	case 14: //TLI pad. SEP and extraction FDAI are the printed flight-plan angles.
	{
		TLIPAD *form = (TLIPAD *)pad;

		if (PZMPTCSM.ManeuverNum < 1 || PZMPTCSM.mantable.empty())
		{
			scrubbed = true;
			A14Msg(upMessage, "TLI pad skipped: no TLI maneuver in the MPT");
			break;
		}

		GMGMED("U20,CSM,1;");
		// TB6 starts 9 min 38 s before TLI ignition. That lead is the S-IVB timebase, not an attitude.
		form->TB6P = DMTBuffer[0].GETI - 9.0 * 60.0 - 38.0;
		form->IgnATT = DMTBuffer[0].IMUAtt;
		form->BurnTime = DMTBuffer[0].DT_B;
		form->dVC = DMTBuffer[0].DVC;
		if (PZMPTCSM.mantable.empty())
		{
			scrubbed = true;
			A14Msg(upMessage, "TLI pad skipped: TLI maneuver was not in the MPT");
			break;
		}
		form->VI = length(PZMPTCSM.mantable[0].V_BO) / 0.3048;
		// type 2 prints both SEP and extraction. Angles are FDAI roll, pitch, yaw.
		// Page 3-5 (02:00-03:00): S-IVB MNVRS TO SEP ATT 02:51:34 (000, 158, 319).
		// Page 3-6 (03:00-04:00): CSM MNVR TO DOCK ATT (301, 338, 041). The pad's
		// extraction line is that dock attitude. The stack is still attached at sep.
		form->type = 2;
		form->SepATT = _V(0.0, 158.0, 319.0);
		form->ExtATT = _V(301.0, 338.0, 41.0);
		sprintf(form->remarks, "SEP p3-5 000/158/319. Extraction is dock att p3-6 301/338/041.");

		GMGMED("M62,CSM,1,D;");
		EZANCHR1.AnchorVectors[9].Vector.GMT = 0.0;
	}
	break;
	case 39: //CSM/LM ejection sep. External delta-V from the live state and REFSMMAT.
	{
		AP11ManPADOpt opt;
		AP11MNV *form = (AP11MNV *)pad;

		// Flight plan p. 3-6 (03:00-04:00): "TLI CUTOFF + 1 HR 20 MIN", "CSM/LM EJECTION",
		// "dVT: 0.4 FPS", "ULLAGE: NONE". The note is a 4-jet RCS -X translation for 3 s
		// (about 0.4 fps) after the spring ejection. HSI-43756 Table IX premission SC
		// ejection is 03:56:34. Table X CSM/LM SEP prelaunch is 03:56:34, 3.0 s, 0.4 fps
		// (real-time plan 03:56:00 and 0.3 fps; flown 05:47:25 and 0.8 fps after the probe
		// delay, p. 4 and 71-FM54-41 p. 2). This pad uses the flight-plan TIG and 0.4 fps.
		// calcParams.TLI is TLI burnout, so cutoff + 1:20 is the p. 3-6 rule.
		// Page 3-9 (04:00-05:00) puts 04:19 on "S-IVB APS EVASIVE BURN (GROUND COMMAND)".
		// That is the S-IVB DCS evasive, not this CSM pad, so the TIG stays at cutoff + 1:20.
		// The burn is RCS, not SPS, so the pad trim is zero. There is no SPS gimbal.
		if (calcParams.TLI > 1.0)
		{
			opt.TIG = calcParams.TLI + 80.0 * 60.0;
		}
		else
		{
			opt.TIG = A14SS(3, 56, 34.0);
		}
		// After the p. 3-6 dock, CSM +X points at the LM and the S-IVB behind it, so -X
		// is away from the S-IVB and posigrade (LVLH +X). No component table is printed.
		// Minus-4 points body +X opposite this delta-V. The FDAI is that solution, not
		// the dock angles 301/338/041, and not the Apollo 11 evasive vector.
		opt.dV_LVLH = _V(0.4, 0.0, 0.0) * 0.3048;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSMINUS4;
		opt.HeadsUp = true;
		opt.UllageDT = 0.0;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.RV_MCC = StateVectorCalcEphem(calcParams.src);
		opt.WeightsTable = A14LiveWeights(this, calcParams.src);

		AP11ManeuverPAD(opt, *form);
		A14ShowCSMWeight(form, opt.WeightsTable);
		sprintf(form->purpose, "CSM/LM SEP");
		sprintf(form->remarks, "p3-6 4-jet -X, 0.4 fps, no ullage. TIG = TLI cutoff +1:20. Table X prelaunch.");
		A14LMWeightRemark(form);
	}
	break;
	case 13: //L/O+8. Table I-7: GETI 8:00, GETIL 46:29, MPL
	{
		EntryOpt entopt;
		SV sv1;
		P37PAD *form = (P37PAD *)pad;

		if (!A14MPTBurnout(PZMPTCSM, 0, SystemParameters.GMTBASE, sv1))
		{
			scrubbed = true;
			A14Msg(upMessage, "L/O+8 skipped: no TLI burnout in the MPT");
			break;
		}
		sv1.gravref = hEarth;

		entopt.entrylongmanual = false;
		entopt.ATPLine = 0;
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		A14P37Line(this, entopt, form, 0, sv1, A14SS(8, 0, 0.0), A14SS(46, 29, 0.0));
	}
	break;
	case 16: //L/O+15, passed at 6:00. Table I-7: GETI 15:00, GETIL 45:56
	{
		EntryOpt entopt;
		SV sv1;
		P37PAD *form = (P37PAD *)pad;

		sv1 = StateVectorCalc(calcParams.src);
		entopt.entrylongmanual = false;
		entopt.ATPLine = 0;
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.type = 1;
		entopt.vessel = calcParams.src;
		A14P37Line(this, entopt, form, 0, sv1, A14SS(15, 0, 0.0), A14SS(45, 56, 0.0));
	}
	break;
	case 17: //Block data 2, passed at 14:00. Table I-7 L/O+25, +35, +45, +60
	{
		EntryOpt entopt;
		SV sv1, sv2;
		P37PAD *form = (P37PAD *)pad;

		sv1 = StateVectorCalc(calcParams.src);

		entopt.entrylongmanual = false;
		entopt.ATPLine = 0;
		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.type = 1;
		entopt.vessel = calcParams.src;

		// Note 2: L/O+15 assumes no MCC-1. These lines are before MCC-2 except note 3.
		A14P37Line(this, entopt, form, 0, sv1, A14SS(25, 0, 0.0), A14SS(70, 3, 0.0));
		A14P37Line(this, entopt, form, 2, sv1, A14SS(45, 0, 0.0), A14SS(93, 49, 0.0));
		A14P37Line(this, entopt, form, 3, sv1, A14SS(60, 0, 0.0), A14SS(117, 53, 0.0));

		// Note 3: L/O+35 assumes the hybrid MCC-2. Solve it; do not invent its delta-V.
		EphemerisData svMCC;
		PLAWDTOutput wt;
		VECTOR3 dvMCC;
		svMCC = StateVectorCalcEphem(calcParams.src);
		wt = A14LiveWeights(this, calcParams.src);
		PZMCCPLN.MidcourseGET = A14_MCC2;
		PZMCCPLN.Config = A14LMAttached(wt);
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 1;
		PZMCCPLN.Mode = 5;
		GMGMED("F23,0.0:0.0:0.0,0.0:0.0:0.0;");
		TranslunarMidcourseCorrectionProcessor(svMCC, wt.CSMWeight, wt.LMAscWeight + wt.LMDscWeight);
		dvMCC = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;
		if (PZMCCXFR.sv_man_bef[0].GMT <= 0.0 || length(dvMCC) < 0.3048)
		{
			A14Msg(upMessage, "L/O+35 not filled: Table I-7 note 3 assumes MCC-2, and that solution is null");
		}
		else
		{
			double p30;
			VECTOR3 dvLvlh;
			int engine = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / wt.ConfigWeight, dvMCC);
			PoweredFlightProcessor(svMCC, wt.CSMWeight, A14_MCC2, engine, wt.LMAscWeight + wt.LMDscWeight, dvMCC, false, p30, dvLvlh);
			sv2 = ExecuteManeuver(sv1, p30, dvLvlh, wt.LMAscWeight + wt.LMDscWeight, RTCC_ENGINETYPE_CSMSPS);
			A14P37Line(this, entopt, form, 1, sv2, A14SS(35, 0, 0.0), A14SS(69, 28, 0.0));
		}
	}
	break;
	case 18: //PTC REFSMMAT. Section E average-window epoch, shared by TLC and TEC
	{
		char buffer[1000];
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;

		refsopt.REFSMMATopt = 6;
		refsopt.REFSMMATTime = A14_PTC_REFSMMAT_MJD;
		REFSMMAT = REFSMMATCalc(&refsopt);
		AGCDesiredREFSMMATUpdate(buffer, REFSMMAT);
		sprintf(uplinkdata, "%s", buffer);
		A14GiveUplink(upString, upDesc, uplinkdata, "PTC REFSMMAT");
	}
	break;
	case 19: //MCC-1 evaluation
	case 21: //MCC-1 update
	case 20: //MCC-2 evaluation
	case 22: //MCC-2 update
	{
		double P30TIG, MCC1GET, MCC2GET, F23time;
		int engine, mccnum;
		VECTOR3 dV_LVLH;
		EphemerisData sv;
		PLAWDTOutput WeightsTable;
		char Buff[128];
		int hh, mm;
		double ss;

		bool IterateNodeGET = false;
		if (SystemParameters.MCLABN < 77.0 * RAD && calcParams.TLI < A14SS(3, 0, 0.0))
		{
			IterateNodeGET = true;
		}

		MCC1GET = A14_MCC1;
		MCC2GET = A14_MCC2;

		sv = StateVectorCalcEphem(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);

		PZMCCPLN.MidcourseGET = MCC2GET;
		PZMCCPLN.Config = A14LMAttached(WeightsTable);
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
			F23time = A14_LOI - 11.0 * 60.0;
			while ((PZMCCDIS.data[0].GET_LOI < A14_LOI - 5.0 || init) && n < 400)
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

		int mcc1reason = 0;
		if (fcn == 19 || fcn == 21)
		{
			double mcc2dv = length(PZMCCDIS.data[0].DV_MCC);
			mccnum = 1;
			// HSI-43756: MCC-1 was not computed when predicted MCC-2 was 70-90 fps.
			// A null MCC-2 solution is not a reason to invent an MCC-1 burn.
			if (mcc2dv >= 70.0 * 0.3048 && mcc2dv <= 90.0 * 0.3048)
			{
				scrubbed = true;
				mcc1reason = 1;
			}
			else if (mcc2dv < 0.3048)
			{
				scrubbed = true;
				mcc1reason = 2;
			}
			else
			{
				PZMCCPLN.MidcourseGET = MCC1GET;
				TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
				if (IterateNodeGET)
				{
					bool init = true;
					int n = 0;
					F23time = A14_LOI - 11.0 * 60.0;
					while ((PZMCCDIS.data[0].GET_LOI < A14_LOI - 5.0 || init) && n < 400)
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
				if (length(PZMCCDIS.data[0].DV_MCC) < 0.3048)
				{
					scrubbed = true;
					mcc1reason = 2;
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
			calcParams.LOI = A14_LOI;
			scrubbed = true;
		}
		GMGMED("F30,1;");

		if (!scrubbed)
		{
			// Table I-5: MCC-2 is an SPS burn. MCC-1 has no engine because it is nominally zero.
			engine = (mccnum == 2) ? RTCC_ENGINETYPE_CSMSPS : mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, PZMCCDIS.data[0].DV_MCC);
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
				if (mccnum == 1 && mcc1reason == 1)
				{
					A14Msg(upMessage, "MCC-1 not computed: predicted MCC-2 is within 70-90 fps (HSI-43756). Table I-5 is nominally zero.");
				}
				else if (mccnum == 1)
				{
					A14Msg(upMessage, "MCC-1 not computed: no usable solution. Table I-5 MCC-1 is nominally zero.");
				}
				else
				{
					A14Msg(upMessage, "MCC-2 skipped: hybrid-transfer solution is null. Table I-5 plans 30:36:07, 73.40 fps SPS.");
				}
				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
				sprintf(uplinkdata, "%s", buffer1);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
			}
			else
			{
				char buffer1[1000];
				char buffer2[1000];
				AP11ManPADOpt manopt;
				AP11MNV *form = (AP11MNV *)pad;

				manopt.TIG = P30TIG;
				manopt.dV_LVLH = dV_LVLH;
				// Table I-5: MCC-2 is an SPS burn, ullage not required. MCC-1 has no engine row.
				manopt.enginetype = (mccnum == 2) ? RTCC_ENGINETYPE_CSMSPS : mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, dV_LVLH);
				manopt.HeadsUp = true;
				manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
				manopt.RV_MCC = sv;
				manopt.WeightsTable = WeightsTable;
				// Printed MCC FDAI in the flight plan are preflight values. Attitude comes from this solution.
				AP11ManeuverPAD(manopt, *form);
				sprintf(form->purpose, "MCC-%d", mccnum);
				if (mccnum == 2)
				{
					sprintf(form->remarks, "Ullage not required. PTC REFSMMAT. Table I-5 planned 73.40 fps; delta-V is solved.");
				}
				else
				{
					sprintf(form->remarks, "PTC REFSMMAT. Predicted MCC-2 is outside the 70-90 fps band.");
				}
				A14ShowCSMWeight(form, WeightsTable);
				A14LMWeightRemark(form);
				AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
				CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
				sprintf(uplinkdata, "%s%s", buffer1, buffer2);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load");
			}
		}
	}
	break;
	case 23: //Lunar flyby. Table I-7: GETI 77:38, GETIL 165:57, docked, MPL
	{
		RTEMoonOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		SV sv;
		PLAWDTOutput WeightsTable;
		char buffer1[1000];
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		entopt.SMODE = 14;
		entopt.RV_MCC = sv;
		entopt.TIGguess = A14SS(77, 38, 0.0);
		entopt.vessel = calcParams.src;
		entopt.t_zmin = A14SS(165, 57, 0.0);
		entopt.entrylongmanual = false;
		entopt.ATPLine = 0;
		entopt.csmlmdocked = A14LMAttached(WeightsTable);
		RTEMoonTargeting(&entopt, &res);

		opt.TIG = res.P30TIG;
		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, res.dV_LVLH);
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.RV_MCC = ConvertSVtoEphemData(sv);
		opt.WeightsTable = WeightsTable;
		AP11ManeuverPAD(opt, *form);
		sprintf(form->purpose, "Flyby");
		sprintf(form->remarks, "Height of pericynthion is %.0f NM", res.FlybyAlt / 1852.0);
		A14ShowCSMWeight(form, WeightsTable);
		A14LMWeightRemark(form);
		// Table I-7 note 4. Passed at 35:00; a negative height means it is not clear of the Moon.
		if (res.FlybyAlt < 0.0)
		{
			A14Msg(upMessage, "Flyby pericynthion is not clear of the Moon. Table I-7 note 4.");
		}
		form->lat = res.latitude * DEG;
		form->lng = res.longitude * DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->GET05G = res.GET05G;
		AGCStateVectorUpdate(buffer1, sv, true, true);
		sprintf(uplinkdata, "%s", buffer1);
		A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
	}
	break;
	case 24: //MCC-3 at Table I-5 60:38:14. HSI-43756: not computed when MCC-4 is 1.7-3.8 fps
	{
		AP11ManPADOpt manopt;
		VECTOR3 dV_LVLH, dv, dv4;
		EphemerisData sv;
		PLAWDTOutput WeightsTable;
		double P30TIG, tig;
		int engine;
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		PZMCCPLN.Config = A14LMAttached(WeightsTable);
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 2;
		PZMCCPLN.Mode = 1;
		PZMCCPLN.MidcourseGET = A14_MCC4;
		TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
		dv4 = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;
		// 3.8 fps is the top of the range in which MCC-3 was not computed.
		if (PZMCCXFR.sv_man_bef[0].GMT <= 0.0 || length(dv4) <= 3.8 * 0.3048)
		{
			scrubbed = true;
			A14Msg(upMessage, "MCC-3 not computed: predicted MCC-4 is not above 3.8 fps (HSI-43756). Table I-5 MCC-3 is nominally zero.");
		}

		if (!scrubbed)
		{
			PZMCCPLN.MidcourseGET = A14_MCC3;
			TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
		}

		tig = GETfromGMT(PZMCCXFR.sv_man_bef[0].GMT);
		dv = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;
		if (!scrubbed && (PZMCCXFR.sv_man_bef[0].GMT <= 0.0 || length(dv) < 0.3048))
		{
			scrubbed = true;
			A14Msg(upMessage, "MCC-3 skipped: solution is null. Table I-5 is nominally zero.");
		}

		if (scrubbed)
		{
			char buffer1[1000];
			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
			sprintf(uplinkdata, "%s", buffer1);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
		}
		else
		{
			char buffer1[1000];
			char buffer2[1000];
			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;
			engine = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / WeightsTable.ConfigWeight, dv);
			PoweredFlightProcessor(sv, WeightsTable.CSMWeight, tig, engine, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, dv, false, P30TIG, dV_LVLH);
			manopt.TIG = P30TIG;
			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = engine;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.RV_MCC = sv;
			manopt.WeightsTable = WeightsTable;
			// Printed MCC FDAI in the flight plan are preflight values. Attitude comes from this solution.
			AP11ManeuverPAD(manopt, *form);
			sprintf(form->purpose, "MCC-3");
			sprintf(form->remarks, "PTC REFSMMAT. Table I-5 nominally zero; predicted MCC-4 is above 3.8 fps.");
			A14ShowCSMWeight(form, WeightsTable);
			A14LMWeightRemark(form);
			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
			AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load");
		}
	}
	break;
	case 25: //MCC-4 decision. Table I-5 nominally zero; HSI-43756 flew it when perilune was high.
	{
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		EphemerisData sv;
		PLAWDTOutput WeightsTable;
		VECTOR3 dv;

		if (CZTDTGTU.GETTD <= 0.0)
		{
			CZTDTGTU.GETTD = A14_TLAND;
		}
		refsopt.LSAzi = calcParams.LSAzi;
		refsopt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
		refsopt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
		refsopt.REFSMMATopt = 8;
		refsopt.REFSMMATTime = CZTDTGTU.GETTD;
		REFSMMAT = REFSMMATCalc(&refsopt);
		EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
		GMGMED("G00,LEM,LLD,CSM,LCV;");

		sv = StateVectorCalcEphem(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		PZMCCPLN.MidcourseGET = A14_MCC4;
		PZMCCPLN.Config = A14LMAttached(WeightsTable);
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 2;
		PZMCCPLN.Mode = 1;
		TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
		dv = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;
		if (PZMCCDIS.data[0].GET_LOI > 0.0)
		{
			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;
		}
		if (PZMCCXFR.sv_man_bef[0].GMT <= 0.0 || length(dv) < 0.3048)
		{
			scrubbed = true;
			DeltaV_LVLH = _V(0.0, 0.0, 0.0);
			A14Msg(upMessage, "MCC-4 scrubbed: solved correction is null. Table I-5 is nominally zero. HSI-43756 flew 3.8 fps SPS at 77:38:14 GET because perilune was 65.17 nm.");
		}
		else
		{
			A14Msg(upMessage, "MCC-4 will be executed at 77:38:14. HSI-43756 used the SPS when the incoming perilune was high.");
		}
	}
	break;
	case 26: //MCC-4 pad at Table I-5 / flown GET 77:38:14. SPS, as flown, to save RCS.
	{
		AP11ManPADOpt manopt;
		VECTOR3 dV_LVLH, dv;
		EphemerisData sv;
		PLAWDTOutput WeightsTable;
		double P30TIG, tig;
		int engine;
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];
		MATRIX3 REFSMMAT;
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		PZMCCPLN.MidcourseGET = A14_MCC4;
		PZMCCPLN.Config = A14LMAttached(WeightsTable);
		PZMCCPLN.Column = 1;
		PZMCCPLN.SFPBlockNum = 2;
		PZMCCPLN.Mode = 1;
		TranslunarMidcourseCorrectionProcessor(sv, WeightsTable.CSMWeight, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight);
		if (PZMCCDIS.data[0].GET_LOI > 0.0)
		{
			calcParams.LOI = PZMCCDIS.data[0].GET_LOI;
		}

		tig = GETfromGMT(PZMCCXFR.sv_man_bef[0].GMT);
		dv = PZMCCXFR.V_man_after[0] - PZMCCXFR.sv_man_bef[0].V;
		if (PZMCCXFR.sv_man_bef[0].GMT <= 0.0 || length(dv) < 0.3048)
		{
			scrubbed = true;
			DeltaV_LVLH = _V(0.0, 0.0, 0.0);
			A14Msg(upMessage, "MCC-4 pad skipped: solved correction is null. Table I-5 is nominally zero.");
			break;
		}
		// HSI-43756: the flown MCC-4 was an SPS minimum-impulse burn, to save RCS.
		engine = RTCC_ENGINETYPE_CSMSPS;
		PoweredFlightProcessor(sv, WeightsTable.CSMWeight, tig, engine, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, dv, false, P30TIG, dV_LVLH);

		manopt.TIG = P30TIG;
		manopt.dV_LVLH = dV_LVLH;
		manopt.enginetype = engine;
		manopt.HeadsUp = false;
		manopt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;
		manopt.RV_MCC = sv;
		manopt.WeightsTable = WeightsTable;
		// Printed MCC FDAI in the flight plan are preflight values. Attitude comes from this solution.
		AP11ManeuverPAD(manopt, *form);
		sprintf(form->purpose, "MCC-4");
		sprintf(form->remarks, "SPS, PTC REFSMMAT. Table I-5 TIG 77:38:14, nominally zero.");
		A14ShowCSMWeight(form, WeightsTable);
		A14LMWeightRemark(form);
		TimeofIgnition = P30TIG;
		DeltaV_LVLH = dV_LVLH;

		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
		CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
		REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;
		AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);
		sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
		A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load, Landing Site REFSMMAT");
	}
	break;
	case 27: //PC+2 after MCC-4. Table I-7 GETI 84:36, GETIL 141:42
	case 28: //PC+2 with MCC-4 scrubbed
	{
		RTEMoonOpt entopt;
		EntryResults res;
		AP11ManPADOpt opt;
		SV sv, sv1;
		PLAWDTOutput WeightsTable;
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		if (fcn == 27 && length(DeltaV_LVLH) >= 0.3048)
		{
			sv1 = ExecuteManeuver(sv, TimeofIgnition, DeltaV_LVLH, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, RTCC_ENGINETYPE_CSMSPS);
			WeightsTable.CSMWeight = sv1.mass;
			WeightsTable.ConfigWeight = WeightsTable.CSMWeight + WeightsTable.LMAscWeight + WeightsTable.LMDscWeight;
		}
		else
		{
			sv1 = sv;
		}

		entopt.returnspeed = 2;
		entopt.SMODE = 14;
		entopt.RV_MCC = sv1;
		entopt.vessel = calcParams.src;
		entopt.TIGguess = A14SS(84, 36, 0.0);
		entopt.t_zmin = A14SS(141, 42, 0.0);
		entopt.csmlmdocked = A14LMAttached(WeightsTable);
		PZREAP.VRMAX = 37500.0;
		entopt.entrylongmanual = false;
		entopt.ATPLine = 0;
		RTEMoonTargeting(&entopt, &res);
		PZREAP.VRMAX = 36323.0;

		opt.TIG = res.P30TIG;
		opt.dV_LVLH = res.dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = false;
		opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;
		opt.RV_MCC = ConvertSVtoEphemData(sv1);
		opt.WeightsTable = WeightsTable;
		AP11ManeuverPAD(opt, *form);
		if (A14LMAttached(WeightsTable))
			sprintf(form->remarks, "Assumes LS REFSMMAT and docked");
		else
			sprintf(form->remarks, "Assumes LS REFSMMAT");
		if (!mcc->mcc_calcs.REFSMMATDecision(form->Att * RAD))
		{
			REFSMMATOpt refsopt;
			MATRIX3 REFSMMAT;
			refsopt.dV_LVLH = res.dV_LVLH;
			refsopt.REFSMMATTime = res.P30TIG;
			refsopt.REFSMMATopt = 0;
			refsopt.vessel = calcParams.src;
			REFSMMAT = REFSMMATCalc(&refsopt);
			opt.HeadsUp = true;
			opt.REFSMMAT = REFSMMAT;
			AP11ManeuverPAD(opt, *form);
			if (A14LMAttached(WeightsTable))
				sprintf(form->remarks, "Docked, preferred REFSMMAT");
			else
				sprintf(form->remarks, "Preferred REFSMMAT");
		}
		sprintf(form->purpose, "PC+2");
		if (fcn == 28)
		{
			sprintf(form->remarks, "Table I-7 note 5 assumes MCC-4; MCC-4 was not executed");
		}
		A14ShowCSMWeight(form, WeightsTable);
		A14LMWeightRemark(form);
		form->lat = res.latitude * DEG;
		form->lng = res.longitude * DEG;
		form->RTGO = res.RTGO;
		form->VI0 = res.VIO / 0.3048;
		form->GET05G = res.GET05G;
		form->type = 2;
	}
	break;
	case 31: //CSM SPS DOI. Table I-5 HP 9.77 nm, 4-jet ullage 14 s. Mode 4 is DOI only
	{
		AP11ManPADOpt manopt;
		double P30TIG;
		VECTOR3 dV_LVLH;
		SV sv;
		PLAWDTOutput WeightsTable;
		char buffer1[1000];
		char buffer2[1000];
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		med_k16.Mode = 4;
		med_k16.Sequence = 1;
		med_k16.GETTH1 = A14_DOI;
		med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = med_k16.GETTH1;
		med_k16.DesiredHeight = 9.77 * 1852.0;

		if (LunarDescentPlanningProcessor(ConvertSVtoEphemData(sv), 0.0) == 0)
		{
			PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], RTCC_ENGINETYPE_CSMSPS, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, PZLDPDIS.DVVector[0] * 0.3048, true, P30TIG, dV_LVLH);
			manopt.TIG = P30TIG;
			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.sxtstardtime = -40.0 * 60.0;
			manopt.RV_MCC = ConvertSVtoEphemData(sv);
			manopt.WeightsTable = WeightsTable;
			AP11ManeuverPAD(manopt, *form);
			sprintf(form->purpose, "DOI");
			sprintf(form->remarks, "Ullage: 4 jet, 14 seconds");
			A14ShowCSMWeight(form, WeightsTable);
			A14LMWeightRemark(form);
			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
			calcParams.DOI = P30TIG;
			AGCStateVectorUpdate(buffer1, sv, true, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load");
		}
		else
		{
			scrubbed = true;
			A14Msg(upMessage, "A14 DOI 86:56:57 GET, 206.6 fps; targeting failed");
			AGCStateVectorUpdate(buffer1, sv, true, true);
			sprintf(uplinkdata, "%s", buffer1);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
		}
	}
	break;
	case 32: //Landing-site REFSMMAT. Threshold is Table I-6 PDI, site stays Fra Mauro
	{
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
		med_k16.GETTH1 = med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = A14_PDI;

		if (LunarDescentPlanningProcessor(ConvertSVtoEphemData(sv), 0.0) == 0)
		{
			PLAWDTOutput lmWt = A14LiveWeights(this, calcParams.tgt);
			double lmMass = lmWt.LMAscWeight + lmWt.LMDscWeight;
			double attached = lmWt.CC[RTCC_CONFIG_C] ? lmWt.CSMWeight : 0.0;

			calcParams.DOI = GETfromGMT(PZLDPELM.sv_man_bef[0].GMT);
			calcParams.PDI = PZLDPDIS.PD_GETIG;
			CZTDTGTU.GETTD = PZLDPDIS.PD_GETTD;
			if (lmMass > 0.0)
				sv.mass = lmMass;
			PoweredFlightProcessor(sv, calcParams.DOI, RTCC_ENGINETYPE_LMDPS, attached, PZLDPELM.V_man_after[0] - PZLDPELM.sv_man_bef[0].V, false, TimeofIgnition, DeltaV_LVLH);

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
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, LS REFSMMAT");
		}
		else
		{
			calcParams.PDI = A14_PDI;
			AGCStateVectorUpdate(buffer1, sv, true, true);
			sprintf(uplinkdata, "%s", buffer1);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
			A14Msg(upMessage, "A14 LS REFSMMAT skipped: descent planning failed");
		}
	}
	break;
	case 35: //LGC activation. DOI is already the CSM burn, so the LM SV is post-DOI
	{
		VehicleDataBlock sv_CSM, sv_LM, sv_LM_post_DOI;
		MATRIX3 REFSMMAT;
		double t_sunrise1, t_sunrise2, t_TPI;
		double tephem, t_AGC, t_actual, deltaT;
		int emem[14];
		char buffer1[1000];
		char buffer2[1000];
		char buffer3[1000];
		char clockupdate[128];
		PDAPOpt opt;
		PDAPResults res;
		LEM *l = (LEM *)calcParams.tgt;

		sv_CSM = StateVectorCalcDataBlock(calcParams.src);
		sv_LM = StateVectorCalcDataBlock(calcParams.tgt);
		tephem = GetTEPHEMFromAGC(&l->agc.vagc, false);
		t_AGC = GetClockTimeFromAGC(&l->agc.vagc) / 100.0;
		tephem = (tephem / 8640000.) + SystemParameters.TEPHEM0;
		t_actual = (oapiGetSimMJD() - tephem) * 86400.0;
		deltaT = t_actual - t_AGC;
		IncrementAGCTime(clockupdate, RTCC_MPT_LM, deltaT);

		sv_LM_post_DOI = sv_LM;
		t_sunrise1 = calcParams.PDI + 3.0 * 3600.0;
		t_sunrise2 = calcParams.PDI + 4.5 * 3600.0;
		t_TPI = mcc->mcc_calcs.FindOrbitalSunrise(sv_CSM, t_sunrise1) - 23.0 * 60.0;

		opt.dt_stage = 999999.9;
		opt.W_TAPS = l->GetAscentStageMass();
		opt.W_TDRY = l->GetMass() - l->GetPropellantMass(l->GetPropellantHandleByIndex(0));
		opt.IsTwoSegment = true;
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv_LM = sv_LM_post_DOI;
		opt.sv_CSM = sv_CSM;
		opt.GMT_LAND = GMTfromGET(CZTDTGTU.GETTD);
		opt.dt_CAN = 0.0;
		opt.DV_CAN = _V(0, 0, 0);
		opt.dt_CSI = 50.0 * 60.0;
		opt.GMT_TPI = GMTfromGET(t_TPI);
		opt.dt_2CAN = 50.0 * 60.0;
		opt.DV_2CAN = _V(10.0, 0, 0) * 0.3048;
		opt.dt_2CSI = 110.0 * 60.0;
		opt.GMT_2TPI = mcc->mcc_calcs.FindOrbitalSunrise(sv_CSM, t_sunrise2) - 23.0 * 60.0;
		PoweredDescentAbortProgram(opt, res);

		calcParams.SVSTORE1.R.x = (int)(res.J1 / 0.3048 / 100.0);
		calcParams.SVSTORE1.R.y = (int)(res.A_min / 0.3048 / 100.0);
		calcParams.SVSTORE1.R.z = (int)(res.A_max / 0.3048 / 100.0);
		calcParams.SVSTORE1.V.x = (int)(res.K1 / 0.3048 / 100.0 * pow(2, 3));

		emem[0] = 16;
		emem[1] = 2550;
		emem[2] = OrbMech::DoubleToBuffer(res.J1, 23, 1);
		emem[3] = OrbMech::DoubleToBuffer(res.J1, 23, 0);
		emem[4] = OrbMech::DoubleToBuffer(res.K1 * PI2, 23, 1);
		emem[5] = OrbMech::DoubleToBuffer(res.K1 * PI2, 23, 0);
		emem[6] = OrbMech::DoubleToBuffer(res.J2, 23, 1);
		emem[7] = OrbMech::DoubleToBuffer(res.J2, 23, 0);
		emem[8] = OrbMech::DoubleToBuffer(res.K2 * PI2, 23, 1);
		emem[9] = OrbMech::DoubleToBuffer(res.K2 * PI2, 23, 0);
		emem[10] = OrbMech::DoubleToBuffer(res.Theta_LIM / PI2, 0, 1);
		emem[11] = OrbMech::DoubleToBuffer(res.Theta_LIM / PI2, 0, 0);
		emem[12] = OrbMech::DoubleToBuffer(res.R_amin, 24, 1);
		emem[13] = OrbMech::DoubleToBuffer(res.R_amin, 24, 0);
		V7XUpdate(71, buffer2, emem, 14);

		REFSMMAT = EZJGMTX3.data[RTCC_REFSMMAT_TYPE_LLD - 1].REFSMMAT;
		AGCStateVectorUpdate(buffer1, 2, RTCC_MPT_LM, sv_LM.sv, true);
		AGCREFSMMATUpdate(buffer3, REFSMMAT, false);
		sprintf(uplinkdata, "%s%s%s%s", buffer1, clockupdate, buffer2, buffer3);
		A14GiveUplink(upString, upDesc, uplinkdata, "LM state vector, V66, clock, abort constants, LS REFSMMAT");
	}
	break;
	case 37: //Undock/sep 104:27:31. Flight plan note 5: radial, CSM below, sep immediate
	{
		AP11ManPADOpt opt;
		EphemerisData sv;
		AP11MNV manpad;
		AP12SEPPAD *form = (AP12SEPPAD *)pad;

		// LVLH +Z points at the Moon. CSM below means the 1 fps sep is toward the Moon.
		sv = StateVectorCalcEphem(calcParams.src);
		calcParams.SEP = A14_UNDOCK;
		opt.TIG = A14_UNDOCK;
		opt.dV_LVLH = _V(0, 0, 1.0) * 0.3048;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSPLUS4;
		opt.HeadsUp = false;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.RV_MCC = sv;
		opt.WeightsTable = A14LiveWeights(this, calcParams.src);
		AP11ManeuverPAD(opt, manpad);
		form->t_Undock = A14_UNDOCK;
		form->t_Separation = A14_UNDOCK;
		form->Att_Undock = manpad.Att;
	}
	break;
	case 38: //Apollo 12 LM DOI. Apollo 14 DOI is the docked CSM SPS burn
	{
		scrubbed = true;
		A14Msg(upMessage, "No A14 LM DOI. CSM DOI is 86:56:57; CSM circ is 105:46:48");
	}
	break;
	case 40: //No TEI-1 on Apollo 14
	case 46:
	case 47:
	case 48:
	case 49:
		A14Msg(upMessage, "A14 has no TEI pad for this update");
		return true;
	case 41: //TEI-4  GETI 91:15 GETIL 141:47, assumes LOI and no DOI
	case 42: //TEI-5  GETI 92:30 GETIL 166:14, assumes DOI
	case 43: //TEI-12 GETI 105:54 GETIL 166:24, assumes no circ
	case 44: //TEI-19 GETI 119:38 GETIL 191:13, assumes circ and no plane change
	case 45: //TEI-34 preliminary GETI 149:15 GETIL 216:40, assumes plane change
	case 50: //TEI-34 nominal, Table I-5 149:14:50 to EOM, GETIL 216:40
	case 51: //TEI-35 GETI 151:14 GETIL 216:16
	{
		AP11ManPADOpt opt;
		double AbortGuess, GETI, GETIL;
		SV sv0, sv1;
		char manname[16];
		EphemerisData sv_e;
		AP11MNV *form = (AP11MNV *)pad;

		sv0 = StateVectorCalc(calcParams.src);
		GMGMED("F79,0;");

		if (fcn == 41 || fcn == 42)
		{
			sv1 = ExecuteManeuver(sv0, TimeofIgnition, DeltaV_LVLH, GetDockedVesselMass(calcParams.src), RTCC_ENGINETYPE_CSMSPS);
		}
		else
		{
			sv1 = sv0;
		}
		sv_e = ConvertSVtoEphemData(sv1);
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);

		if (fcn == 41)
		{
			sprintf(manname, "TEI-4");
			GETI = A14SS(91, 15, 0.0);
			GETIL = A14SS(141, 47, 0.0);
		}
		else if (fcn == 42)
		{
			sprintf(manname, "TEI-5");
			GETI = A14SS(92, 30, 0.0);
			GETIL = A14SS(166, 14, 0.0);
		}
		else if (fcn == 43)
		{
			sprintf(manname, "TEI-12");
			GETI = A14SS(105, 54, 0.0);
			GETIL = A14SS(166, 24, 0.0);
		}
		else if (fcn == 44)
		{
			sprintf(manname, "TEI-19");
			GETI = A14SS(119, 38, 0.0);
			GETIL = A14SS(191, 13, 0.0);
		}
		else if (fcn == 45)
		{
			sprintf(manname, "TEI-34");
			GETI = A14SS(149, 15, 0.0);
			GETIL = A14SS(216, 40, 0.0);
		}
		else if (fcn == 50)
		{
			sprintf(manname, "TEI-34");
			GETI = A14_TEI;
			GETIL = A14SS(216, 40, 0.0);
		}
		else
		{
			sprintf(manname, "TEI-35");
			GETI = A14SS(151, 14, 0.0);
			GETIL = A14SS(216, 16, 0.0);
		}

		AbortGuess = GETI;
		opt.WeightsTable = A14LiveWeights(this, calcParams.src);
		// TEI-4 and TEI-5 are solved after LOI. That burn is already in sv1.
		if (fcn == 41 || fcn == 42)
		{
			opt.WeightsTable.CSMWeight = sv1.mass;
			A14FinishWeights(opt.WeightsTable);
		}
		VEHDATABUF.csmmass = opt.WeightsTable.CSMWeight;
		VEHDATABUF.lmascmass = opt.WeightsTable.LMAscWeight;
		VEHDATABUF.lmdscmass = opt.WeightsTable.LMDscWeight;
		VEHDATABUF.sv = sv_e;
		if (opt.WeightsTable.CC[RTCC_CONFIG_A] && opt.WeightsTable.CC[RTCC_CONFIG_D])
			VEHDATABUF.config = "CL";
		else if (opt.WeightsTable.CC[RTCC_CONFIG_A])
			VEHDATABUF.config = "CA";
		else
			VEHDATABUF.config = "C";

		med_f75_f77.T_0_min = AbortGuess - 3600.0;
		med_f77.T_max = AbortGuess + 3600.0;
		med_f75_f77.T_Z = GETIL;
		med_f77.Site = (fcn == 50) ? "EOM" : "MPL";
		DetermineRTESite(med_f77.Site);

		PZREAP.RTEVectorTime = GMTfromGET(med_f75_f77.T_V) / 3600.0;
		PZREAP.RTET0Min = GMTfromGET(med_f75_f77.T_0_min) / 3600.0;
		PZREAP.RTET0Max = GMTfromGET(med_f77.T_max) / 3600.0;
		PZREAP.RTETimeOfLanding = GMTfromGET(med_f75_f77.T_Z) / 3600.0;
		PZREAP.RTEPTPMissDistance = med_f77.MissDistance;
		PMMREAST(77, &sv_e);

		med_f80.ASTCode = PZREAP.AbortScanTableData[0].ASTCode;
		med_f80.ManeuverCode = "CSUX";
		med_f80.REFSMMAT = "TEI";
		med_f80.HeadsUp = false;
		med_f80.NumQuads = 4;
		med_f80.UllageDT = (fcn == 50) ? 12.0 : 11.0;
		PMMREDIG(false);

		if (fcn == 50)
		{
			GMGMED("G11,CSM,REP;");
			GMGMED("G00,CSM,DOD,CSM,LCV;");
			opt.REFSMMAT = EZJGMTX1.data[RTCC_REFSMMAT_TYPE_LCV - 1].REFSMMAT;
		}

		opt.TIG = PZREAP.RTEDTable[0].GETI;
		opt.dV_LVLH = PZREAP.RTEDTable[0].DV_XDV;
		opt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		opt.HeadsUp = false;
		opt.RV_MCC = sv_e;
		AP11ManeuverPAD(opt, *form);
		A14ShowCSMWeight(form, opt.WeightsTable);

		RMMYNIInputTable entin;
		RMMYNIOutputTable entout;
		EphemerisData2 sv_EI_ECT;
		ELVCNV(PZREAP.AbortScanTableData[0].sv_EI, 0, 1, sv_EI_ECT);
		entin.R0 = sv_EI_ECT.R;
		entin.V0 = sv_EI_ECT.V;
		entin.GMT0 = sv_EI_ECT.GMT;
		entin.lat_T = PZREAP.RTEDTable[0].lat_imp_tgt;
		entin.lng_T = PZREAP.RTEDTable[0].lng_imp_tgt;
		entin.KSWCH = 3;
		RMMYNI(entin, entout);

		sprintf(form->purpose, manname);
		form->lat = PZREAP.RTEDTable[0].lat_imp_tgt * DEG;
		form->lng = PZREAP.RTEDTable[0].lng_imp_tgt * DEG;
		form->RTGO = entout.R_EMS / 1852.0;
		form->VI0 = entout.V_EMS / 0.3048;
		form->GET05G = GETfromGMT(entout.t_05g);
		form->type = 2;
		if (fcn == 41) sprintf(form->remarks, "Assumes LOI, no DOI");
		else if (fcn == 42) sprintf(form->remarks, "Assumes DOI");
		else if (fcn == 43) sprintf(form->remarks, "Assumes no circ");
		else if (fcn == 44) sprintf(form->remarks, "Assumes circ, no PC");
		else if (fcn == 45) sprintf(form->remarks, "Preliminary, assumes PC");
		else if (fcn == 50) sprintf(form->remarks, "Ullage: 4 jet, 12 sec; EOM");
		else sprintf(form->remarks, "Block data, MPL");
		A14LMWeightRemark(form);

		if (fcn != 51)
		{
			SplashLatitude = PZREAP.RTEDTable[0].lat_imp_tgt;
			SplashLongitude = PZREAP.RTEDTable[0].lng_imp_tgt;
			calcParams.TEI = PZREAP.RTEDTable[0].GETI;
			calcParams.EI = PZREAP.RTEDTable[0].ReentryPET;
			if (calcParams.TEI <= 0.0) calcParams.TEI = GETI;
		}

		if (fcn == 50)
		{
			char buffer1[1000], buffer2[1000], buffer3[1000];
			TimeofIgnition = PZREAP.RTEDTable[0].GETI;
			DeltaV_LVLH = PZREAP.RTEDTable[0].DV_XDV;
			AGCStateVectorUpdate(buffer1, sv1, true, true);
			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);
			AGCDesiredREFSMMATUpdate(buffer3, opt.REFSMMAT);
			sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load, TEI REFSMMAT");
		}
	}
	break;
	case 61: //Mosting A, rev 2. Table I-10
	case 62: //H-3, rev 3
	case 63: //Landing site, rev 17
	case 64: //Rev 18: RP-2, 12-1, Dollond E, FHI
	case 65: //Rev 29: RP-4, Ansgarius N, DE-2, Encke E
	case 67: //14-1 through 14-4. The rev cell is printed on the 14-1 row (12, 13, 15)
	case 68: //Rev 15: RP-3, RP-5, Daguerre 66
	{
		LMARKTRKPADOpt opt;
		EphemerisData sv0;
		double GET_SV;
		AP11LMARKTRKPAD *form = (AP11LMARKTRKPAD *)pad;

		sv0 = StateVectorCalcEphem(calcParams.src);
		opt.sv0 = sv0;
		GET_SV = GETfromGMT(sv0.GMT);
		form->type = 0;

		if (fcn == 61)
		{
			A14Landmark(opt, form, 0, "MOSTING A", -3.250, -5.283, 0.0, GET_SV);
			opt.entries = 1;
		}
		else if (fcn == 62)
		{
			A14Landmark(opt, form, 0, "H-3", -3.691, -7.542, 0.0, GET_SV);
			opt.entries = 1;
		}
		else if (fcn == 63)
		{
			A14Landmark(opt, form, 0, "LDG SITE", -3.672, -17.463, -0.76, GET_SV);
			opt.entries = 1;
		}
		else if (fcn == 64)
		{
			A14Landmark(opt, form, 0, "RP-2", -0.283, 141.250, 0.0, GET_SV);
			A14Landmark(opt, form, 1, "12-1", -5.736, 112.309, 0.0, GET_SV);
			A14Landmark(opt, form, 2, "DOLLOND E", -10.433, 15.733, 0.0, GET_SV);
			A14Landmark(opt, form, 3, "FHI", -3.246, -17.317, 0.0, GET_SV);
			opt.entries = 4;
		}
		else if (fcn == 65)
		{
			A14Landmark(opt, form, 0, "RP-4", -5.850, 120.250, 0.0, GET_SV);
			A14Landmark(opt, form, 1, "ANSGARIUS N", -11.633, 81.067, 0.0, GET_SV);
			A14Landmark(opt, form, 2, "DE-2", -9.250, 19.592, 0.0, GET_SV);
			A14Landmark(opt, form, 3, "ENCKE E", 0.283, -40.300, 0.0, GET_SV);
			opt.entries = 4;
		}
		else if (fcn == 67)
		{
			A14Landmark(opt, form, 0, "14-1", -4.046, -15.600, -0.44, GET_SV);
			A14Landmark(opt, form, 1, "14-2", -3.610, -15.317, -0.15, GET_SV);
			A14Landmark(opt, form, 2, "14-3", -3.919, -15.139, -0.38, GET_SV);
			A14Landmark(opt, form, 3, "14-4", -3.470, -14.890, -0.87, GET_SV);
			opt.entries = 4;
		}
		else
		{
			A14Landmark(opt, form, 0, "RP-3", -3.533, 131.700, 0.0, GET_SV);
			A14Landmark(opt, form, 1, "RP-5", -10.567, 99.400, 0.0, GET_SV);
			A14Landmark(opt, form, 2, "DAGUERRE 66", -11.717, 33.200, 0.0, GET_SV);
			opt.entries = 3;
		}
		LandmarkTrackingPAD(opt, *form);
	}
	break;
	case 70: //PDI pad. No LM DOI precedes PDI on Apollo 14
	{
		AP11PDIPAD *form = (AP11PDIPAD *)pad;
		PDIPADOpt opt;
		VehicleDataBlock sv;

		PLAWDTOutput lmWt = A14LiveWeights(this, calcParams.tgt);
		double lmMass = lmWt.LMAscWeight + lmWt.LMDscWeight;

		sv = StateVectorCalcDataBlock(calcParams.tgt);
		// PDI has no weight line. The ignition mass is the LM, ascent stage after staging.
		if (lmMass > 0.0)
			sv.Weight = lmMass;
		opt.direct = true;
		opt.HeadsUp = true;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
		opt.R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		opt.sv0 = sv;
		opt.t_land = CZTDTGTU.GETTD;
		PDI_PAD(opt, *form);
	}
	break;
	case 79:
		A14Msg(upMessage, "Landing confirmed");
		break;
	case 93: //Plane-change evaluation. Liftoff seed is Table I-6 ascent GETI
	{
		SV sv_CSM, sv_Liftoff;
		VECTOR3 R_LS;
		double TIG_nom, GETbase, MJD_TIG_nom, dt1, LmkRange;

		sv_CSM = StateVectorCalc(calcParams.src);
		GETbase = CalcGETBase();
		calcParams.LunarLiftoff = A14_LIFTOFF;
		TIG_nom = calcParams.LunarLiftoff;
		MJD_TIG_nom = OrbMech::MJDfromGET(TIG_nom, GETbase);
		sv_Liftoff = coast(sv_CSM, (MJD_TIG_nom - sv_CSM.MJD) * 24.0 * 3600.0);
		R_LS = OrbMech::r_from_latlong(BZLAND.lat[RTCC_LMPOS_BEST], BZLAND.lng[RTCC_LMPOS_BEST], BZLAND.rad[RTCC_LMPOS_BEST]);
		dt1 = OrbMech::findelev_gs(SystemParameters.AGCEpoch, SystemParameters.MAT_J2000_BRCS, sv_Liftoff.R, sv_Liftoff.V, R_LS, MJD_TIG_nom, 180.0 * RAD, sv_Liftoff.gravref, LmkRange);
		if (abs(LmkRange) < 8.0 * 1852.0)
		{
			A14Msg(upMessage, "Plane Change has been scrubbed");
			scrubbed = true;
		}
	}
	break;
	case 95:
		scrubbed = true;
		A14Msg(upMessage, "No Apollo 14 PC-2 in the flight plan");
		break;
	case 110: //CSM sep 146:28:31. Flight plan note 6 and the LM-impact note: 1 fps retrograde
	{
		AP11ManPADOpt opt;
		EphemerisData sv;
		VECTOR3 dV_LVLH;
		int hh, mm;
		double ss;
		char buffer1[1000];
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalcEphem(calcParams.src);
		calcParams.SEP = A14_SEP;
		// LVLH +X is posigrade, so retrograde is -X. The note fires +Z thrusters in that attitude.
		dV_LVLH = _V(-1.0, 0, 0) * 0.3048;
		opt.TIG = A14_SEP;
		opt.dV_LVLH = dV_LVLH;
		opt.enginetype = RTCC_ENGINETYPE_CSMRCSPLUS4;
		opt.HeadsUp = false;
		opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
		opt.RV_MCC = sv;
		opt.WeightsTable = A14LiveWeights(this, calcParams.src);
		AP11ManeuverPAD(opt, *form);
		A14ShowCSMWeight(form, opt.WeightsTable);
		sprintf(form->purpose, "SEP");
		OrbMech::SStoHHMMSS(A14_SEP - 5.0 * 60.0, hh, mm, ss);
		sprintf(form->remarks, "1 fps retrograde, +Z thrusters. Jettison radial, %d:%02d:%02.0lf", hh, mm, ss);
		A14LMWeightRemark(form);
		form->type = 2;
		TimeofIgnition = A14_SEP;
		DeltaV_LVLH = dV_LVLH;

		AGCStateVectorUpdate(buffer1, RTCC_MPT_CSM, RTCC_MPT_CSM, sv, true);
		sprintf(uplinkdata, "%s", buffer1);
		A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
	}
	break;
	case 111: //LM deorbit. Note: 180 fps retrograde, 36.5 fps north, TIG 147:52:58.9
	case 112: //CSM P76 for that burn
	{
		// LVLH +X is posigrade and +Y is north on this westward lunar orbit.
		DeltaV_LVLH = _V(-180.0, 36.5, 0.0) * 0.3048;
		TimeofIgnition = A14_DEORBIT;

		if (fcn == 111)
		{
			AP11LMManPADOpt opt;
			SV sv;
			char buffer1[1000];
			char buffer2[1000];
			AP11LMMNV *form = (AP11LMMNV *)pad;

			sv = StateVectorCalc(calcParams.tgt);
			opt.TIG = A14_DEORBIT;
			opt.dV_LVLH = DeltaV_LVLH;
			opt.enginetype = RTCC_ENGINETYPE_LMRCSPLUS4;
			opt.HeadsUp = false;
			opt.REFSMMAT = GetREFSMMATfromAGC(&mcc->lm->agc.vagc, false);
			opt.RV_MCC = ConvertSVtoEphemData(sv);
			// Config "L" is the full LM. Config "A" is the ascent stage after staging.
			opt.WeightsTable = A14LiveWeights(this, calcParams.tgt);
			AP11LMManeuverPAD(opt, *form);
			form->LMWeight = A14KgToLb(opt.WeightsTable.LMAscWeight + opt.WeightsTable.LMDscWeight);
			form->CSMWeight = opt.WeightsTable.CC[RTCC_CONFIG_C] ? A14KgToLb(opt.WeightsTable.CSMWeight) : 0.0;
			sprintf(form->purpose, "LM DEORBIT");
			sprintf(form->remarks, "180 retrograde, 36.5 north. Impact 3.5S 19.27W. P99. LM WT %.0f.", form->LMWeight);
			AGCStateVectorUpdate(buffer1, sv, false);
			LGCExternalDeltaVUpdate(buffer2, A14_DEORBIT, DeltaV_LVLH);
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			A14GiveUplink(upString, upDesc, uplinkdata, "LM state vector, Target load");
		}
		else
		{
			AP11P76PAD *form = (AP11P76PAD *)pad;

			form->entries = 1;
			sprintf(form->purpose[0], "CSM P76");
			form->TIG[0] = A14_DEORBIT;
			form->DV[0] = DeltaV_LVLH / 0.3048;
		}
	}
	break;
	case 120:
	case 121:
	case 122:
	case 123:
		// The impact note calls P99 with V30E. Those octal loads embed one planning
		// state vector, so they are not uplinked on the live trajectory.
		scrubbed = true;
		A14Msg(upMessage, "A14 deorbit is P99 (V30E). Planning octals are not uplinked");
		break;
	case 130:
	case 131:
	case 602:
	case 603:
	case 604:
	case 605:
	case 607:
	case 608:
		scrubbed = true;
		A14Msg(upMessage, "A14 photography update scrubbed: no attitude in the flight plan");
		break;
	case 210: //MCC-5 166:14:50
	case 211:
	case 212: //MCC-6 194:26:59
	case 213: //MCC-7 decision
	case 214: //MCC-7 213:26:59
	case 300:
	{
		EntryOpt entopt;
		EntryResults res;
		double MCCtime;
		char manname[8];
		SV sv;
		bool eom;

		sv = StateVectorCalc(calcParams.src);
		eom = (fcn == 210 || fcn == 211 || fcn == 212 || fcn == 213 || fcn == 214);
		if (fcn == 210)
		{
			MCCtime = A14_MCC5;
			sprintf(manname, "MCC-5");
		}
		else if (fcn == 211 || fcn == 212)
		{
			MCCtime = A14_MCC6;
			sprintf(manname, "MCC-6");
		}
		else if (fcn == 213 || fcn == 214)
		{
			MCCtime = A14_MCC7;
			sprintf(manname, "MCC-7");
		}
		else if (fcn == 300)
		{
			MCCtime = calcParams.TEI + 5.0 * 3600.0;
			sprintf(manname, "MCC");
		}
		else
		{
			MCCtime = OrbMech::GETfromMJD(sv.MJD, CalcGETBase());
			sprintf(manname, "MCC");
		}

		entopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
		entopt.RV_MCC = sv;
		entopt.TIGguess = MCCtime;
		entopt.vessel = calcParams.src;
		entopt.csmlmdocked = A14LMAttached(A14LiveWeights(this, calcParams.src));
		entopt.type = 3;
		if (eom)
		{
			// Nominal transearth MCCs target the EOM meridian, not MPL.
			entopt.entrylongmanual = true;
			entopt.lng = A14_EOM_LNG;
		}
		else
		{
			entopt.entrylongmanual = false;
			entopt.ATPLine = 0;
		}
		EntryTargeting(entopt, res);

		if (!eom)
		{
			entopt.lng = EntryCalculations::MPL2(res.latitude);
			if (MCCtime < calcParams.EI - 24.0 * 3600.0 && abs(res.longitude - entopt.lng) > 2.0 * RAD)
			{
				entopt.type = 1;
				entopt.t_Z = res.GET400K;
				EntryTargeting(entopt, res);
			}
		}

		// 71-FM54-41: the MCC-5 execution threshold is 1 fps. MCC-6/MCC-7 keep the
		// Apollo 11 mission-rule split already used for the H missions.
		if (fcn == 210 || MCCtime > res.GET400K - 50.0 * 3600.0)
		{
			if (length(res.dV_LVLH) < 1.0 * 0.3048) scrubbed = true;
		}
		else
		{
			if (length(res.dV_LVLH) < 2.0 * 0.3048) scrubbed = true;
		}

		if (fcn != 213)
		{
			AP11ManPADOpt opt;
			MATRIX3 REFSMMAT;
			AP11MNV *form = (AP11MNV *)pad;

			if (fcn == 214)
			{
				REFSMMATOpt refsopt;
				refsopt.REFSMMATopt = 3;
				refsopt.vessel = calcParams.src;
				refsopt.useSV = true;
				refsopt.RV_MCC = res.sv_postburn;
				REFSMMAT = REFSMMATCalc(&refsopt);
			}
			else
			{
				REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			}

			if (scrubbed)
			{
				EntryUpdateCalc(ConvertSVtoEphemData(sv), PZREAP.RRBIAS, true, res);
				res.dV_LVLH = _V(0, 0, 0);
				res.P30TIG = entopt.TIGguess;
			}
			else
			{
				opt.WeightsTable = A14LiveWeights(this, calcParams.src);
				opt.TIG = res.P30TIG;
				opt.dV_LVLH = res.dV_LVLH;
				opt.enginetype = mcc->mcc_calcs.SPSRCSDecision(SPS_THRUST / opt.WeightsTable.ConfigWeight, res.dV_LVLH);
				opt.HeadsUp = false;
				opt.REFSMMAT = REFSMMAT;
				opt.RV_MCC = ConvertSVtoEphemData(sv);
				AP11ManeuverPAD(opt, *form);
				A14ShowCSMWeight(form, opt.WeightsTable);
				A14LMWeightRemark(form);
				sprintf(form->purpose, manname);
				form->lat = res.latitude * DEG;
				form->lng = res.longitude * DEG;
				form->RTGO = res.RTGO;
				form->VI0 = res.VIO / 0.3048;
				form->GET05G = res.GET05G;
			}

			if (scrubbed && (fcn == 210 || fcn == 212))
			{
				char buffer1[1000];
				char buffer2[1000];
				A14Msg(upMessage, fcn == 210 ? "MCC-5 has been scrubbed" : "MCC-6 has been scrubbed");
				AGCStateVectorUpdate(buffer1, sv, true, true);
				CMCEntryUpdate(buffer2, res.latitude, res.longitude);
				sprintf(uplinkdata, "%s%s", buffer1, buffer2);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Entry target");
			}
			else if (scrubbed && fcn == 214)
			{
				char buffer1[1000];
				char buffer2[1000];
				char buffer3[1000];
				A14Msg(upMessage, "MCC-7 has been scrubbed");
				AGCStateVectorUpdate(buffer1, sv, true, true);
				CMCEntryUpdate(buffer2, res.latitude, res.longitude);
				AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);
				sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Entry target, Entry REFSMMAT");
			}
			else if (!scrubbed && (fcn == 210 || fcn == 212 || fcn == 300))
			{
				char buffer1[1000];
				char buffer2[1000];
				AGCStateVectorUpdate(buffer1, sv, true, true);
				CMCRetrofireExternalDeltaVUpdate(buffer2, res.latitude, res.longitude, res.P30TIG, res.dV_LVLH);
				sprintf(uplinkdata, "%s%s", buffer1, buffer2);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load");
			}
			else if (!scrubbed && fcn == 214)
			{
				char buffer1[1000];
				char buffer2[1000];
				char buffer3[1000];
				AGCStateVectorUpdate(buffer1, sv, true, true);
				CMCRetrofireExternalDeltaVUpdate(buffer2, res.latitude, res.longitude, res.P30TIG, res.dV_LVLH);
				AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);
				sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
				A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load, Entry REFSMMAT");
			}
		}
		else if (scrubbed)
		{
			A14Msg(upMessage, "MCC-7 has been scrubbed");
		}
		else
		{
			A14Msg(upMessage, "MCC-7 will be executed");
		}

		DeltaV_LVLH = res.dV_LVLH;
		TimeofIgnition = res.P30TIG;
		SplashLatitude = res.latitude;
		SplashLongitude = res.longitude;
		calcParams.SVSTORE1 = res.sv_postburn;
		calcParams.EI = res.GET400K;
	}
	break;
	case 216:
	case 217:
	case 218:
	{
		bool result = CalculationMTP_H1(fcn, pad, upString, upDesc, upMessage);
		if (pad != NULL)
		{
			AP11ENT *form = (AP11ENT *)pad;
			sprintf(form->Area[0], "EOM");
		}
		return result;
	}
	case 500:
		break;
	case 501:
		A14Msg(upMessage, "GET sync if error exceeds 1 min. Flown -40 min was the launch hold.");
		break;
	case 502: //Circ 105:46:48. Table I-5 HP 56.04 nm, 4-jet ullage 11 s
	{
		AP11ManPADOpt manopt;
		SV sv, sv_tig;
		VECTOR3 dV_iner, dV_LVLH;
		MATRIX3 Q_Xx;
		double P30TIG, dt, r_peri;
		PLAWDTOutput WeightsTable;
		char buffer1[1000];
		char buffer2[1000];
		AP11MNV *form = (AP11MNV *)pad;

		sv = StateVectorCalc(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		dt = A14_CIRC - OrbMech::GETfromMJD(sv.MJD, CalcGETBase());
		sv_tig = coast(sv, dt);
		r_peri = BZLAND.rad[RTCC_LMPOS_BEST] + 56.04 * 1852.0;
		dV_iner = OrbMech::AdjustPeriapsis(sv_tig.R, sv_tig.V, OrbMech::mu_Moon, r_peri);
		Q_Xx = OrbMech::LVLH_Matrix(sv_tig.R, sv_tig.V);
		dV_LVLH = mul(Q_Xx, dV_iner);

		if (length(dV_LVLH) < 1.0)
		{
			scrubbed = true;
			A14Msg(upMessage, "A14 circ 105:46:48 GET, HP 56.04 nm; targeting failed");
			AGCStateVectorUpdate(buffer1, sv, true, true);
			sprintf(uplinkdata, "%s", buffer1);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66");
		}
		else
		{
			PoweredFlightProcessor(sv, A14_CIRC, RTCC_ENGINETYPE_CSMSPS, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, dV_LVLH, true, P30TIG, dV_LVLH);
			manopt.TIG = P30TIG;
			manopt.dV_LVLH = dV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = false;
			manopt.REFSMMAT = GetREFSMMATfromAGC(&mcc->cm->agc.vagc, true);
			manopt.RV_MCC = ConvertSVtoEphemData(sv);
			manopt.WeightsTable = WeightsTable;
			AP11ManeuverPAD(manopt, *form);
			A14ShowCSMWeight(form, WeightsTable);
			sprintf(form->purpose, "CIRC");
			sprintf(form->remarks, "Ullage: 4 jet, 11 seconds");
			A14LMWeightRemark(form);
			TimeofIgnition = P30TIG;
			DeltaV_LVLH = dV_LVLH;
			AGCStateVectorUpdate(buffer1, sv, true, true);
			CMCExternalDeltaVUpdate(buffer2, P30TIG, dV_LVLH);
			sprintf(uplinkdata, "%s%s", buffer1, buffer2);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, V66, Target load");
		}
	}
	break;
	case 73: //Lunar surface card. Table I-6 does not give the Apollo 12 T2/T3 times.
		scrubbed = true;
		A14Msg(upMessage, "A14 surface card skipped. Table I-6 ascent 142:24:29, TPI 143:09:40; no T2/T3");
		break;
	case 85: //Liftoff times. Only the Table I-6 nominal is printed.
	case 86:
	{
		LIFTOFFTIMES *form = (LIFTOFFTIMES *)pad;

		form->entries = 1;
		form->startdigit = 1;
		form->TIG[0] = A14_LIFTOFF;
		A14Msg(upMessage, "Nominal ascent only, Table I-6 142:24:29. Other rev times are not tabulated.");
	}
	break;
	case 94: //PC-1. Table I-5 planned TIG 118:09:40. The processor solves the burn.
	{
		SV sv;
		double GET_SV;
		AP11ManPADOpt manopt;
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		char buffer1[1000], buffer2[1000], buffer3[1000];
		AP11MNV *form = (AP11MNV *)pad;

		PLAWDTOutput WeightsTable;

		sv = StateVectorCalc(calcParams.src);
		WeightsTable = A14LiveWeights(this, calcParams.src);
		GET_SV = OrbMech::GETfromMJD(sv.MJD, CalcGETBase());
		calcParams.LunarLiftoff = A14_LIFTOFF;
		// One hour of coast before the search. Not a second flight-plan time.
		med_k16.GETTH1 = GET_SV + 3600.0;
		// Plane epoch is liftoff, Table I-6. Planned TIG remains 118:09:40.
		med_k16.GETTH2 = med_k16.GETTH3 = med_k16.GETTH4 = A14_LIFTOFF;
		med_k16.Mode = 7;
		med_k16.Sequence = 1;
		med_k16.Vehicle = RTCC_MPT_CSM;
		GZGENCSN.LDPPAzimuth = 0.0;

		if (LunarDescentPlanningProcessor(ConvertSVtoEphemData(sv), 0.0) != 0)
		{
			scrubbed = true;
			A14Msg(upMessage, "A14 PC-1 skipped: plane-change targeting failed");
		}
		else
		{
			PoweredFlightProcessor(sv, PZLDPDIS.GETIG[0], RTCC_ENGINETYPE_CSMSPS, WeightsTable.LMAscWeight + WeightsTable.LMDscWeight, PZLDPDIS.DVVector[0] * 0.3048, true, TimeofIgnition, DeltaV_LVLH);
			refsopt.dV_LVLH = DeltaV_LVLH;
			refsopt.HeadsUp = true;
			refsopt.REFSMMATTime = TimeofIgnition;
			refsopt.REFSMMATopt = 0;
			refsopt.vessel = calcParams.src;
			refsopt.vesseltype = 0;
			REFSMMAT = REFSMMATCalc(&refsopt);

			manopt.TIG = TimeofIgnition;
			manopt.dV_LVLH = DeltaV_LVLH;
			manopt.enginetype = RTCC_ENGINETYPE_CSMSPS;
			manopt.HeadsUp = true;
			manopt.REFSMMAT = REFSMMAT;
			manopt.UllageDT = 0.0;
			manopt.RV_MCC = ConvertSVtoEphemData(sv);
			manopt.WeightsTable = WeightsTable;
			AP11ManeuverPAD(manopt, *form);
			A14ShowCSMWeight(form, manopt.WeightsTable);
			sprintf(form->purpose, "PC-1");
			// Table I-5: 118:09:40, 18.4 s, 360.7 fps, HA 61.71 HP 57.41. Ullage column is not isolated.
			sprintf(form->remarks, "Table I-5 planned 118:09:40. Ullage not stated, none applied");
			A14LMWeightRemark(form);

			AGCStateVectorUpdate(buffer1, sv, true);
			CMCExternalDeltaVUpdate(buffer2, TimeofIgnition, DeltaV_LVLH);
			AGCDesiredREFSMMATUpdate(buffer3, REFSMMAT);
			sprintf(uplinkdata, "%s%s%s", buffer1, buffer2, buffer3);
			A14GiveUplink(upString, upDesc, uplinkdata, "CSM state vector, Target load, Plane Change REFSMMAT");
		}
	}
	break;
	case 100: //Ascent targeting. Table I-6 times only; H/V components are not in the table.
	{
		REFSMMATOpt refsopt;
		MATRIX3 REFSMMAT;
		SV sv_LM;
		char buffer1[64];
		char buffer2[1000];

		calcParams.LunarLiftoff = A14_LIFTOFF;
		calcParams.TPI = A14_TPI;
		// Cleared so the Apollo 12 insertion split 5533.9 / 34.4 fps is not reused.
		DeltaV_LVLH = _V(0, 0, 0);

		refsopt.LSLat = BZLAND.lat[RTCC_LMPOS_BEST];
		refsopt.LSLng = BZLAND.lng[RTCC_LMPOS_BEST];
		refsopt.REFSMMATopt = 5;
		refsopt.REFSMMATTime = A14_LIFTOFF;
		refsopt.vessel = calcParams.src;
		REFSMMAT = REFSMMATCalc(&refsopt);
		EMGSTSTM(RTCC_MPT_LM, REFSMMAT, RTCC_REFSMMAT_TYPE_LLD, RTCCPresentTimeGMT());
		GMGMED("G00,LEM,LLD,CSM,LCV;");

		sv_LM = StateVectorCalc(calcParams.tgt);
		sprintf(buffer1, "V45E");
		AGCStateVectorUpdate(buffer2, sv_LM, false);
		sprintf(uplinkdata, "%s%s", buffer1, buffer2);
		A14GiveUplink(upString, upDesc, uplinkdata, "Reset surface flag, LM state vector");
		A14Msg(upMessage, "A14 ascent Table I-6: 142:24:29, TPI 143:09:40, 6053.4 fps. H/V split is not in the table.");
	}
	break;
	case 105: //Ascent pad. Table I-6 prints total DV, not the horizontal and vertical split.
		scrubbed = true;
		A14Msg(upMessage, "A14 ascent Table I-6: 142:24:29, 7:10.7, 6053.4 fps, HA 50.96 HP 9.14, ullage none. H/V not printed.");
		break;
	case 106: //No coelliptic CSI on the direct rendezvous.
		scrubbed = true;
		A14Msg(upMessage, "No A14 CSI. Direct rendezvous; Table I-6 TPI is 143:09:40.");
		break;
	case 29: //LOI from the Apollo 14 SFP
	case 30:
	{
		bool result = CalculationMTP_H1(fcn, pad, upString, upDesc, upMessage);
		if (pad != NULL)
		{
			AP11MNV *form = (AP11MNV *)pad;
			PLAWDTOutput wt = A14LiveWeights(this, calcParams.src);
			A14ShowCSMWeight(form, wt);
			// H1 writes "LM weight is" from a docked table. N47 is already the total.
			if (strstr(form->remarks, "LM weight") != NULL)
			{
				if (form->LMWeight > 1.0)
					sprintf(form->remarks, "Includes LM %.0f.", form->LMWeight);
				else
					form->remarks[0] = '\0';
			}
			else
				A14LMWeightRemark(form);
		}
		return result;
	}
	case 7: //CSM DAP. The two printed weights follow the live config, not fcn 7 versus 700.
	case 700:
	{
		AP10DAPDATA *form = (AP10DAPDATA *)pad;
		PLAWDTOutput wt = A14LiveWeights(this, calcParams.src);
		bool docked = A14LMAttached(wt);

		CSMDAPUpdate(calcParams.src, *form, docked, A14AscentOnly(wt));
		form->ThisVehicleWeight = A14KgToLb(wt.CSMWeight);
		form->OtherVehicleWeight = docked ? A14KgToLb(wt.LMAscWeight + wt.LMDscWeight) : 0.0;
	}
	break;
	case 9: //LM DAP with V42. LM weight is the LM. CSM weight is present only while docked.
	{
		LMACTDATA *form = (LMACTDATA *)pad;
		AP10DAPDATA dap;
		PLAWDTOutput wt = A14LiveWeights(this, calcParams.tgt);
		bool docked = wt.CC[RTCC_CONFIG_C];
		VECTOR3 lmn20, csmn20, V42angles;
		LEM *lem;

		if (calcParams.tgt == NULL)
			break;

		LMDAPUpdate(calcParams.tgt, dap, docked, A14AscentOnly(wt));
		lem = (LEM *)calcParams.tgt;
		csmn20.x = calcParams.src->imu.Gimbal.X;
		csmn20.y = calcParams.src->imu.Gimbal.Y;
		csmn20.z = calcParams.src->imu.Gimbal.Z;
		lmn20.x = lem->imu.Gimbal.X;
		lmn20.y = lem->imu.Gimbal.Y;
		lmn20.z = lem->imu.Gimbal.Z;
		V42angles = OrbMech::LMDockedFineAlignment(lmn20, csmn20);
		form->V42Angles.x = V42angles.x * DEG;
		form->V42Angles.y = V42angles.y * DEG;
		form->V42Angles.z = V42angles.z * DEG;
		form->CSMWeight = docked ? A14KgToLb(wt.CSMWeight) : 0.0;
		form->LMWeight = A14KgToLb(wt.LMAscWeight + wt.LMDscWeight);
		form->PitchTrim = dap.PitchTrim;
		form->RollTrim = dap.YawTrim;
	}
	break;
	default:
		switch (fcn)
		{
		case 1: //CSM state vector
		case 2: //LM state vector in the CSM
		case 5: //CSM state vector with V66
		case 10: //Liftoff initialization from the live launch azimuth
		case 15: //TLI evaluation from the LVDC timebase
		case 33: //LOI evaluation
		case 36: //AGS activation from the live clock
		case 60: //Rev 1 map from the trajectory
		case 66: //LM acquisition at the landed site
		case 71: //PDI abort TPI is sunrise minus 23 min, the H-mission rule
		case 74: //P22 acquisition at the landed site
		case 75: //LM state vector and RLS
		case 77: //DOI evaluation
		case 78: //PDI evaluation
		case 80: //Stay for T1
		case 81: //Stay for T2
		case 96: //Liftoff REFSMMAT at the Table I-6 liftoff already stored
		case 101: //Insertion state vectors
		case 102:
		case 107: //Liftoff evaluation
		case 140: //PTC quads from the live RCS load
		case 170: //No-PDI recycle. Abort offsets match update 35, not an Apollo 12 site.
		case 200: //TEI evaluation
		case 205: //Entry-interface evaluation
		case 600: //Map from the trajectory
		case 601:
			// None of these copy an Apollo 12 site, attitude, or pad number.
			return CalculationMTP_H1(fcn, pad, upString, upDesc, upMessage);
		default:
			scrubbed = true;
			if (upMessage != NULL)
			{
				sprintf(upMessage, "A14 update %d skipped: no flight-plan value; Apollo 12 data is not used", fcn);
			}
			break;
		}
		break;
	}

	return scrubbed;
}
