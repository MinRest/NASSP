/*
  L.5 staging contract catalog for reconciliation.

  Source of the expectations: the ODX12 L.5 staging report dated 2026-11-06.
  This header does not define those functions. Against-current-tree
  applicability is UNVERIFIED. No staging source was copied into this tree.

  ODX12 owns the merge. The symbols below are the existing staging API.
*/

#ifndef LIOH_L5_CONTRACT_H
#define LIOH_L5_CONTRACT_H

namespace nassp_lioh_review {

static const char* kRepoHead = "8ca3a48c748760177f04dc584dc7492178527bee";
static const char* kStagingReport = "L5_ODX12_STAGING_REPORT_b3d8";
static const char* kReportedStagingPath =
	"C:\\OrbiterDev\\patches\\nassp459_merge_prep\\lioh_finite_odx12_20261106\\";
static const char* kReportedMergePath = "C:\\OrbiterDev\\NASSP-459-merge";
static const char* kReportedOnTestSha256 =
	"68185EA1B0A0206176325A8AFCB445F2943A04F0898CBF53754A0F6E9769A9A4";
static const char* kReportedOffTestSha256 =
	"FC60C65C8CB5B9C862F593C458E75482984690A2899CA19A08E74AAD098B323C";

/* Reported by the staging harness. Not recomputed in this checkout. */
static const int kStagingHarnessReexecutedHere = 0;
static const int kAgainstCurrentTreeVerified = 0;
static const int kStagingSourcesCopied = 0;
static const int kStateApiDefaultOff = 1;
static const int kVesselSaveAirflowLinked = 0;
static const int kLmSecAssessEnabled = 0;
static const int kChecklistItemIdsBound = 0;
static const int kFprProposedRows = 154;
static const int kFprFixturesNotRun = 15;
static const int kContractCasesRunnableHere = 0;

/* Reported L.2 fresh capacities. This adapter does not apply them. */
static const double kReportedCmFreshManHours = 36.0;
static const double kReportedLmPrimFreshManHours = 41.0;
static const double kReportedLmSecManHours = 14.0;

struct ContractCase {
	const char* id;
	const char* symbol;
	const char* enable_off;
	const char* enable_on;
	int rem_may_change;
	int gen_may_change;
	const char* note;
};

struct SensingBit {
	const char* name;
	int alone_sufficient;
	int cm_conjunction;
	int lm_prim_conjunction;
	int mutually_exclusive_divert;
};

static const ContractCase kContractCases[] = {
	{ "STATE_API", "StateApiEnabled", "false", "true", 0, 0,
	  "Implementation is off by default." },
	{ "SAVE_API", "SaveApiEnabled", "false", "true", 0, 0,
	  "Save helper is off by default and is not vessel-linked." },
	{ "OBSERVE", "ObserveForAnimation", "readable",
	  "visuals follow the observation", 0, 0,
	  "Visuals only. Cover, cartridge, and twist stay distinct." },
	{ "WOULD", "WouldConsume", "false",
	  "true only when Present, remaining > 0, dt > 0, and load > 0", 0, 0,
	  "Predicate. It is not itself the capacity writer." },
	{ "CONSUME", "TryConsume", "false",
	  "drains remaining_man_hours and does not bump commit_generation", 1, 0,
	  "Cover reopen does not refresh capacity." },
	{ "ASSESS_CM", "AssessSwapComplete", "false",
	  "true only for divert to the slot, interlock pin withdrawn, manual bleed, element removed, fresh seated, cover locked, used stowed, and divert center",
	  0, 0, "CM twist is tracked for animation and is not an assess requirement." },
	{ "ASSESS_LM_PRIM", "AssessSwapComplete", "false",
	  "true only for SEL SEC, primary vent, element removed, fresh seated, twist locked, cover locked, used stowed, and SEL PRIM",
	  0, 0, "Twist is required on LM primary." },
	{ "ASSESS_LM_SEC", "AssessSwapComplete", "false", "false", 0, 0,
	  "LM secondary assess stays false. Do not enable it in this branch." },
	{ "SENSING", "ApplySensingObservation", "no-op",
	  "updates cover, cartridge, twist, and procedure_phase", 0, 0,
	  "Identity is untouched." },
	{ "COMMIT", "TryCommitSwap", "false",
	  "sets remaining_man_hours to the L.2 fresh capacity and increments commit_generation only when assess is true",
	  1, 1, "CM A/B 36 man-hours, LM primary 41. LM secondary does not commit." },
	{ "SERIAL", "Serialize/Deserialize", "helpers",
	  "LIOHV2 writes twist and LIOHV1 still reads", 0, 0,
	  "Not linked to vessel save or load." },
	{ "SCENARIO", "ParseScenarioLine", "missing",
	  "have_record only, no invented record", 0, 0,
	  "ShouldWrite and Format stay on the staging side." },
	{ "COVER_REOPEN", "none", "identity unchanged", "identity unchanged", 0, 0,
	  "Opening or closing the cover is not a capacity or generation writer." },
	{ "SEL_VENT", "none", "identity unchanged", "identity unchanged", 0, 0,
	  "Selector and vent motion are existing cockpit switches." },
	{ "BITS_ALONE", "none", "not sufficient", "not sufficient", 0, 0,
	  "attenuation_panel_open, ground_wire_connected, and lm_ascent_cover_cleared do not complete a swap. Checklist item ids stay unbound." },
	{ "A13", "none", "excluded", "excluded", 0, 0,
	  "No Slot::MAILBOX. Preserve the existing timed-swap adapter and A13STATE." },
	{ "FPR", "none", "154 proposed rows, 15 fixtures NOT_RUN",
	  "row completion is not cartridge replacement completion", 0, 0,
	  "The FPR package was not in this checkout and was not run." }
};

static const int kContractCaseCount =
	(int)(sizeof(kContractCases) / sizeof(kContractCases[0]));

static const SensingBit kSensingBits[] = {
	{ "cover_unlocked", 0, 0, 0, 0 },
	{ "cover_open", 0, 0, 0, 0 },
	{ "cover_locked", 0, 1, 1, 0 },
	{ "element_removed", 0, 1, 1, 0 },
	{ "fresh_installed", 0, 1, 1, 0 },
	{ "used_stowed", 0, 1, 1, 0 },
	{ "cartridge_twist_locked", 0, 0, 1, 0 },
	{ "divert_a", 0, 1, 0, 1 },
	{ "divert_b", 0, 1, 0, 1 },
	{ "divert_center", 0, 1, 0, 0 },
	{ "interlock_pin_withdrawn", 0, 1, 0, 0 },
	{ "manual_bleed_pressed", 0, 1, 0, 0 },
	{ "lm_sel_sec", 0, 0, 1, 0 },
	{ "lm_prim_vented", 0, 0, 1, 0 },
	{ "lm_sel_prim", 0, 0, 1, 0 },
	{ "attenuation_panel_open", 0, 0, 0, 0 },
	{ "ground_wire_connected", 0, 0, 0, 0 },
	{ "lm_ascent_cover_cleared", 0, 0, 0, 0 }
};

static const int kSensingBitCount =
	(int)(sizeof(kSensingBits) / sizeof(kSensingBits[0]));

}

#endif
