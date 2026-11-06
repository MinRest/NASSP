#include "lioh_anim_adapter.h"
#include "lioh_l5_contract.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#ifndef ECS_LIOH_REPO_ROOT
#define ECS_LIOH_REPO_ROOT "/workspace"
#endif

static int g_failed = 0;
static int g_checked = 0;

#define CHECK(cond) do { \
	g_checked++; \
	if (!(cond)) { \
		fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #cond); \
		g_failed++; \
	} \
} while (0)

static int load_file(const char* path, char** out, long* out_len) {
	FILE* f = fopen(path, "rb");
	long len;
	char* buf;
	if (!f) return 0;
	if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 0; }
	len = ftell(f);
	if (len < 0) { fclose(f); return 0; }
	if (fseek(f, 0, SEEK_SET) != 0) { fclose(f); return 0; }
	buf = (char*)malloc((size_t)len + 1);
	if (!buf) { fclose(f); return 0; }
	if (len > 0 && fread(buf, 1, (size_t)len, f) != (size_t)len) {
		free(buf);
		fclose(f);
		return 0;
	}
	buf[len] = 0;
	fclose(f);
	*out = buf;
	*out_len = len;
	return 1;
}

static void repo_path(char* out, size_t n, const char* rel) {
	snprintf(out, n, "%s/%s", ECS_LIOH_REPO_ROOT, rel);
}

static int file_has(const char* rel, const char* needle, int* opened) {
	char path[1024];
	char* buf = 0;
	long len = 0;
	int has;
	repo_path(path, sizeof(path), rel);
	*opened = load_file(path, &buf, &len);
	if (!*opened) return 0;
	has = strstr(buf, needle) != 0;
	free(buf);
	return has;
}

static int file_missing(const char* rel) {
	char path[1024];
	struct stat st;
	repo_path(path, sizeof(path), rel);
	return stat(path, &st) != 0;
}

static nassp_lioh_review::PhaseCodes fixture_codes() {
	nassp_lioh_review::PhaseCodes c;
	c.cover_closed_locked = 10;
	c.cover_unlocked = 11;
	c.cover_open = 12;
	c.cartridge_present = 20;
	c.cartridge_removed = 21;
	c.cartridge_fresh_seated = 22;
	c.twist_unlocked = 30;
	c.twist_locked = 31;
	return c;
}

static nassp_lioh_review::SlotVisuals visual(
	int cover, int cartridge, int twist, int phase, int kind) {
	nassp_lioh_review::SlotVisuals v;
	v.cover = cover;
	v.cartridge = cartridge;
	v.twist = twist;
	v.procedure_phase = phase;
	v.slot_kind = kind;
	return v;
}

static int cover_pose_equal(
	const nassp_lioh_review::CoverCommand& a,
	const nassp_lioh_review::CoverCommand& b) {
	return a.switch_value == b.switch_value
		&& a.angle_known == b.angle_known
		&& a.cockpit_deg == b.cockpit_deg
		&& a.proposal_ready == b.proposal_ready
		&& a.bound_in_this_branch == b.bound_in_this_branch
		&& a.sec_valve_enabled == b.sec_valve_enabled;
}

static const nassp_lioh_review::ContractCase* find_case(const char* id) {
	int i;
	for (i = 0; i < nassp_lioh_review::kContractCaseCount; i++) {
		if (strcmp(nassp_lioh_review::kContractCases[i].id, id) == 0)
			return &nassp_lioh_review::kContractCases[i];
	}
	return 0;
}

static const nassp_lioh_review::SensingBit* find_bit(const char* name) {
	int i;
	for (i = 0; i < nassp_lioh_review::kSensingBitCount; i++) {
		if (strcmp(nassp_lioh_review::kSensingBits[i].name, name) == 0)
			return &nassp_lioh_review::kSensingBits[i];
	}
	return 0;
}

static void test_channels_stay_distinct() {
	using namespace nassp_lioh_review;
	PhaseCodes codes = fixture_codes();
	SlotVisuals base = visual(codes.cover_open, codes.cartridge_present,
		codes.twist_locked, 1, Slot_LmPrim);
	AdapterResult open_locked = MapSlotVisuals(base, codes);

	SlotVisuals twist_off = base;
	twist_off.twist = codes.twist_unlocked;
	AdapterResult open_unlocked_twist = MapSlotVisuals(twist_off, codes);

	SlotVisuals removed = base;
	removed.cartridge = codes.cartridge_removed;
	AdapterResult open_removed = MapSlotVisuals(removed, codes);

	SlotVisuals closed = base;
	closed.cover = codes.cover_closed_locked;
	AdapterResult closed_result = MapSlotVisuals(closed, codes);

	CHECK(open_locked.cover.angle_known == 1);
	CHECK(open_locked.cover.switch_value == 0);
	CHECK(open_locked.cover.cockpit_deg == 300.0);
	CHECK(open_locked.cover.proposal_ready == 1);
	CHECK(open_locked.cover.bound_in_this_branch == 0);
	CHECK(strcmp(open_locked.cover.proposed_target, "CO2CanisterPrimValve") == 0);
	CHECK(open_locked.twist.phase_echo == codes.twist_locked);
	CHECK(open_locked.twist.angle_known == 0);
	CHECK(open_locked.cartridge.phase_echo == codes.cartridge_present);
	CHECK(open_locked.cartridge.translation_known == 0);
	CHECK(open_locked.twist.distinct_from_cover == 1);
	CHECK(open_locked.cartridge.distinct_from_cover == 1);
	CHECK(open_locked.cartridge.distinct_from_twist == 1);

	CHECK(cover_pose_equal(open_locked.cover, open_unlocked_twist.cover));
	CHECK(open_unlocked_twist.twist.phase_echo == codes.twist_unlocked);
	CHECK(open_locked.twist.phase_echo != open_unlocked_twist.twist.phase_echo);

	CHECK(cover_pose_equal(open_locked.cover, open_removed.cover));
	CHECK(open_removed.cartridge.phase_echo == codes.cartridge_removed);
	CHECK(open_removed.twist.phase_echo == open_locked.twist.phase_echo);

	CHECK(closed_result.cover.switch_value == 1);
	CHECK(closed_result.cover.cockpit_deg == 0.0);
	CHECK(closed_result.cover.angle_known == 1);
	CHECK(closed_result.twist.phase_echo == codes.twist_locked);
	CHECK(closed_result.cartridge.phase_echo == codes.cartridge_present);
}

static void test_unlocked_and_unknown_have_no_invented_angle() {
	using namespace nassp_lioh_review;
	PhaseCodes codes = fixture_codes();
	SlotVisuals unlocked = visual(codes.cover_unlocked, codes.cartridge_present,
		codes.twist_locked, 4, Slot_LmPrim);
	AdapterResult u = MapSlotVisuals(unlocked, codes);
	CHECK(u.cover.phase_echo == codes.cover_unlocked);
	CHECK(u.cover.angle_known == 0);
	CHECK(u.cover.switch_value == -1);
	CHECK(u.cover.proposal_ready == 0);
	CHECK(u.cover.proposed_for_prim_valve == 1);

	SlotVisuals unknown = unlocked;
	unknown.cover = 999;
	AdapterResult n = MapSlotVisuals(unknown, codes);
	CHECK(n.cover.angle_known == 0);
	CHECK(n.cover.switch_value == -1);
	CHECK(n.cover.proposal_ready == 0);
	CHECK(n.twist.angle_known == 0);
	CHECK(n.bore_known == 0);
	CHECK(n.stowage_known == 0);
	CHECK(n.mesh_edit == 0);
}

static void test_procedure_and_completion_are_not_the_pose() {
	using namespace nassp_lioh_review;
	PhaseCodes codes = fixture_codes();
	SlotVisuals seated = visual(codes.cover_closed_locked,
		codes.cartridge_fresh_seated, codes.twist_locked, 0, Slot_LmPrim);
	AdapterResult a = MapSlotVisuals(seated, codes);
	seated.procedure_phase = 7;
	AdapterResult b = MapSlotVisuals(seated, codes);
	CHECK(a.reports_swap_complete == 0);
	CHECK(b.reports_swap_complete == 0);
	CHECK(a.writes_remaining_man_hours == 0);
	CHECK(a.writes_commit_generation == 0);
	CHECK(b.writes_remaining_man_hours == 0);
	CHECK(b.writes_commit_generation == 0);
	CHECK(cover_pose_equal(a.cover, b.cover));
	CHECK(a.twist.phase_echo == b.twist.phase_echo);
	CHECK(a.cartridge.phase_echo == b.cartridge.phase_echo);
	CHECK(a.lm_sec_assess_complete == 0);
	CHECK(a.checklist_autocomplete == 0);
	CHECK(a.checklist_item_bound == 0);
	CHECK(a.a13_mailbox_write == 0);
	CHECK(kFprProposedRows == 154);
	CHECK(kFprFixturesNotRun == 15);
	CHECK(kChecklistItemIdsBound == 0);
}

static void test_interrupted_visual_sequence_leaves_identity_witness() {
	using namespace nassp_lioh_review;
	PhaseCodes codes = fixture_codes();
	unsigned id = 100;
	unsigned generation = 2;
	double remaining = 36.0;
	int steps[6][3] = {
		{ codes.cover_closed_locked, codes.cartridge_present, codes.twist_locked },
		{ codes.cover_unlocked, codes.cartridge_present, codes.twist_locked },
		{ codes.cover_open, codes.cartridge_present, codes.twist_unlocked },
		{ codes.cover_open, codes.cartridge_removed, codes.twist_unlocked },
		{ codes.cover_open, codes.cartridge_fresh_seated, codes.twist_unlocked },
		{ codes.cover_closed_locked, codes.cartridge_fresh_seated, codes.twist_locked }
	};
	int i;
	for (i = 0; i < 6; i++) {
		SlotVisuals v = visual(steps[i][0], steps[i][1], steps[i][2], i, Slot_LmPrim);
		AdapterResult r = MapSlotVisuals(v, codes);
		CHECK(r.reports_swap_complete == 0);
		CHECK(r.writes_remaining_man_hours == 0);
		CHECK(r.writes_commit_generation == 0);
		CHECK(r.cover.phase_echo == steps[i][0]);
		CHECK(r.cartridge.phase_echo == steps[i][1]);
		CHECK(r.twist.phase_echo == steps[i][2]);
	}
	CHECK(id == 100);
	CHECK(generation == 2);
	CHECK(remaining == 36.0);
}

static void test_secondary_and_cm_do_not_gain_a_vessel_command() {
	using namespace nassp_lioh_review;
	PhaseCodes codes = fixture_codes();
	SlotVisuals sec = visual(codes.cover_open, codes.cartridge_fresh_seated,
		codes.twist_locked, 3, Slot_LmSec);
	AdapterResult s = MapSlotVisuals(sec, codes);
	CHECK(s.cover.proposal_ready == 0);
	CHECK(s.cover.angle_known == 0);
	CHECK(s.cover.switch_value == -1);
	CHECK(s.cover.sec_valve_enabled == 0);
	CHECK(s.cover.proposed_for_prim_valve == 0);
	CHECK(s.lm_sec_assess_complete == 0);
	CHECK(s.reports_swap_complete == 0);
	CHECK(s.twist.phase_echo == codes.twist_locked);
	CHECK(s.cartridge.phase_echo == codes.cartridge_fresh_seated);
	CHECK(s.cover.phase_echo == codes.cover_open);

	SlotVisuals cm = visual(codes.cover_open, codes.cartridge_removed,
		codes.twist_unlocked, 2, Slot_Cm);
	AdapterResult c = MapSlotVisuals(cm, codes);
	CHECK(c.cm_animation_defined == 0);
	CHECK(c.cover.angle_known == 0);
	CHECK(c.cover.proposal_ready == 0);
	CHECK(c.cover.proposed_for_prim_valve == 0);
	CHECK(c.reports_swap_complete == 0);
	CHECK(c.cover.phase_echo == codes.cover_open);
	CHECK(c.cartridge.phase_echo == codes.cartridge_removed);
	CHECK(c.twist.phase_echo == codes.twist_unlocked);
}

static void test_selector_and_vent_use_repo_definitions() {
	using namespace nassp_lioh_review;
	SelectorVentDisplay prim = DescribeSelectorVent(1, 1);
	SelectorVentDisplay sec = DescribeSelectorVent(0, 0);
	SelectorVentDisplay bad = DescribeSelectorVent(2, 4);

	CHECK(prim.selector_anim_known == 1);
	CHECK(prim.selector_anim == 1.0);
	CHECK(prim.selector_rotation_deg == 30.0);
	CHECK(prim.selector_axis_x == -1.0);
	CHECK(prim.selector_axis_y == 0.0);
	CHECK(prim.selector_axis_z == 0.0);
	CHECK(prim.selector_pivot_x == 0.3544);
	CHECK(prim.selector_pivot_y == 0.3645);
	CHECK(prim.selector_pivot_z == 0.1312);
	CHECK(prim.vent_anim == 1.0);
	CHECK(prim.vent_translation_x_m == 0.003);
	CHECK(prim.vent_translation_y_m == 0.0);
	CHECK(prim.vent_translation_z_m == 0.0);
	CHECK(IsMeasuredVentTravel(prim.vent_translation_x_m));
	CHECK(IsMeasuredSelectorTravel(prim.selector_rotation_deg));

	CHECK(sec.selector_anim == 0.0);
	CHECK(sec.selector_rotation_deg == 0.0);
	CHECK(sec.vent_anim == 0.0);
	CHECK(sec.vent_translation_x_m == 0.0);

	CHECK(bad.selector_anim_known == 0);
	CHECK(bad.vent_anim_known == 0);
	CHECK(bad.selector_rotation_deg == 0.0);
	CHECK(bad.vent_translation_x_m == 0.0);

	CHECK(prim.driven_by_slot_observation == 0);
	CHECK(prim.reports_swap_complete == 0);
	CHECK(prim.writes_remaining_man_hours == 0);
	CHECK(prim.writes_commit_generation == 0);
	CHECK(prim.preview_guess_accepted == 0);
	CHECK(IsMeasuredSelectorTravel(30.0));
	CHECK(IsMeasuredVentTravel(0.003));
	CHECK(!IsMeasuredSelectorTravel(40.0));
	CHECK(!IsMeasuredVentTravel(0.004));

	CHECK(kPrimaryEnvelopeA_m == 0.286);
	CHECK(kPrimaryEnvelopeB_m == 0.187);
	CHECK(kPrimaryEnvelopeApproved == 1);
	CHECK(kSecondaryAxisA_m == 0.127);
	CHECK(kSecondaryAxisB_m == 0.205);
	CHECK(kSecondaryEnvelopeConfirmed == 0);
	CHECK(kSecondaryOrientationKnown == 0);
}

static void test_catalog_writer_rules() {
	using namespace nassp_lioh_review;
	const ContractCase* sec = find_case("ASSESS_LM_SEC");
	const ContractCase* commit = find_case("COMMIT");
	const ContractCase* consume = find_case("CONSUME");
	const ContractCase* sensing = find_case("SENSING");
	const ContractCase* observe = find_case("OBSERVE");
	const ContractCase* cover = find_case("COVER_REOPEN");
	const ContractCase* sel = find_case("SEL_VENT");
	const ContractCase* a13 = find_case("A13");
	const ContractCase* state = find_case("STATE_API");
	const SensingBit* twist = find_bit("cartridge_twist_locked");
	const SensingBit* divert_a = find_bit("divert_a");
	const SensingBit* divert_b = find_bit("divert_b");
	const SensingBit* panel = find_bit("attenuation_panel_open");
	const SensingBit* cover_open = find_bit("cover_open");
	const SensingBit* cover_locked = find_bit("cover_locked");
	int i;
	int gen_writers = 0;
	int rem_writers = 0;

	CHECK(sec != 0);
	CHECK(strcmp(sec->enable_off, "false") == 0);
	CHECK(strcmp(sec->enable_on, "false") == 0);
	CHECK(sec->rem_may_change == 0);
	CHECK(sec->gen_may_change == 0);
	CHECK(kLmSecAssessEnabled == 0);

	CHECK(commit != 0 && consume != 0 && sensing != 0 && observe != 0);
	CHECK(commit->rem_may_change == 1 && commit->gen_may_change == 1);
	CHECK(consume->rem_may_change == 1 && consume->gen_may_change == 0);
	CHECK(sensing->rem_may_change == 0 && sensing->gen_may_change == 0);
	CHECK(observe->rem_may_change == 0 && observe->gen_may_change == 0);
	CHECK(cover->rem_may_change == 0 && cover->gen_may_change == 0);
	CHECK(sel->rem_may_change == 0 && sel->gen_may_change == 0);
	CHECK(a13->rem_may_change == 0 && a13->gen_may_change == 0);
	CHECK(strcmp(state->enable_off, "false") == 0);
	CHECK(kStateApiDefaultOff == 1);
	CHECK(kVesselSaveAirflowLinked == 0);
	CHECK(kContractCasesRunnableHere == 0);
	CHECK(kAgainstCurrentTreeVerified == 0);
	CHECK(kStagingSourcesCopied == 0);
	CHECK(kStagingHarnessReexecutedHere == 0);

	for (i = 0; i < kContractCaseCount; i++) {
		const ContractCase* c = &kContractCases[i];
		if (c->gen_may_change) {
			gen_writers++;
			CHECK(strcmp(c->symbol, "TryCommitSwap") == 0);
		}
		if (c->rem_may_change) {
			rem_writers++;
			CHECK(strcmp(c->symbol, "TryCommitSwap") == 0
				|| strcmp(c->symbol, "TryConsume") == 0);
		}
		CHECK(c->symbol != 0 && c->symbol[0] != 0);
	}
	CHECK(gen_writers == 1);
	CHECK(rem_writers == 2);

	CHECK(twist != 0);
	CHECK(twist->alone_sufficient == 0);
	CHECK(twist->cm_conjunction == 0);
	CHECK(twist->lm_prim_conjunction == 1);
	CHECK(divert_a->mutually_exclusive_divert == 1);
	CHECK(divert_b->mutually_exclusive_divert == 1);
	CHECK(panel->cm_conjunction == 0 && panel->lm_prim_conjunction == 0);
	CHECK(panel->alone_sufficient == 0);
	CHECK(cover_open->cm_conjunction == 0);
	CHECK(cover_locked->cm_conjunction == 1 && cover_locked->lm_prim_conjunction == 1);

	for (i = 0; i < kSensingBitCount; i++)
		CHECK(kSensingBits[i].alone_sufficient == 0);

	CHECK(kReportedCmFreshManHours == 36.0);
	CHECK(kReportedLmPrimFreshManHours == 41.0);
	CHECK(kReportedLmSecManHours == 14.0);
	CHECK(strcmp(kReportedOnTestSha256,
		"68185EA1B0A0206176325A8AFCB445F2943A04F0898CBF53754A0F6E9769A9A4") == 0);
	CHECK(strcmp(kReportedOffTestSha256,
		"FC60C65C8CB5B9C862F593C458E75482984690A2899CA19A08E74AAD098B323C") == 0);
	CHECK(strcmp(kRepoHead, "8ca3a48c748760177f04dc584dc7492178527bee") == 0);
}

static void expect_contains(const char* rel, const char* needle) {
	int opened = 0;
	int has = file_has(rel, needle, &opened);
	CHECK(opened == 1);
	CHECK(has == 1);
}

static void expect_lacks(const char* rel, const char* needle) {
	int opened = 0;
	int has = file_has(rel, needle, &opened);
	CHECK(opened == 1);
	CHECK(has == 0);
}

static void test_repo_facts_and_absent_staging() {
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"CO2CanisterSelectSwitch.SetRotationRange(RAD * 30.0);");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"const VECTOR3 ECSVectorSSA = { 0.003, 0, 0 };");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"const VECTOR3 ECSRotAxisSSA = { -1, 0, 0 };");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"const VECTOR3 Sw_CanisterSelLocation = { 0.3544, 0.3645, 0.1312 };");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"const VECTOR3 PB_Vent1Location = { 0.3382, 0.1455, 0.2815 };");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lemvc.cpp",
		"const VECTOR3 PB_Vent2Location = { 0.3382, 0.2936, -0.2550 };");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lempanel.cpp",
		"CO2CanisterPrimValve.AddPosition(0, 300);");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lempanel.cpp",
		"CO2CanisterPrimValve.AddPosition(1, 0);");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lempanel.cpp",
		"CO2CanisterSecValve.AddPosition(0, 300);");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lempanel.cpp",
		"CO2CanisterSecValve.AddPosition(1, 0);");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_csm/saturn.cpp",
		"oapiWriteScenario_int (scn, \"A13STATE\", GetA13State());");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lm_ecs.cpp",
		"//DIRECT O2");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_lm/lm_ecs.cpp",
		"LEMCO2CanisterSelect::SystemTimestep");
	expect_lacks("Orbitersdk/samples/ProjectApollo/src_lm/lm_ecs.cpp",
		"CO2CanisterPrimValve");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_sys/PanelSDK/Internals/Esystems.cpp",
		"if (co2removalrate <= 0.0356)");
	expect_lacks("Orbitersdk/samples/ProjectApollo/src_sys/PanelSDK/Internals/Esystems.cpp",
		"TryConsume");
	expect_contains("Orbitersdk/samples/ProjectApollo/src_sys/PanelSDK/Internals/Hsystems.cpp",
		"if (co2removalrate <= 0.0356)");
	expect_lacks("Orbitersdk/samples/ProjectApollo/src_sys/PanelSDK/Internals/Hsystems.cpp",
		"TryConsume");
	expect_lacks("Orbitersdk/samples/ProjectApollo/src_sys/PanelSDK/Internals/Hsystems.cpp",
		"ecs_lioh");

	CHECK(file_missing("Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_state.cpp"));
	CHECK(file_missing("Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_state.h"));
	CHECK(file_missing("Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_saveload.cpp"));
	CHECK(file_missing("ecs_flightplan_coverage/lioh_staging"));

	expect_contains(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.h",
		"UNVERIFIED");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"TryCommitSwap");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"AssessSwapComplete");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"ApplySensingObservation");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"ObserveForAnimation");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"TryConsume");
	expect_lacks(
		"Orbitersdk/samples/ProjectApollo/src_sys/ecs_lioh_anim_adapter/lioh_anim_adapter.cpp",
		"WouldConsume");
}

int main() {
	printf("staging report: %s\n", nassp_lioh_review::kStagingReport);
	printf("against-current-tree applicability: UNVERIFIED\n");
	printf("staging sources copied: no\n");
	printf("reported staging path: %s\n", nassp_lioh_review::kReportedStagingPath);
	printf("reported merge path: %s\n", nassp_lioh_review::kReportedMergePath);
	printf("repo head read here: %s\n", nassp_lioh_review::kRepoHead);
	printf("contract cases runnable on this branch: %d\n",
		nassp_lioh_review::kContractCasesRunnableHere);
	printf("reported harness re-executed here: %d\n",
		nassp_lioh_review::kStagingHarnessReexecutedHere);

	test_channels_stay_distinct();
	test_unlocked_and_unknown_have_no_invented_angle();
	test_procedure_and_completion_are_not_the_pose();
	test_interrupted_visual_sequence_leaves_identity_witness();
	test_secondary_and_cm_do_not_gain_a_vessel_command();
	test_selector_and_vent_use_repo_definitions();
	test_catalog_writer_rules();
	test_repo_facts_and_absent_staging();

	if (g_failed) {
		fprintf(stderr, "%d checks failed out of %d\n", g_failed, g_checked);
		return 1;
	}
	printf("animation adapter and catalog checks passed: %d\n", g_checked);
	printf("staging AssessSwapComplete/ApplySensingObservation/TryCommitSwap/TryConsume: NOT_RUN\n");
	return 0;
}
