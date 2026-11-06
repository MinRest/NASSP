#include "lioh_anim_adapter.h"

#include <string.h>

namespace nassp_lioh_review {

static const double kSelectorRangeDeg = 30.0;
static const double kVentTravelM = 0.003;
static const double kCoverClosedDeg = 0.0;
static const double kCoverOpenDeg = 300.0;
static const char* kPrimValveTarget = "CO2CanisterPrimValve";

static int same_code(int value, int code) {
	return value == code;
}

int IsMeasuredSelectorTravel(double degrees) {
	double d = degrees - kSelectorRangeDeg;
	if (d < 0.0) d = -d;
	return d < 1.0e-12;
}

int IsMeasuredVentTravel(double metres) {
	double d = metres - kVentTravelM;
	if (d < 0.0) d = -d;
	return d < 1.0e-12;
}

AdapterResult MapSlotVisuals(const SlotVisuals& visuals, const PhaseCodes& codes) {
	AdapterResult r;
	memset(&r, 0, sizeof(r));

	r.cover.phase_echo = visuals.cover;
	r.cover.switch_value = -1;
	r.cover.angle_known = 0;
	r.cover.cockpit_deg = 0.0;
	r.cover.proposal_ready = 0;
	r.cover.bound_in_this_branch = 0;
	r.cover.proposed_for_prim_valve = 0;
	r.cover.sec_valve_enabled = 0;
	r.cover.proposed_target = "";

	r.twist.phase_echo = visuals.twist;
	r.twist.angle_known = 0;
	r.twist.distinct_from_cover = 1;

	r.cartridge.phase_echo = visuals.cartridge;
	r.cartridge.translation_known = 0;
	r.cartridge.distinct_from_cover = 1;
	r.cartridge.distinct_from_twist = 1;

	r.reports_swap_complete = 0;
	r.writes_remaining_man_hours = 0;
	r.writes_commit_generation = 0;
	r.lm_sec_assess_complete = 0;
	r.checklist_autocomplete = 0;
	r.checklist_item_bound = 0;
	r.a13_mailbox_write = 0;
	r.mesh_edit = 0;
	r.cm_animation_defined = 0;
	r.bore_known = 0;
	r.stowage_known = 0;

	/* procedure_phase is intentionally unread. */
	(void)visuals.procedure_phase;
	(void)codes.cartridge_present;
	(void)codes.cartridge_removed;
	(void)codes.cartridge_fresh_seated;
	(void)codes.twist_unlocked;
	(void)codes.twist_locked;

	if (visuals.slot_kind != Slot_LmPrim)
		return r;

	r.cover.proposed_for_prim_valve = 1;
	r.cover.proposed_target = kPrimValveTarget;

	if (same_code(visuals.cover, codes.cover_closed_locked)) {
		r.cover.switch_value = 1;
		r.cover.angle_known = 1;
		r.cover.cockpit_deg = kCoverClosedDeg;
		r.cover.proposal_ready = 1;
	} else if (same_code(visuals.cover, codes.cover_open)) {
		r.cover.switch_value = 0;
		r.cover.angle_known = 1;
		r.cover.cockpit_deg = kCoverOpenDeg;
		r.cover.proposal_ready = 1;
	}

	return r;
}

SelectorVentDisplay DescribeSelectorVent(int selector_switch_state, int vent_switch_state) {
	SelectorVentDisplay d;
	memset(&d, 0, sizeof(d));

	d.selector_switch_state = selector_switch_state;
	d.vent_switch_state = vent_switch_state;
	d.selector_axis_x = -1.0;
	d.selector_axis_y = 0.0;
	d.selector_axis_z = 0.0;
	d.selector_pivot_x = 0.3544;
	d.selector_pivot_y = 0.3645;
	d.selector_pivot_z = 0.1312;
	d.driven_by_slot_observation = 0;
	d.reports_swap_complete = 0;
	d.writes_remaining_man_hours = 0;
	d.writes_commit_generation = 0;
	d.preview_guess_accepted = 0;

	if (selector_switch_state == 0) {
		d.selector_anim_known = 1;
		d.selector_anim = 0.0;
		d.selector_rotation_deg = 0.0;
	} else if (selector_switch_state == 1) {
		d.selector_anim_known = 1;
		d.selector_anim = 1.0;
		d.selector_rotation_deg = kSelectorRangeDeg;
	} else {
		d.selector_anim_known = 0;
		d.selector_anim = 0.0;
		d.selector_rotation_deg = 0.0;
	}

	if (vent_switch_state == 0) {
		d.vent_anim_known = 1;
		d.vent_anim = 0.0;
	} else if (vent_switch_state == 1) {
		d.vent_anim_known = 1;
		d.vent_anim = 1.0;
		d.vent_translation_x_m = kVentTravelM;
	} else {
		d.vent_anim_known = 0;
	}

	return d;
}

}
