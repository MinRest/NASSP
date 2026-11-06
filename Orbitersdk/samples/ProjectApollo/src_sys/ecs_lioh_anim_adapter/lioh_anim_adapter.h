/*
  LiOH animation adapter — review mapping only.

  ODX12 owns integration into the custom 459 tree. This branch does not
  define ObserveForAnimation, AssessSwapComplete, ApplySensingObservation,
  TryCommitSwap, TryConsume, WouldConsume, or the LIOHV1/LIOHV2 helpers.

  Against-current-tree applicability is UNVERIFIED. The L.5 staging sources
  were not in this checkout and were not copied.

  ObserveForAnimation is visuals-only. Cover, cartridge occupancy, and
  cartridge twist stay distinct channels. This mapper has no parameter for
  identity.remaining_man_hours or identity.commit_generation.
*/

#ifndef LIOH_ANIM_ADAPTER_H
#define LIOH_ANIM_ADAPTER_H

namespace nassp_lioh_review {

/* Caller supplies the staging enum values. The integers used by this
   branch's tests are fixture labels, not a verified staging ABI. */
struct PhaseCodes {
	int cover_closed_locked;
	int cover_unlocked;
	int cover_open;
	int cartridge_present;
	int cartridge_removed;
	int cartridge_fresh_seated;
	int twist_unlocked;
	int twist_locked;
};

/* SlotKind is an adapter label so a secondary observation cannot be
   played back as the primary cover. It is not a second vessel type. */
enum SlotKind {
	Slot_Cm = 1,
	Slot_LmPrim = 2,
	Slot_LmSec = 3
};

/* Fields correspond to the visuals ObserveForAnimation is documented
   to expose: cover, cartridge, twist. procedure_phase is accepted and
   ignored so a label cannot become a cockpit command or a completion. */
struct SlotVisuals {
	int cover;
	int cartridge;
	int twist;
	int procedure_phase;
	int slot_kind;
};

struct CoverCommand {
	int phase_echo;
	int proposal_ready;
	int bound_in_this_branch;
	int proposed_for_prim_valve;
	int sec_valve_enabled;
	int switch_value;
	int angle_known;
	double cockpit_deg;
	const char* proposed_target;
};

struct TwistCommand {
	int phase_echo;
	int angle_known;
	int distinct_from_cover;
};

struct CartridgeCommand {
	int phase_echo;
	int translation_known;
	int distinct_from_cover;
	int distinct_from_twist;
};

struct AdapterResult {
	CoverCommand cover;
	TwistCommand twist;
	CartridgeCommand cartridge;
	int reports_swap_complete;
	int writes_remaining_man_hours;
	int writes_commit_generation;
	int lm_sec_assess_complete;
	int checklist_autocomplete;
	int checklist_item_bound;
	int a13_mailbox_write;
	int mesh_edit;
	int cm_animation_defined;
	int bore_known;
	int stowage_known;
};

struct SelectorVentDisplay {
	int selector_switch_state;
	int selector_anim_known;
	double selector_anim;
	double selector_rotation_deg;
	double selector_axis_x;
	double selector_axis_y;
	double selector_axis_z;
	double selector_pivot_x;
	double selector_pivot_y;
	double selector_pivot_z;
	int vent_switch_state;
	int vent_anim_known;
	double vent_anim;
	double vent_translation_x_m;
	double vent_translation_y_m;
	double vent_translation_z_m;
	int driven_by_slot_observation;
	int reports_swap_complete;
	int writes_remaining_man_hours;
	int writes_commit_generation;
	int preview_guess_accepted;
};

AdapterResult MapSlotVisuals(const SlotVisuals& visuals, const PhaseCodes& codes);
SelectorVentDisplay DescribeSelectorVent(int selector_switch_state, int vent_switch_state);

/* Measured full-range travel from this repo's cockpit code.
   Selector: CO2CanisterSelectSwitch.SetRotationRange(RAD * 30.0).
   Vent: MGROUP_TRANSLATE by ECSVectorSSA { 0.003, 0, 0 }. */
int IsMeasuredSelectorTravel(double degrees);
int IsMeasuredVentTravel(double metres);

/* Approved primary envelope 286 x 187 mm. Secondary 127 x 205 mm is an
   estimate; orientation is not confirmed. Neither value is a mesh edit. */
static const double kPrimaryEnvelopeA_m = 0.286;
static const double kPrimaryEnvelopeB_m = 0.187;
static const int kPrimaryEnvelopeApproved = 1;
static const double kSecondaryAxisA_m = 0.127;
static const double kSecondaryAxisB_m = 0.205;
static const int kSecondaryEnvelopeConfirmed = 0;
static const int kSecondaryOrientationKnown = 0;

}

#endif
