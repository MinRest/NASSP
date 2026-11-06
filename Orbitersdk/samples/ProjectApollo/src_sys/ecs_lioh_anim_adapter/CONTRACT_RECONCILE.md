# LiOH L.5 review mapping

ODX12 owns merge integration into the custom 459 tree. This branch is a review mapping and a test catalog. It is not a replacement subsystem, and it was not copied onto `NASSP-459-merge`.

Against-current-tree applicability: **UNVERIFIED**.

## Baseline that was actually read

| Item | Value |
|---|---|
| Checkout | `MinRest/NASSP` branch `Orbiter2016` |
| HEAD | `8ca3a48c748760177f04dc584dc7492178527bee` (matches `origin/Orbiter2016` after fetch) |
| `ecs_lioh_*` in this checkout | Absent |
| FPR package `/workspace/ecs_flightplan_coverage/lioh_staging/` | Absent |
| Reported staging path | `C:\OrbiterDev\patches\nassp459_merge_prep\lioh_finite_odx12_20261106\` |
| Reported merge path | `C:\OrbiterDev\NASSP-459-merge` (`src_sys\ecs_lioh_*`) |
| Reported harness | `build_ecs_lioh_l5_tests.cmd` PASS, not re-executed here |
| Reported ON exe SHA256 | `68185EA1B0A0206176325A8AFCB445F2943A04F0898CBF53754A0F6E9769A9A4` |
| Reported OFF exe SHA256 | `FC60C65C8CB5B9C862F593C458E75482984690A2899CA19A08E74AAD098B323C` |

`X:\OrbiterDev` and `C:\OrbiterDev` were not mounted. The staging report is the integration target. Its unit tests are recorded as already passing, with vessel, save/load, and airflow unlinked, and with the implementation off by default.

## Existing symbols this branch leaves undefined

`StateApiEnabled`, `SaveApiEnabled`, `ObserveForAnimation`, `WouldConsume`, `TryConsume`, `AssessSwapComplete`, `ApplySensingObservation`, `TryCommitSwap`, `Serialize` / `Deserialize`, `ParseScenarioLine`, `ShouldWrite`, `Format`.

Identity writers named by the report:

- `identity.remaining_man_hours` changes through `TryConsume` or through `TryCommitSwap`.
- `identity.commit_generation` changes through `TryCommitSwap` only.
- `ApplySensingObservation` updates cover, cartridge occupancy, twist, and `procedure_phase`.
- `ObserveForAnimation` returns visuals. Mesh playback follows that observation.
- LM secondary `AssessSwapComplete` stays false. This branch does not enable it.
- Reported fresh capacities, applied only by the staging commit path: CM A/B 36 man-hours, LM primary 41, LM secondary 14.

## Animation mapping the integrator can bind

`MapSlotVisuals` reads cover, cartridge, and twist. It has no identity argument. `procedure_phase` is accepted and ignored. `reports_swap_complete` is 0. `bound_in_this_branch` is 0.

| Observation | Proposed cockpit playback | Bound here |
|---|---|---|
| LM primary `ClosedLocked` | Existing `CO2CanisterPrimValve` value 1, 0 degrees (`AddPosition(1, 0)`) | No |
| LM primary `Open` | Existing `CO2CanisterPrimValve` value 0, 300 degrees (`AddPosition(0, 300)`) | No |
| LM primary `Unlocked` | No angle. The registered switch has two positions. | No |
| LM secondary, any cover | No vessel command. Secondary assess stays off. `CO2CanisterSecValve` remains in the cockpit and is not in the report's live-bindable set. | No |
| CM, any cover | No angle. This repo has no Panel 350 divert, bleed, or cover animation. | No |
| Twist `Locked` or `Unlocked` | Phase echo only. Twist angle, bore, and stowage location are unknown. | No |
| Cartridge `Present` / `Removed` / `FreshSeated` | Phase echo only. No translation. | No |

Enum integers are supplied by the caller (`PhaseCodes`). The test fixture uses 10/11/12, 20/21/22, and 30/31 so the mapper cannot be read as the staging ABI.

Selector and vent are existing switches, not slot-observation outputs (`driven_by_slot_observation` is 0):

| Control | Measured definition in this repo | Preview value that is rejected |
|---|---|---|
| `CO2CanisterSelectSwitch` | `SetRotationRange(RAD * 30.0)`, axis `{-1, 0, 0}`, pivot `Sw_CanisterSelLocation` `{0.3544, 0.3645, 0.1312}`. State 1 (PRIM, toggle up) anim 1. State 0 anim 0. | 40 degrees |
| `CO2CanisterPrimVent` / `SecVent` | `MGROUP_TRANSLATE` by `ECSVectorSSA {0.003, 0, 0}` (3 mm) at anim 1. | 4 mm |

Primary envelope recorded for the asset handoff: 286 x 187 mm, approved. Secondary 127 x 205 mm, estimated, orientation unconfirmed. `mesh_edit` is 0.

## Adaptation points

1. Fill `SlotVisuals` from `ObserveForAnimation`. Keep that function as the only observer.
2. Pass the staging `CoverPhase`, `CartridgeOccupancy`, and `CartridgeTwistPhase` values in `PhaseCodes` when the staging header is on the include path.
3. Play the primary cover command on the existing `CO2CanisterPrimValve` only after ODX12 binds it. Leave `CO2CanisterSecValve` unwired unless the staging live set gains it.
4. Leave selector and vent on `LEMCO2CanisterSelect` / `LEMCO2CanisterVent`. Use the 30 degree and 3 mm definitions above.
5. Leave CM divert, pressure pin, and manual bleed as sensing bits. This repo has no Panel 350 objects for them.
6. Keep `TryConsume` and `TryCommitSwap` as the only identity writers. Fresh capacities stay 36 / 36 / 41. LM secondary commit stays unavailable.
7. Keep checklist auto-complete unbound. The 154 proposed rows and 15 unrun fixtures were not in this tree. A completed checklist row is not `AssessSwapComplete`.
8. Keep `Saturn::A13State` (`A13STATE`: `ApolloExploded`, `CryoStir`, `KranzPlayed`) and the timed-swap adapter unchanged. The report excludes `Slot::MAILBOX`.
9. Keep LIOHV2 twist serialization and LIOHV1 reads on the staging save helper. This branch does not parse scenarios.
10. Leave `StateApiEnabled` and `SaveApiEnabled` defaulting off. Vessel CMake, `AtmRegen`, and `h_CO2Scrubber` stay without these symbols, matching the report.

## What the tests do on this branch

`build_lioh_anim_adapter_tests.sh` compiles a host program with no Orbiter dependency. It checks the mapper, the measured selector/vent strings in `lemvc.cpp` and `lempanel.cpp`, the contract catalog's writer rules, and the absence of staging sources. Contract behavior cases are marked not runnable here.
