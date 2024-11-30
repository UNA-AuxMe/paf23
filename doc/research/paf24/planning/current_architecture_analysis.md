Notes (needs to be refined later):
- Current behaviour tree documentation is incomplete:
  - Priorities: unstuck - leave parking space - intersection - laneswitch - overtake - cruise
  -  Road Features covers behaviour decision logic
  -  Intersection includes stop signs
  -  No documentation for each behaviour
- Already available function headers that are not implemented or incomplete like dotted line detection(not marked with a TODO) or left/right lane(same topic?)
- Stop sign only handled in intersection-approach, not sure though if simulation has any special stop signs elsewhere like at construction side so this is probably okay
- lane change missing check for traffic, maneuvers also has a SwitchLaneLeft function(incomplete)
- overtake structure is a bit confusing, a refactor with better sub behaviours seems reasonable (wait for lane change - lane changing - overtaking - lane changing back - finished)
