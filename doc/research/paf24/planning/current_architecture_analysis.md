Notes (needs to be refined later):
- Current behaviour tree documentation is incomplete:
  - Priorities: unstuck - leave parking space - intersection - laneswitch - overtake - cruise
  -  Road Features covers behaviour decision logic
  -  Intersection includes stop signs
  -  No documentation for each behaviour
- Already available function headers that are not implemented or incomplete like dotted line detection(not marked with a TODO) or left/right lane(same topic?)
- Stop sign only handled in intersection-approach, not sure though if simulation has any special stop signs elsewhere like at construction side so this is probably okay
- lane change missing check for traffic, maneuvers also has a SwitchLaneLeft/Right functions(incomplete)
- overtake structure is a bit confusing, a refactor with better sub behaviours seems reasonable (wait for lane change - lane changing - overtaking - lane changing back - finished)
- maybe laneswitch should have a higher priority than intersection since we might need to switch to the correct lane before handling the intersection itself
- laneswitch has some wrong comments most likely by copying from intersection
- laneswitch success is uncertain, currently succesfull once we reach the next waypoint
- we have no "fail state" behaviour for getting back on track e.g. after missing an exit or something else goes wrong that makes us go off track, but this is most likely not something we should focus on right now
- motion planning is currently a bit of a god class and difficult to read, it should be discussed if a refactor is reasonable
- it seems like currently lane changes are calculated as specific points in global planing, motion planning then only slows down so we switch lane at that specific point. It would be more reasonable to
make laneswitch more general so that when it is triggered, the trajectory gets modified and the vehicle tries to switch lanes as soon as possible.
- there doesn't seem to be a proper global collision avoidance, currently it seems to only handle the nearest obstacle in front IF we are in overtake and close proximity collisions by emergency brake. Collision check should maybe go through all lidar objects, predict possible collisions and then publish all potential collisions. But it is questionable how far into the future we should plan. Different types of possible collisions e.g. cars, pedestrians, bikes, construction side etc. might need seperate handling as well.
Or we should maybe leave most of this to acting.
