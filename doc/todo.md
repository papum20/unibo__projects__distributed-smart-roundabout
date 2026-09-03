vehicle:
* spawn no overlap

controller:
* logic: check all %2pi, what if get 0? eg at entrance (eg for conflict)
* optimiziation (not only acc -2)
  * otherwise stop
  * before roundabout, not inside
* optimize againsta failsafe vehicles (eg. consider their max speed and try to do something, instead of just yielding)
* if vehicle in front is in failsafe?
* if vehicle in front entering, cant stop bc too fast and just entered failsafe?
* add safety distance to compensate for bad approximative vehicle collision detection
* 2 steps:
  1. check if need to slow down to avoid collision with v on front
  2. current checks, considering a from prev step as max
  * also check, when entering, if v2 will be able to stop (to avoid that v1 enters slow and v2 was going fast and they dont collide immediately but after a bit)

test:
* test error cases for dockers not responding
* final tests/benchmarks:
  * show cars throughput w w/o controller coordination (need autonomous cars)
  * total summed time halved

commit:
