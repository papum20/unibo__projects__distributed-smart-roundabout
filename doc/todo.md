vehicle:

controller:
* logic: check all %2pi, what if get 0? eg at entrance (eg for conflict)
* optimiziation (not only acc -2)
  * before roundabout, not inside
* add safety distance to compensate for bad approximative vehicle collision detection
* what if cant yield safely
* check if hit v2 from behind after enetering
* change vehicle_can_enter_safely equation, its flawed, and also if a=A

test:
* test error cases for dockers not responding
* final tests/benchmarks:
  * show cars throughput w w/o controller coordination (need autonomous cars)
  * total summed time halted

commit:
