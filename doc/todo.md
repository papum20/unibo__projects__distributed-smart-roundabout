vehicle:
* spawn no overlap
* if no signal, dont just stop
  * impl failsafe

controller:
* logic: check all %2pi, what if get 0? eg at entrance (eg for conflict)
* optimiziation (not only acc -2)
  * otherwise stop
  * before roundabout, not inside

test:
* test error cases for dockers not responding
  * add a way to do that
* final tests/benchmarks:
  * show cars throughput w w/o controller coordination (need autonomous cars)
  * total summed time halted

commit:
