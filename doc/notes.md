## Report

latex template: https://github.com/unibo-fc-isi-ds/template-final-report  

### Additional Learning Goals
* integration tests (between softwares)

## Implementation

### Mosquitto
Since version 2.0, Mosquitto is very secure by default (it blocks external connections). For a university simulation, open it up so containers can talk easily.  

### Pydantic
for:
* model-first development
* runtime validation

### Physics

simplified physics model:
* uniformly accelerated motion

### Refs

Car deceleration:
* https://copradar.com/chapts/references/acceleration.html hard stop brake, average driver 4.5ms2  
* https://autosxpert.com/how-fast-can-cars-decerate/ hard stop brake, average driver and conditions, may wear tires 5ms2  
* https://calculator.academy/deceleration-calculator-w-formula/ hard braking 6-8ms2  

Car acceleration:
* https://ranwhenparked.net/average-0-60-speed/ common models 6-8s 0 to 60mph (26.8m/s, that's ~4.5m/s2 considering 6s)  

Car failsafe max speed:
* 30km/h (8.3m/s) allows to stop (for emergency) in 6.9m < ROUNDABOUT_PROXIMITY_DIST=9m
  * it could go faster far from the roundabout and only decelerate when approaching, but since we focus on the crossroad and since the project isn't about algorithms 	we'll ignore that
