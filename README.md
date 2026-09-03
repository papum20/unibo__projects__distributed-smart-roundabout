# Microservices-ACMEMobility
Project for the Distributed Systems course at University of Bologna 2025/2026. A simulation of a **Distributed Cyber-Physical System (CPS)** managing a "Smart Roundabout".  

## Requirements

- docker


## Usage

Viewer: http://localhost:8080  
* `spacebar` : pause/resume simulation

```bash
# pause simulation
./src/pause.sh p
./src/ctrl.sh pause
# resume simulation
./src/pause.sh r
./src/ctrl.sh resume vehicle-id
# enter failsafe
./src/ctrl.sh f
# exit failsafe, for vehicle with ID starting with 4fd2
./src/ctrl.sh c 4fd2
# mark vehicle as disconnected
./src/ctrl.sh d 4fd2
# exit disconnected
./src/ctrl.sh n 4fd2
```


### Commands

Start all services:
```bash
./src/start.sh
```

Stop all services:
```bash
./src/down.sh
```

Run tests:
```bash
pytest
```



