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
# resume simulation
./src/pause.sh r
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



