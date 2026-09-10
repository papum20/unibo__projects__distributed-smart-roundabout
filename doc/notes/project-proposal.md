**Title:** [Bologna] [D'Ugo] Project proposal: Smart Roundabout Orchestrator

**Vision**
I propose to design and implement a simulation of a **Distributed Cyber-Physical System (CPS)** managing a "Smart Roundabout". The system consists of a Central Controller node and multiple autonomous Car nodes.

Unlike a local simulation, every Car and the Controller will run as independent processes (containerized), communicating exclusively over a network. The Controller receives telemetry (position, speed, timestamp) from Cars (sensors) and issues commands (accelerate, brake, maintain speed) to optimize traffic flow and prevent collisions (to Cars' actuators).

The core challenge is managing the **uncertainty of distributed systems**: network latency, message loss, and node failures. The system must guarantee **Safety** (no accidents) even if the network lags, while trying to maximize **Liveness** (throughput) when conditions are optimal.

**Learning Goals**
*   **Architecture:** Implementing a centralized control pattern in a distributed setting (Sensor $\to$ Controller $\to$ Actuator loop over network).
*   **Time & Synchronization:** Handling clock skew and network jitter (ordering events from cars arriving out of order).
*   **Dependability & Fault Tolerance:** Implementing a **Fail-Safe** state. If the Controller crashes or the network is too slow (heartbeat timeout), Cars must detect it and switch to a local "safety mode" (e.g., stop before entering the junction).
*   **Middleware:** Using a message broker or socket-based communication to decouple components.

**Intended Technologies**
*   **Language:** Python (using `asyncio` for concurrency).
*   **Communication:** MQTT (using `Eclipse Mosquitto`). This fits the IoT/CPS theme of "connected cars".
*   **Deployment:** Docker & Docker Compose (to simulate the network of distinct nodes).
*   **Visualization:** A lightweight Web Dashboard (Python Flask/FastAPI + simple HTML/JS canvas) or a CLI visualizer to show the real-time state of the roundabout.

**Intended Deliverables**
*   Source code (Car Agent, Controller Agent, Viewer).
*   Docker Compose file to orchestrate the simulation.
*   A mechanism to artificially inject *network lag* and *packet loss* to demonstrate the system's robustness during the exam.
*   Final Report.

**Usage Scenarios**
1.  **Normal Operation:** Cars approach, register with the Controller, receive speed adjustments to interleave perfectly without stopping (efficiency).
2.  **Network Saturation/Lag:** The Controller receives position updates late. It must account for the delay and issue conservative commands to ensure safety margins are maintained.
3.  **Controller Failure:** The Controller process is killed. Cars detect the absence of "heartbeats" or commands and automatically brake to a halt before entering the danger zone (Fail-Safe).

**Group Members**
Daniele D'Ugo - daniele.dugo@studio.unibo.it

