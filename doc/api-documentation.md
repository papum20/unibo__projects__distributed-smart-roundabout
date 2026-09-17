# API documentation

Documentation for the Web Viewer REST API.  

**GET `/api/config`**
*   **Description:** Retrieves the static geometrical configuration of the simulated roundabout.
*   **Response:** JSON object containing `n_roads`, `r_radius`, `lane_width`, `car_length`, `car_width`, and `scale`.

**GET `/api/state`**
*   **Description:** Retrieves the current real-time state of the simulation.
*   **Response:** JSON object containing `vehicles` (active vehicles dictionary), `tot_vehicles_spawned`, `ghosts` (disconnected vehicles being tracked), `precedence_queue`, and recent `collisions`.

**POST `/api/control`**
*   **Description:** Submits a `SystemCommand` to alter the simulation state, pause/resume, or inject network faults into vehicles.
*   **Payload:** JSON object matching the `SystemCommand` schema.
    *   `command` (string, required): `PAUSE`, `RESUME`, `ENTER_FAILSAFE`, `EXIT_FAILSAFE`, `ENTER_DISCONNECTED`, `EXIT_DISCONNECTED`.
    *   `vehicle_id` (string, optional): Prefix of the target vehicle ID.
    *   `vehicle_count` (int, optional): Number of random vehicles to target.
	*	if no vehicle is specified, the command will be applied to all vehicles.
*   **Response:** `{"status": "ok", "commands": [...]}`
