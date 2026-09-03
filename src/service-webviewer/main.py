import asyncio
import logging
import json
from contextlib import asynccontextmanager

import aiomqtt
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
import uvicorn

from common.const import (
	ROUNDABOUT_RADIUS,
	ROUNDABOUT_N_ROADS,
	LANE_WIDTH,
	CAR_LENGTH,
	CAR_WIDTH
)
from common.get_env import config
from common.models.models import Vehicle, SystemCommand, VehicleCollision



logger = logging.getLogger(__name__)



# In-memory dictionary to store the latest state of all cars
vehicles_state								= {}
tot_vehicles_spawned						= 0
vehicles_collisions: list[VehicleCollision] = []
controller_precedence_q						= []


async def mqtt_listener():
	global tot_vehicles_spawned, controller_precedence_q

	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}')
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/{config.TOPIC_VEHICLE_COLLISIONS_SUFFIX}')
		await client.subscribe(config.TOPIC_CONTROLLER_STATUS)
		print("Viewer subscribed to telemetry and controller status...")

		async for message in client.messages:
			payload = json.loads(message.payload)

			if str(message.topic) == config.TOPIC_CONTROLLER_STATUS:
				controller_precedence_q = payload.get("precedence_queue", [])
				continue

			if str(message.topic) == f'{config.TOPIC_VEHICLE_PREFIX}/{config.TOPIC_VEHICLE_COLLISIONS_SUFFIX}':
				vehicles_collisions.append(VehicleCollision(**payload))
				continue

			vehicle						= Vehicle(**payload)
			if vehicle.id not in vehicles_state:
				tot_vehicles_spawned	+= 1
			vehicles_state[vehicle.id]	= vehicle

# https://fastapi.tiangolo.com/advanced/events/#use-case
@asynccontextmanager
async def lifespan(app: FastAPI):
	listener_task = asyncio.create_task(mqtt_listener())

	try:
		yield
	finally:
		listener_task.cancel()
		await asyncio.gather(listener_task, return_exceptions=True)


app = FastAPI(lifespan=lifespan)




@app.get("/api/config")
async def get_config():
	return {
		"n_roads"		: ROUNDABOUT_N_ROADS,
		"r_radius"		: ROUNDABOUT_RADIUS,
		"lane_width"	: LANE_WIDTH,
		"car_length"	: CAR_LENGTH,
		"car_width"		: CAR_WIDTH,
		"scale"			: 2.5	# drawing scaling
	}


@app.post("/api/control")
async def control_sim(cmd: SystemCommand):
	resolved_vehicle_id = None
	if cmd.vehicle_id:
		matches = [
			vehicle_id
			for vehicle_id in vehicles_state
			if vehicle_id.startswith(cmd.vehicle_id)
		]
		if not matches:
			raise HTTPException(
				status_code=404,
				detail=f"No vehicle matches ID prefix '{cmd.vehicle_id}'"
			)
		if len(matches) > 1:
			raise HTTPException(
				status_code=409,
				detail={
					"message": f"Vehicle ID prefix '{cmd.vehicle_id}' is ambiguous",
					"matches": matches,
				},
			)
		resolved_vehicle_id = matches[0]
	
	topic = (
		f"{config.TOPIC_SYSCTRL_PREFIX}/{resolved_vehicle_id}"
		if cmd.vehicle_id
		else f"{config.TOPIC_SYSCTRL_PREFIX}/{config.TOPIC_SYSCTRL_BROADCAST_SUFFIX}"
	)
	resolved_cmd = cmd.model_copy(
		update={"vehicle_id": resolved_vehicle_id}
	)

	async with aiomqtt.Client(hostname=config.HOST_BROKER,  port=config.PORT_BROKER) as client:
		await client.publish(topic, payload=resolved_cmd.model_dump_json())

	return {"status": "ok", "command": resolved_cmd, "topic": topic}


@app.get("/api/state")
async def get_state():
	return {
		"vehicles"				: vehicles_state,
		"tot_vehicles_spawned"	: tot_vehicles_spawned,
		"precedence_queue"		: controller_precedence_q,
		"collisions"			: vehicles_collisions,
	}


@app.get("/")
async def serve_frontend():
	with open("index.html", "r") as f:
		return HTMLResponse(content=f.read())



if __name__ == "__main__":
	uvicorn.run(
		app, host="0.0.0.0", port=config.PORT_WEBVIEWER,
		access_log=False,
		#log_level="warning"
	)