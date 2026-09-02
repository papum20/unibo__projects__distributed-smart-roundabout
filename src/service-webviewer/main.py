import asyncio
import logging
import json
from contextlib import asynccontextmanager

import aiomqtt
from fastapi import FastAPI
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
from common.models.models import Vehicle, SystemCommand



logger = logging.getLogger(__name__)



# In-memory dictionary to store the latest state of all cars
vehicles_state = {}
controller_precedence_q = []


async def mqtt_listener():
	global controller_precedence_q

	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}')
		await client.subscribe(config.TOPIC_CONTROLLER_STATUS)
		print("Viewer subscribed to telemetry and controller status...")

		async for message in client.messages:
			payload = json.loads(message.payload)

			if str(message.topic) == config.TOPIC_CONTROLLER_STATUS:
				controller_precedence_q = payload.get("precedence_queue", [])
				continue

			vehicle						= Vehicle(**payload)
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
	# Connect to broker and send the pause/resume command
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client: # adjust hostname if needed
		payload = json.dumps({"command": cmd.command.value})
		await client.publish("system/control", payload=payload)
	return {"status": "ok", "command": cmd}


@app.get("/api/state")
async def get_state():
	return {
        "vehicles"			: vehicles_state,
        "precedence_queue"	: controller_precedence_q,
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