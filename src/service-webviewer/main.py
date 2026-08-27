import asyncio
import logging
import json
from contextlib import asynccontextmanager

import aiomqtt
from fastapi import FastAPI
from fastapi.responses import HTMLResponse
import uvicorn

from common.get_env import config
from common.models.models import VehicleState



logger = logging.getLogger(__name__)



# In-memory dictionary to store the latest state of all cars
vehicles_state = {}


async def mqtt_listener():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}')
		print("Viewer subscribed to telemetry...")
		async for message in client.messages:
			payload						= json.loads(message.payload)
			vehicle						= VehicleState(**payload)
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



@app.get("/api/state")
async def get_state():
	return vehicles_state


@app.get("/")
async def serve_frontend():
	with open("index.html", "r") as f:
		return HTMLResponse(content=f.read())



if __name__ == "__main__":
	uvicorn.run(app, host="0.0.0.0", port=config.PORT_WEBVIEWER)