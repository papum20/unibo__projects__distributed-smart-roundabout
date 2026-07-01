import asyncio
import logging
import aiomqtt
from uuid import uuid4

from common.get_env import config
from common.models.models import Position, VehicleState



logger = logging.getLogger(__name__)

TIMER_STATE = 1.0	# seconds


vehicle_id = str(uuid4())
logger.info("Generated vehicle_id: %s", vehicle_id)

vehicle = VehicleState(
	id		= vehicle_id,
	pos		= Position(x=45.0, y=9.0),
	speed	= 10.0
)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		while True:

			await client.publish(f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', payload=vehicle.model_dump_json())
			logger.info("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', vehicle.model_dump_json())

			await asyncio.sleep(TIMER_STATE)


asyncio.run(main())