import asyncio
import logging
import aiomqtt
from uuid import uuid4

from common.get_env import config
from common.models.models import Vehicle



logger = logging.getLogger(__name__)


vehicle_id = str(uuid4())
logger.info("Generated vehicle_id: %s", vehicle_id)

vehicle = Vehicle(
	id		= vehicle_id,
	speed	= 10.0
)


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.publish(f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', payload=vehicle.model_dump_json())
		logger.info("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', vehicle.model_dump_json())


asyncio.run(main())