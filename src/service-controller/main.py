import asyncio
import logging
import aiomqtt

from common.get_env import config
from common.models.models import Vehicle



logger = logging.getLogger(__name__)



async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}')
		  
		async for message in client.messages:
			logger.info("Received on topic %s: %s", message.topic, message.payload)

			if message.topic.matches(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}'):
				pass



asyncio.run(main())