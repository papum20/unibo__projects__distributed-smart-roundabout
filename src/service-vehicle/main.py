import asyncio
import aiomqtt

from common.get_env import config
from common.models.models import Vehicle



vehicle = Vehicle(
    id		= 1,
    speed	= 10.0
)



async def main():
    async with aiomqtt.Client(config.URL_BROKER) as client:
        await client.publish(config.TOPIC_VEHICLE, payload=vehicle.model_dump(mode='json'))


asyncio.run(main())