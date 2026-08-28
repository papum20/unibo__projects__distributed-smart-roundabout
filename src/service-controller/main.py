import asyncio
import logging
import aiomqtt

from common import math_utils
from common.const import ROUNDABOUT_POS
from common.get_env import config
from common.models.models import Command, Vehicle



logging.basicConfig(level=logging.INFO if not config.DEBUG_MODE else logging.DEBUG)
logger = logging.getLogger(__name__)



def evaluate_traffic(vehicles: list[Vehicle], safe_distance: float = 15.0) -> dict[str, Command]:
	"""
	Take a list of all current vehicles.
	@return a dictionary mapping vehicle_id -> Command.
	"""
	# default: tell everyone to maintain speed
	commands = {v.id: Command(target_acceleration=0.0) for v in vehicles}
	
	if len(vehicles) < 2:
		# only one car, no collisions possible
		return commands
		
	# Sort vehicles by distance to the roundabout center (closest first).
	# The car closest to the center is at the front of the line.
	sorted_vehicles = sorted(vehicles, key=lambda v: math_utils.get_dist(v.pos, ROUNDABOUT_POS))
	
	# check distances between consecutive cars
	for i in range(len(sorted_vehicles) - 1):
		v_ahead		= sorted_vehicles[i]
		v_behind	= sorted_vehicles[i+1]
		
		dist_ahead	= math_utils.get_dist(v_ahead.pos, ROUNDABOUT_POS)
		dist_behind	= math_utils.get_dist(v_behind.pos, ROUNDABOUT_POS)
		
		# if the car behind is too close to the car ahead, tell it to brake
		if (dist_behind - dist_ahead) < safe_distance:
			commands[v_behind.id] = Command(target_acceleration=-2.0)
			
	return commands


async def main():
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		await client.subscribe(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}')
		  
		while True:
		  
			async for message in client.messages:
				logger.debug("Received on topic %s: %s", message.topic, message.payload)

				if message.topic.matches(f'{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}'):
					pass



if __name__ == "__main__":
	asyncio.run(main())