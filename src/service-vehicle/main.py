import asyncio
import json
import logging
import random
import time

import aiomqtt
from uuid import uuid4

from common import math_utils, physics
from common.const import (
	CAR_VISION_RADIUS_M,
	TIMER_NETWORK_TIMEOUT,
	UPDATES_P_S_VEHICLE,
	AREA_RADIUS,
	ROUNDABOUT_POS,
)
from common.get_env import config
from common.models.models import (
	Command, Position, SystemCommand, SystemCommandValue
)
from common.models.vehicle import (
	Vehicle, VehicleNavState, VehiclePosition, VehicleState, 
)
from common.vehicle import vehicle_navigate
from .logic import (
	evaluate_failsafe,
	vehicle_navigate_spawn,
	vehicle_reset
)



logger = logging.getLogger(__name__)

TIMER_NETWORK_FAILSAFE_S	= TIMER_NETWORK_TIMEOUT


vehicle_id = str(uuid4())
logger.info("Generated vehicle_id: %s", vehicle_id)

class RuntimeState:
	def __init__(self):
		self.vehicle = Vehicle(
			id			= vehicle_id,
			pos			= Position(x=45.0, y=9.0),
			pos_angle	= 0.0,
			speed		= 10.0
		)

		# global variables for network state
		self.last_command			= Command(target_acceleration=0.0)
		self.last_net_update_time	= time.time()

		self.sysctrl_pause			= False
		self.sysctrl_failsafe		= False
		self.sysctrl_disconnected	= False

		# v_id -> (VehiclePosition, timestamp)
		self.local_vision_vehicles: dict[str, VehiclePosition] = {}

state = RuntimeState()



async def loop_listen_commands(client, s: RuntimeState = state):
	command_topic			= f"{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_COMMAND_SUFFIX}"
	positions_topic			= f"{config.TOPIC_VEHICLE_PREFIX}/+/{config.TOPIC_VEHICLE_POSITION_SUFFIX}"
	sysctrl_topic			= f"{config.TOPIC_SYSCTRL_PREFIX}/{config.TOPIC_SYSCTRL_BROADCAST_SUFFIX}"
	sysctrl_broadcast_topic = f"{config.TOPIC_SYSCTRL_PREFIX}/{vehicle_id}"

	await client.subscribe(command_topic)
	await client.subscribe(positions_topic)
	await client.subscribe(sysctrl_topic)
	await client.subscribe(sysctrl_broadcast_topic)
	
	async for message in client.messages:
		payload = json.loads(message.payload)

		if str(message.topic) in (sysctrl_topic, sysctrl_broadcast_topic):
			command = SystemCommand(**payload)
			if command.command == SystemCommandValue.PAUSE:
				s.sysctrl_pause = True
				logger.info("SysCtrl: Simulation PAUSED")
			elif command.command == SystemCommandValue.RESUME:
				s.sysctrl_pause = False
				s.last_net_update_time = time.time()	# prevent instant failsafe
				logger.info("SysCtrl: Simulation RESUMED")
			elif command.command == SystemCommandValue.ENTER_FAILSAFE:
				s.sysctrl_failsafe = True
				logger.info("SysCtrl: ENTER FAILSAFE")
			elif command.command == SystemCommandValue.EXIT_FAILSAFE:
				s.sysctrl_failsafe = False
				logger.info("SysCtrl: EXIT FAILSAFE")
			elif command.command == SystemCommandValue.ENTER_DISCONNECTED:
				s.sysctrl_disconnected = True
				logger.info("SysCtrl: ENTER DISCONNECTED")
			elif command.command == SystemCommandValue.EXIT_DISCONNECTED:
				s.sysctrl_disconnected = False
				logger.info("SysCtrl: EXIT DISCONNECTED")

		elif message.topic.matches(positions_topic):
			v_pos = VehiclePosition(**payload)
			if v_pos.id != vehicle_id and math_utils.get_dist(v_pos.pos, s.vehicle.pos) < CAR_VISION_RADIUS_M:
				v_pos.timestamp = time.time()
				s.local_vision_vehicles[v_pos.id] = v_pos

		elif message.topic.matches(command_topic):
			s.last_command			= Command(**payload)
			s.last_net_update_time	= time.time()
			logger.debug("Received command: %s", s.last_command)


async def loop_publish_vision(client, s: RuntimeState = state):
	while True:
		current_time = time.time()
		# remove old vehicles from vision
		s.local_vision_vehicles = {
			v_id: v_pos
			for v_id, v_pos in s.local_vision_vehicles.items()
			if v_pos.timestamp and current_time - v_pos.timestamp < TIMER_NETWORK_TIMEOUT
		}

		vision_data = [
			v_pos.model_dump()
			for v_pos in s.local_vision_vehicles.values()
		]
		vision_topic = f"{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_VISION_SUFFIX}"
		await client.publish(vision_topic, payload=json.dumps(vision_data))
		
		await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)



async def loop_physics(client, s: RuntimeState = state):
	last_time = time.time()
	
	while True:
		current_time	= time.time()
		dt				= current_time - last_time
		last_time		= current_time

		if s.sysctrl_pause:
			# update the failsafe timer so it doesn't trigger during pause
			s.last_net_update_time = current_time 
			await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)
			continue

		spawn_command = vehicle_navigate_spawn(s.vehicle, list(s.local_vision_vehicles.values()))
		if spawn_command is not None:
			active_command	= spawn_command

		else:
			# check for command, otherwise failsafe
			if s.sysctrl_failsafe or s.sysctrl_disconnected or current_time - s.last_net_update_time > TIMER_NETWORK_FAILSAFE_S:
				if s.sysctrl_disconnected:
					s.vehicle.state = VehicleState.DISCONNECTED
				else:
					s.vehicle.state = VehicleState.FAILSAFE
				active_command	= evaluate_failsafe(s.vehicle, list(s.local_vision_vehicles.values()))
				if not s.sysctrl_failsafe:
					logger.warning("Network lost. FAILSAFE triggered.")
			else:
				s.vehicle.state	= VehicleState.NORMAL
				active_command	= s.last_command

		s.vehicle.acceleration = active_command.target_acceleration
		s.vehicle.speed = physics.update_speed(
			speed		= s.vehicle.speed, 
			acc			= s.vehicle.acceleration, 
			dt			= dt, 
			max_speed	= s.vehicle.params.max_speed
		)
		s.vehicle = vehicle_navigate(dt, s.vehicle, logger=logger)

		if s.vehicle.nav_state == VehicleNavState.EXITING and math_utils.get_dist(s.vehicle.pos, ROUNDABOUT_POS) > AREA_RADIUS:
			vehicle_reset(s.vehicle)
		v_pos = s.vehicle.to_pos()

		telemetry_topic	= f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}'
		await client.publish(telemetry_topic, payload=s.vehicle.model_dump_json())
		logger.debug("Published to topic %s: %s", f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_TELEMETRY_SUFFIX}', s.vehicle.model_dump_json())
		pos_topic		= f'{config.TOPIC_VEHICLE_PREFIX}/{vehicle_id}/{config.TOPIC_VEHICLE_POSITION_SUFFIX}'
		await client.publish(pos_topic, payload=v_pos.model_dump_json())
		logger.debug("Published to topic %s: %s", pos_topic, v_pos.model_dump_json())

		await asyncio.sleep(1.0 / UPDATES_P_S_VEHICLE)


async def main(s: RuntimeState = state):
	vehicle_reset(s.vehicle)
	# random boot delay so cars don't overlap on startup
	await asyncio.sleep(random.uniform(0.0, 5.0))
	
	async with aiomqtt.Client(hostname=config.HOST_BROKER, port=config.PORT_BROKER) as client:
		# run both concurrently
		await asyncio.gather(
			loop_listen_commands(client, s),
			loop_publish_vision(client, s),
			loop_physics(client, s)
		)



if __name__ == "__main__":
	asyncio.run(main())