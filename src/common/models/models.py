from enum import Enum

from pydantic import BaseModel, Field




class Position(BaseModel):
	x	: float		# m
	y	: float		# m


class Command(BaseModel):
	target_acceleration: float	# m/s^2

class SystemCommandValue(Enum):
	PAUSE				= "PAUSE"
	RESUME				= "RESUME"
	ENTER_FAILSAFE		= "ENTER_FAILSAFE"
	EXIT_FAILSAFE		= "EXIT_FAILSAFE"
	ENTER_DISCONNECTED	= "ENTER_DISCONNECTED"
	EXIT_DISCONNECTED	= "EXIT_DISCONNECTED"

class SystemCommand(BaseModel):
	command			: SystemCommandValue
	vehicle_id		: str | None = None
	vehicle_count	: int | None = Field(default=None, ge=1, le=99)
	