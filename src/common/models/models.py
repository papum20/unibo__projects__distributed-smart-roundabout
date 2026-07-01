from pydantic import BaseModel



class Position(BaseModel):
	x	: float		# m
	y	: float		# m


class VehicleState(BaseModel):
	id		: str
	pos		: Position
	speed	: float		# m/s