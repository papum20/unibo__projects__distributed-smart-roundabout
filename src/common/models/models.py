from pydantic import BaseModel


class Vehicle(BaseModel):
	id:		str
	speed:	float	# m/s