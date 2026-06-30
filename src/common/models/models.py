from pydantic import BaseModel


class Vehicle(BaseModel):
	id:		int
	speed:	float	# m/s