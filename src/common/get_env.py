import os
import sys
import logging

from dotenv import load_dotenv



load_dotenv() 

# Configure Logging
logging.basicConfig(level=logging.INFO)



def get_env_or_exit(key: str) -> str:
	val = os.environ.get(key)
	if val is None:
		print(f"Missing required environment variable (make sure .env is configured in the parent directory): {key}", file=sys.stderr)
		sys.exit(2)
	return val



class Config:

	# Read Configuration from Docker Environment
	URL_BROKER				= get_env_or_exit("URL_BROKER")
	URL_BROKER_WEBSOCKET	= get_env_or_exit("URL_BROKER_WEBSOCKET")
	URL_CONTROLLER			= get_env_or_exit("URL_CONTROLLER")

	TOPIC_VEHICLE			= get_env_or_exit("TOPIC_VEHICLE")

	logger = logging.getLogger(__name__)


config = Config()
