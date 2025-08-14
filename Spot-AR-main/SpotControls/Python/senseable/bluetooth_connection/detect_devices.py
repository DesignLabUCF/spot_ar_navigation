import sys
from datetime import datetime
import time

import logging
from bosdyn.client.util import setup_logging

import bluetooth


_LOGGER = logging.getLogger('bluetooth_plugin')


def main():
	setup_logging()

	try:
		_LOGGER.info("Discovering bluetooth devices for 10 seconds...")
		nearby_devices = bluetooth.discover_devices(duration=10, lookup_names=True)
		if len(nearby_devices) > 0:
			_LOGGER.info("Discovered the following devices:")
			for d in nearby_devices:
				_LOGGER.info(d)
			primary_device = nearby_devices[0]

			# Replace this with subproccess to connect and run whatever stuff on the bluetooth device
			while True:
				_LOGGER.info(datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3])
				_LOGGER.info(primary_device)
				time.sleep(0.5)
		else:
			raise Exception("No devices detected. Exiting.")
	except Exception as e:
		_LOGGER.info(e)
		return
	finally:
		pass


if __name__ == '__main__':
	main()