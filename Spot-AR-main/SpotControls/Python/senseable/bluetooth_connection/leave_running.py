import sys
from datetime import datetime
import time

import logging
from bosdyn.client.util import setup_logging


_LOGGER = logging.getLogger('bluetooth_plugin')


def main():
	setup_logging()
	while True:
		log_str = datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3]
		_LOGGER.info(log_str)
		time.sleep(0.05)


if __name__ == '__main__':
	main()