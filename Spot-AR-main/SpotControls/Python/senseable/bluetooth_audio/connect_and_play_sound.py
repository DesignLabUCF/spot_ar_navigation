import sys
from datetime import datetime
import time
import subprocess
import bluetooth
#import pyaudio
import multiprocessing # Used over thread because we can kill these at will, which is likely needed mid-audio play
import logging
from bosdyn.client.util import setup_logging


#TEST_AUDIO_PATH = "/test_audio.mp3"
TEST_AUDIO_PATH = "/beep.mp3"
_LOGGER = logging.getLogger('bluetooth_plugin')

current_audio_process = None


# Play audio in console using mpg123. Best used when launched as it's own thread
# TODO it is not exiting the process for some reason, need to fix
def play_audio_console(audio_path):
	try:
		#run_cmd = f"mpg123 -f 8000 {audio_path}"
		run_cmd = ['mpg123', '-f', '8000', audio_path] # https://stackoverflow.com/questions/18962785/oserror-errno-2-no-such-file-or-directory-while-using-python-subprocess-wit
		#subprocess.run(run_cmd, shell=False, stdout=subprocess.PIPE)
		subprocess.Popen(run_cmd, shell=False, stdout=subprocess.PIPE)
	except Exception as e:
		_LOGGER.info(e)
		return False
	return True

def still_connected(mac_address):
	# https://stackoverflow.com/questions/40325218/python-bluetooth-check-connection-status
	stdoutdata = subprocess.getoutput("hcitool con")
	if mac_address in stdoutdata.split():
		return True
	else:
		return False

def main():
	setup_logging()

	try:
		# Discover and read in device info
		_LOGGER.info("Discovering bluetooth devices for 10 seconds...")
		nearby_devices = bluetooth.discover_devices(duration=10, lookup_names=True)
		# TODO - Account for no bluetooth dongle found - "[Errno 19] No such device"
		if len(nearby_devices) > 0:
			_LOGGER.info("Discovered the following devices in Python:")
			primary_device = None
			for d in nearby_devices:
				_LOGGER.info(d)
				if "Prime" in d[1]:
					_LOGGER.info("Python found device: " + d[1])
					primary_device = d
					break
			'''
				Connect to device using subprocess and play looping audio
				https://stackoverflow.com/questions/89228/how-do-i-execute-a-program-or-call-a-system-command
			'''
			# Read device info
			#primary_device = nearby_devices[0]			
			device_mac_address = primary_device[0]
			device_name = primary_device[1]

			# Remove device if already connected for some reason (if not connected, should still run command and have no real issues)
			run_arg = f"bluetoothctl remove {device_mac_address}"
			connect_results = subprocess.run(
				[run_arg],
				capture_output=True,
				text=True,
				#stdout=subprocess.PIPE,
				shell=True)

			# Pair and connect with device
			run_arg = f"bluetoothctl --timeout 10 scan on && bluetoothctl trust {device_mac_address} && bluetoothctl pair {device_mac_address} && bluetoothctl connect {device_mac_address}"
			connect_results = subprocess.run(
				[run_arg],
				capture_output=True,
				text=True,
				#stdout=subprocess.PIPE,
				shell=True)
			_LOGGER.info(connect_results.stdout)

			if not ("Connection successful" in connect_results.stdout):
				raise Exception("Bluetooth connecting failed. Exiting")

			'''
			connect_results = subprocess.run(['bluetoothctl', 'paired-devices'], capture_output=True, text=True)
			_LOGGER.info(connect_results.stdout)
			if (device_mac_address in connect_results.stdout) == False:
				raise Exception("Pairing with '" + device_name + "' failed. Exiting")
			'''

			# Play confirm sound - https://stackoverflow.com/questions/36894315/how-to-select-a-specific-input-device-with-pyaudio
			'''
			_LOGGER.info("Playing audio at path: " + TEST_AUDIO_PATH)
			p = pyaudio.PyAudio()
			for i in range(0, p.get_device_count()):
				connected_device = p.get_device_info_by_index(i)
			'''

			# TODO If looping, periodically check if device still connected. Restart program if it isnt?
			while True:
				_LOGGER.info(datetime.now().strftime("%m-%d-%Y_%H-%M-%S-%f")[:-3])
				if still_connected(device_mac_address):
					# Play audio in console as test
					current_audio_process = multiprocessing.Process(target=play_audio_console, args=(TEST_AUDIO_PATH,))
					current_audio_process.daemon = True
					current_audio_process.start()
					time.sleep(5)
					#current_audio_process.terminate()
					current_audio_process.terminate()
					time.sleep(0.5)
				else:
					raise Exception("Connection to device was terminated. Exiting")	
		else:
			raise Exception("No devices detected. Exiting")
	except Exception as e:
		_LOGGER.info(e)
		# TODO kill multiprocess if running
		return
	finally:
		pass


if __name__ == '__main__':
	main()