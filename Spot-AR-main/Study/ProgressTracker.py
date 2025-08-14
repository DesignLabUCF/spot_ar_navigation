##################################
#   Experiment Progress Tracker  #
#     SENSEable Design Lab       #
##################################
# v1.0
# 1/12/2024
##################################
# No arguments required
# EXP: 'python ProgressTracker.py'
##################################
# Authors: 
# Sermarini
##################################


import sys
import os
import csv
import threading
from time import sleep
from datetime import datetime
from tkinter import *
from tkinter import messagebox

# Scripts params
TIME_FORMAT = "%m_%d_%Y-%H_%M_%S.%f"
NUM_LAPS = 9
control_groups = ["AR", "Tablet", "VR"]
environment_groups = ["Inside", "Outside"]
# Globals
active_thread = None
save_text = None
stopwatch_buttons = []
stopwatch_active = False
start_time = None
lap_times = []
lap_ends = []
lap_starts = []
lap_time_labels = []
#lap_reverses = []
lap_directions = ["Forward", "Forward", "Forward", "Backward", "Backward", "Backward", "Combined", "Combined", "Combined"]
active_lap = None
'''
control_group = None
environment_group = None
'''
control_var = None
environment_var = None

class stopwatch_thread(threading.Thread):
	def __init__(self, lap):
		threading.Thread.__init__(self)
		self.name = lap
		self.lap = int(lap)
		self.running = False
		self.start_time = None         
	def run(self):
		self.running = True
		self.start_time = datetime.now()
		print("Starting stopwatch")
		while self.running:
			#print('running ' + self.name)
			update_lap_time(self.lap, datetime.now() - self.start_time, self.start_time, False)
			#sleep(0.1)     
	def get_id(self):
		if hasattr(self, '_thread_id'):
			return self._thread_id
		for id, thread in threading._active.items():
			if thread is self:
				return id
	def stop(self):
		self.running = False
		print("Stopping stopwatch")
		return datetime.now() - self.start_time, self.start_time

'''
class ConditionGroup():
	def __init__(self, window, name, groups):
		self.window = window
		self.name = name
		self.groups = groups
		self.checkboxes = []
		self.checkboxVars = []
		self.currently_selected = -1
		self.make_checkboxes()
	def select(self, index):
		self.currently_selected = index
		for i in range(0, len(self.checkboxes)):
			if(i == index):
				continue
			self.checkboxVars[i].set(False)
	def make_checkboxes(self):
		for cond in self.groups:
			self.checkboxVars.append(IntVar())
			cb = Checkbutton(self.window, text=cond,variable=self.checkboxVars[len(self.checkboxVars)-1], onvalue=1, offvalue=0, command=lambda x = len(self.checkboxVars)-1, : self.select(x))
			self.checkboxes.append(cb)
		return self.checkboxes
	def get_checkboxes(self):
		return self.checkboxes
	def get_selected_group(self):
		if self.currently_selected == -1:
			return None
		return self.groups[self.currently_selected]
'''

def save_data(is_manual_save):
	# Get save type from input
	save_type = "MANUAL" if is_manual_save else "AUTO"
	print("Initiating " + save_type + " save.")
	# Get subject id from input
	subject_id = None
	if save_text != None:
		subject_id = save_text.get("1.0","end-1c")
	# Check text box input of subject ID
	if subject_id == None or subject_id == "":
		messagebox.showerror('Error', 'Error: No subject ID inputted to top text box. Please enter an ID before saving. Save not complete.')
		print("ERROR: No subject ID inputted to top text box. Please enter an ID before saving. Save not complete.")
		return
	'''
	# Check study condition group inputs
	if control_group == None or control_group.get_selected_group() == None:
		messagebox.showerror('Error', 'Error: No control group has been selected. Please ensure a checkbox is selected. Save not complete.')
		print('Error', 'Error: No control group has been selected. Please ensure a checkbox is selected. Save not complete.')
		return
	if environment_group == None or environment_group.get_selected_group() == None:
		messagebox.showerror('Error', 'Error: No environment group has been selected. Please ensure a checkbox is selected. Save not complete.')
		print('Error', 'Error: No control group has been selected. Please ensure a checkbox is selected. Save not complete.')
		return
	'''
	# Get study conditions
	'''
	subject_type = control_group.get_selected_group()
	study_env = environment_group.get_selected_group()
	'''
	subject_type = control_groups[control_var.get()]
	study_env = environment_groups[environment_var.get()]
	# Get directory and datetime info
	now = datetime.now()
	file_directory = os.path.join("Subjects", subject_id, "ProgressTracker")
	#file_name = "ProgressTracker_" + now.strftime("%d_%m_%Y-%H_%M_%S_%f")
	file_name = save_type + "_" + now.strftime(TIME_FORMAT)
	full_file = os.path.join(file_directory, file_name) + ".csv"
	# If needed, create subject directory
	if not os.path.isdir(file_directory):
		print("Subject directory not found. " + file_directory + " created.")
		os.makedirs(file_directory)	
	# Create CSV
	print("Creating " + full_file + "...")
	with open(full_file, 'w', newline='') as csvfile:
		writer = csv.writer(csvfile)
		# Comments (Subject ID, condition, time of creation)
		writer.writerow([ \
			"Subject ID:",
			subject_id, 
			"Type:",
			subject_type,
			"Environment:",
			study_env, 
			"Time of creation:",
			now.strftime(TIME_FORMAT)])
		# Headers
		writer.writerow([ \
			"Participant.ID", \
			"Control", \
			"Complexity", \
			"Lap", \
			"Time", \
			"End_Timestamp", \
			"Start_Timestamp", \
			"Route"
			])
		for i in range(0, NUM_LAPS):
			writer.writerow([ \
				subject_id,
				subject_type,
				study_env,
				i + 1,
				lap_times[i],
				lap_ends[i],
				lap_starts[i],
				#lap_reverses[i].get(),
				lap_directions[i],
			])
	print(full_file + " generated!")

def trigger_stopwatch(lap):
	global stopwatch_active
	global active_thread
	global active_lap
	#global stopwatch_buttons

	if stopwatch_active:
		end_time, start_time = active_thread.stop()
		update_lap_time(active_lap, end_time, start_time, True)
		save_data(False)
		# Update internal tracking values
		stopwatch_active = False
		# Update display
		[ x.config(text = "Start") for x in stopwatch_buttons ]
	else:
		active_thread = stopwatch_thread(lap)
		active_thread.start()
		# Update internal tracking values
		stopwatch_active = True
		active_lap = lap
		# Update display
		[ x.config(text = "Stop") for x in stopwatch_buttons ]


def update_lap_time(lap, end_time, start_time, should_save):
	global lap_times
	global lap_ends
	global lap_starts
	global lap_time_labels

	lap_index = lap - 1
	# Update internal value
	lap_times[lap_index] = end_time.total_seconds()
	lap_ends[lap_index] = datetime.now().strftime(TIME_FORMAT)
	lap_starts[lap_index] = start_time.strftime(TIME_FORMAT)
	# Update GUI label
	lap_time_labels[lap_index].config(text = end_time.total_seconds())
	# Update saved values
	if should_save:
		print(lap_times)

def main(argv):
	global lap_times
	global lap_ends
	global lap_starts
	global lap_time_labels
	global save_text
	global stopwatch_buttons
	'''
	global control_group
	global environment_group
	'''
	global control_var
	global environment_var

	## Init params
	lap_times = [-1] * NUM_LAPS
	lap_ends = [-1] * NUM_LAPS
	lap_starts = [-1] * NUM_LAPS
	## Init GUI
	window = Tk()
	## Configure GUI
	window.title("Study Tracker")
	#window.geometry("1200x800")
	window.resizable(width=False, height=False)
	window.wm_iconbitmap("icon.ico")
	# Create GUI frame
	frame = Frame(window)
	# Create GUI header row/save button
	save_label = Label(window, text="Subject ID: ")
	save_label.config(bg= "gold")
	save_label.grid(column=0, row=0, sticky="EW")
	save_text = Text(window, height=1, width=10)
	#save_text.config(bg= "gold")
	save_text.grid(column=1, row=0, sticky=W)
	save_button = Button(window, text="MANUAL SAVE PROGRESS", command=lambda : save_data(True)) # x=task from https://stackoverflow.com/questions/4236182/generate-tkinter-buttons-dynamically
	save_button.grid(column=2, row=0, sticky="NESW", columnspan=2)
	# Create study control type row
	'''
	control_label = Label(window, text="Control Type: ")
	control_label.grid(column=0, row=1, sticky="E")
	control_group = ConditionGroup(window, "control_type", ["AR", "Controller", "Tablet"])
	control_checkboxes = control_group.get_checkboxes()
	for i in range(0, len(control_checkboxes)):
		control_checkboxes[i].grid(column=i + 1, row=1, sticky="W")
	'''
	control_label = Label(window, text="Control Type: ")
	control_label.grid(column=0, row=1, sticky="E")
	control_label.config(bg= "gold")
	control_var = IntVar()
	for i in range(0, len(control_groups)):
		control_radio = Radiobutton(window, text=control_groups[i], variable=control_var, value=i)
		control_radio.grid(column=i + 1, row=1, sticky="W")
	# Create study environment row
	'''
	environment_label = Label(window, text="Environment: ")
	environment_label.grid(column=0, row=2, sticky="E")
	environment_group = ConditionGroup(window, "environment", ["Inside", "Outside"])
	environment_checkboxes = environment_group.get_checkboxes()
	for i in range(0, len(environment_checkboxes)):
		environment_checkboxes[i].grid(column=i + 1, row=2, sticky="W")
	'''
	environment_label = Label(window, text="Environment: ")
	environment_label.grid(column=0, row=2, sticky="E")
	environment_label.config(bg= "gold")
	environment_var = IntVar()
	for i in range(0, len(environment_groups)):
		environment_radio = Radiobutton(window, text=environment_groups[i], variable=environment_var, value=i)
		environment_radio.grid(column=i + 1, row=2, sticky="W")
	# Create sections for each lap
	for lap in range(1, NUM_LAPS + 1):
		# Text labels
		row = 2 + lap
		id_label = Label(window, text=f"Lap {lap}")
		id_label.grid(column=0, row=row, sticky="NESW")
		time_label = Label(window, text="0.000000")
		time_label.grid(column=1, row=row, sticky="NESW")
		lap_time_labels.append(time_label)
		# Stopwatch trigger button
		stopwatch_button = Button(window, text="Start", command=lambda x = lap, : trigger_stopwatch(x))
		stopwatch_button.grid(column=2, row=row, sticky="NESW")
		stopwatch_buttons.append(stopwatch_button)
		# Direction checkbox
		'''
		lap_reverses.append(IntVar())
		direction_checkbox = Checkbutton(window, text='Reverse?',variable=lap_reverses[lap - 1], onvalue=1, offvalue=0, command=lambda x = lap: save_data(False))
		direction_checkbox.grid(column=3, row=row, sticky="NESW")
		'''
		direction_label = Label(window, text=lap_directions[lap - 1])
		direction_label.grid(column=3, row=row, sticky="NESW")

	# Run GUI
	window.mainloop()





if __name__ == "__main__":
	main(sys.argv[1:])