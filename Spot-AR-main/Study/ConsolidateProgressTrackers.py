##################################
#  Consolidate Progress Trackers #
#     SENSEable Design Lab       #
##################################
# v1.0
# 4/15/2023
##################################
# Pass in nothing
# EXP: 'python ConsolidateProgressTrackers.py'
##################################
# Authors: 
# Sermarini
##################################

import sys
import os
import csv
import pandas as pd

directory_path = "Subjects"
output_filename = "ConsolidatedProgressTrackers.csv"

subject_dataframes = []


def main(argv):
	subject_dirs = os.listdir(directory_path)
	# Remove non subject folders safe check
	for i in range(len(subject_dirs) - 1, 0, -1):
		if not os.path.isdir(os.path.join(directory_path, subject_dirs[i])):
			subject_dirs.pop(i)
	if ".gitkeep" in subject_dirs: # Github management file
		subject_dirs.remove(".gitkeep") 
	# Print debug log to console
	print("Subjects Directories: ")
	print(f"Found Subjects: {subject_dirs}")
	# Iterate through each subject file
	subject_data = []
	for subject_dir in subject_dirs:
		print(f"Subject {subject_dir}")
		file_found = False
		subject_path = os.path.join(directory_path, subject_dir, "ProgressTracker")
		print(f".Path: {subject_path}")
		manual_save_files = []
		auto_save_files = []
		for file in os.listdir(subject_path):
			file = os.path.join(subject_path, file)
			print(f"..File: {file}")
			# Find last updated file
			if "ProgressTracker_AUTO" in file:
				auto_save_files.append(file)
			elif "ProgressTracker_MANUAL" in file:
				manual_save_files.append(file)
		# Pull latest autosave file
		if(len(auto_save_files) <= 0):
			continue
		auto_save_files.sort(reverse=True)
		file = auto_save_files[0]
		print(f"...Latest Autosave File: {file}")
		# Parse file
		df = pd.read_csv(file, skiprows=1)
		subject_dataframes.append(df)
		print(df)
	# Combine all
	if(len(subject_dataframes) == 0):
		print("No valid dataframes found in subjects folders. Exiting unsuccesfully...")
		return
	consolidated = pd.concat(subject_dataframes)
	consolidated = consolidated.loc[:, ~consolidated.columns.str.contains('^Unnamed')] # Remove waste column created by metadata
	print(consolidated)
	output_path = os.path.join(directory_path, output_filename)
	print("Creating " + output_path + "...")
	consolidated.to_csv(output_path)
	print(output_path + " succesfully created!")





if __name__ == "__main__":
	main(sys.argv[1:])