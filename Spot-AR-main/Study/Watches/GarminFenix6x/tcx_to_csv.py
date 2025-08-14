import xml.etree.ElementTree as ET
import pandas as pd 

def tcx_to_df(filename_tcx):
	#in_filename = "data\\UCF.Garmin.009\\2023-04-30\\UCF.Garmin.009-datalog-activity-2023-04-30-SESSION-1.tcx"
	#out_filename = filename_tcx.replace(".tcx", "_CONV.csv")

	df = pd.DataFrame(columns=['timestamp', 'heart_rate'])

	tree = ET.parse(filename_tcx)
	root = tree.getroot()
	activity_root = root[0][0]

	start_time = activity_root[0].text
	#print(start_time)

	lap_root = activity_root[1]
	total_time = lap_root[0].text
	#print(total_time)
	track_root = lap_root[7]
	for i in track_root:
		timestamp = i[0].text
		heart_rate = i[1][0].text
		#print(timestamp, " - ", heart_rate)
		df = df._append({'timestamp': timestamp, 'heart_rate': heart_rate}, ignore_index=True)

	return(df)
	#df.to_csv(out_filename, index=False)	

