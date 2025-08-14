from time import sleep
import threading

test_val = 0
test_val_list = []

def inc_test_val():
	global test_val
	for i in range(0, 100):
		test_val = test_val + 1
		sleep(0.05)


def inc_test_val_list():
	global test_val_list
	for i in range(0, 100):
		test_val_list.append(i)
		sleep(0.05)

'''
input_thread = threading.Thread(target=inc_test_val)
input_thread.start()
while test_val < 50:
	print(test_val)
'''

'''
input_thread = threading.Thread(target=inc_test_val_list)
input_thread.start()
while len(test_val_list) < 50:
	print(test_val_list)
'''

def main():
	global test_val_list

	'''
	input_thread = threading.Thread(target=inc_test_val_list)
	input_thread.start()
	while len(test_val_list) < 50:
		print(test_val_list)
	'''
	input_thread = threading.Thread(target=inc_test_val_list)
	input_thread.start()
	input_thread2 = threading.Thread(target=inc_test_val_list)
	input_thread2.start()
	while len(test_val_list) < 50:
		print(test_val_list)

if __name__ == '__main__':
	main()			