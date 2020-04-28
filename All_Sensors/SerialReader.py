import serial
import time

s = serial.Serial('COM4', 9600)
time.sleep(2)

for i in range(50):
	bstr = s.readline()
	s_str = bstr.decode()
	s_str = s_str.rstrip() # remove \n and \r

	print(s_str)
	time.sleep(0.1)            # wait (sleep) 0.1 seconds

ser.close()
