import serial
import time

ser = serial.Serial('/dev/tty.usbserial-A800JDPN', 38400,
                    timeout=0.5, parity=serial.PARITY_NONE
                    )  # open serial port
print(ser.name)         # check which port was really used



"""
ser = serial.Serial()
ser.baud=38400
ser.timeout=2
ser.parity=serial.PARITY_NONE

ser.port = '/dev/tty.usbserial-A800JDPN'
ser.open()
"""

get1 = b'\xff\xff\x01\x04\x02\x24\x02\xd2'
ping = b'\xff\xff\x01\x02\x01\x00'

def send(id):
	s = list(get1)
	s[2] = id
	s[7] = (sum(s[2:7]) & 0xff) ^ 0xff
	s = bytes(s)
	print('getting %d' % id, s)
	ser.write(s)
	resp = ser.read(8)
	if len(resp) > 0:
		print('  response', list(resp))
		return True
	return False


#print('is open %d' % ser.is_open)
#print('fd %d' % ser.fileno())
#z = ser.get_settings()
#time.sleep(1)
#ser.write(b'\xff')
#ser.flush()
#ser.read(1)

import fcntl
import os

z = fcntl.fcntl(ser.fileno(), fcntl.F_GETFL)
print('fcntl %x %x %x' % (z, os.O_NONBLOCK, os.O_RDWR))
fcntl.fcntl(ser.fileno(), fcntl.F_SETFL, os.O_RDWR)
print('is open %d' % ser.is_open)

# import select

# print('select')
# fdsets = select.select([], [ser.fileno()], [], 10)
# print('done')

while not send(1):
	pass


print('in %d, out %d' % (ser.in_waiting, ser.out_waiting))

for id in range(1, 6):
	send(id)

for id in range(1, 6):
	send(id)

time.sleep(0.5)
for c in get1:
	ser.write(c)



ser.close()             # close port
