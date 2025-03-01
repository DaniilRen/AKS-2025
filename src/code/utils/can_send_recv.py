import can
import struct
import time

bus = can.interface.Bus('can0', bustype='socketcan', fd=True)  


def send_message(data):
	msg = can.Message(arbitration_id=0x065, data=struct.pack('i', 180)) 
	bus.send(msg)


def receive_message():
	msg = bus.recv(timeout=0.1)  
	if msg is not None:
		if msg.arbitration_id == 18:
			try:
				data = str(msg).split()
				data = [int(i, 16) for i in data[-6:-2]]
				print(data)
			except struct.error as e:
				print("Error unpacking data:", e)
	else:
		print("No message received.")


last_send_time = time.time()
while True:
	if time.time() - last_send_time > 1:  
		send_message()
		last_send_time = time.time()
	receive_message()