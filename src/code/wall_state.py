"""
Скрипт запускается на rpi для принятия сигнала об открытии двери модуля
и отправки запроса на сервер для начала автономного движения
"""

import requests
import can as can_fd

# Шина обмена данными с STM 32
bus = can_fd.interface.Bus('can0', bustype='socketcan', fd=True)  
# Состояние двери модуля
OPEN = False

def wall_closed() -> bool:
    global OPEN
    if OPEN:
        return 
    msg = bus.recv(timeout=0.1)  
    if msg is not None:
        try:
            data = str(msg).split()[-4]
            print(data)
            if data == "01":
                OPEN = True
                print("Making request...")
                resp = requests.post("http://192.168.1.150:6969/wall_state", json={"data": "open"})
                print(f"resp: {resp}")
        except Exception as e:
            print("Error unpacking data:", e)


if __name__ == "__main__":
	while True: 
			wall_closed()
