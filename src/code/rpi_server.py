""" 
Скрипт запускается непосредственно на rpi для обработки запросов с gui-интерфейса 
и дальнейшей отправки сигналов дял управления ПН на VBCore STM32 
"""

from flask import Flask, request
import can as can_fd
import struct
import requests

# Шина обмена данными по can
bus = can_fd.interface.Bus('can0', bustype='socketcan', fd=True)  
# Создаем объект запуска приложения на Flask
app = Flask(__name__)

# Основная страница. Ни для чего не используеутся 
@app.route('/', methods=["GET", "POST"])
def about():
    return 'Raspberry server for CAN. Main page'

# Полученние данных с удаленного ПК
@app.route('/can', methods=["GET", "POST"])
def can():
    data = request.json['data']
    print("Received data from outer server:", data)
    send_message(data)
    return ('', 204)

# ОТправка сообщения на VBCore STM32
def send_message(data):
    msg = can_fd.Message(arbitration_id=0x065, data=bytes(data)) 
    bus.send(msg)
    print("Sent msg to STM32:", msg)


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=6969, debug=True)