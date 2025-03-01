""" 
Локальный сервер для графического интерфейса. 
Обрабатывает одометрию ровера, изменения скорости и реализует управление движением.
Для управления полезной нагрузкой посылает запросы на сервер на rpi
"""

from flask import Flask, render_template, Response, request, jsonify
import cv2
import rospy
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import math
import requests


# Класс содержит набор методов для управления ровера в режиме телеметрии
class Brover:
    # Подписка на топик для дальнешей посылки на него сообщений с параметрами движения,
    # создание пустого сообщения 
    def __init__(self) -> None:
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber('/odom_pose2d', Pose2D, self.odom_cb)
        self.cmd_msg = Twist()
        self.odom_msg = Pose2D()
        self.ang_vel = 0.4
        self.linear_vel = 0.1
        self.direction = None
        self.directions = {
            'w': 'forward', # вперед
            's': 'backward', # назад
            'a': 'left', # поворот влево
            'd': 'right', # поворот вправо
            'z': 'stop', # остановка
            'reset': 'stop' # остановка
        }
        self.binds = {
            'w':((self.linear_vel, 0), 'drive Forward'), 
            's':((-self.linear_vel, 0), 'drive Backward'),
            'a':((0, self.ang_vel),'turn Left' ), 
            'd':((0, -self.ang_vel), 'turn Right'), 
            'o':((2, 1), 'camera Up'), 
            'p':((2, 0), 'camera Down'), 
            'k':((1, 0), 'camera Left'),
            'l':((1, 1), 'camera Right'), 
            'n':((3, 1), 'payload Up'),
            'm':((3, 0), 'payload Down'),
            'v':((4, 1), 'payload Roll'), 
            'b':((4, 0), 'payload Roll Back'), 
        }

    # Публикация сообщения со скоростями в топик
    def publish(self) -> None:
        self.cmd_pub.publish(self.cmd_msg)

    def odom_cb(self, msg) -> None:
        self.odom_msg = msg

    def get_odom(self) -> dict:
        msg = self.odom_msg
        return {'x': round(msg.x, 2), 'y': round(msg.y, 2), 'theta': round(msg.theta*180/math.pi, 2), 'direction': self.direction}

    def update_direction(self, key: str) -> None:
        self.direction = self.directions[key]

    def set_velocity(self, linear_vel=0, ang_vel=0) -> None:
        if linear_vel != 0:
                        self.binds['w'] = ((linear_vel, 0), 'drive Forward')
                        self.binds['s'] = ((-linear_vel, 0), 'drive Backward')
        if ang_vel != 0:
                        self.binds['a'] = ((0, ang_vel),'turn Left' ),
                        self.binds['d'] = ((0, -self.ang_vel), 'turn Right')

    def can_send(self, data) -> None:
        resp = requests.post(url='http://192.168.1.34:6969/can', json={'data': data})
			


# Класс для получения и отображения кадров из топика, также позволяет сохранять текущий кадр на диск
class Video:
    # Конструктор, создаем подписчика и задаем желаемый размер кадров
    def __init__(self,
                 image_size=(640, 480),
                 image_topic="/front_camera/image_raw/compressed"
    ):
        self.bridge = CvBridge()
        self.image_size = image_size

        self.image_sub = rospy.Subscriber(
                image_topic, 
                CompressedImage, 
                self.callback)

    # Callback-функция для получения кадров из топика
    def callback(self, msg):
        # Декодируем содержимое из сообщения и изменяем размер кадра
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        except CvBridgeError as e:
            print(e)
        self.cv_image = cv2.resize(cv_image, self.image_size)

    def get_image(self) -> np.ndarray:
        return self.cv_image
                        

app = Flask(__name__)

# Функция преобразует картинку с камеры ровера и передает в интерфейс 
def gen_frames():
    global rover, video_handler
    try:
        while not rospy.is_shutdown():
            image = video_handler.get_image()
            _, buffer = cv2.imencode('.jpg', image)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')  # concat frame one by one and show result
            # Публикуем сообщение со скоростями
            rover.publish()
            rate.sleep()
    except Exception as e:
        print(e)
    finally:
        # При выходе из всегда программы отправляем сообщение с нулевыми скоростями, чтобы остановить робота
        rover.cmd_msg = Twist()
        rover.publish()

# Видео для отображения на интерфейсе 
@app.route('/video_feed', methods=["POST", "GET"])
def video_feed():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Одометрия ровера для отображения на интерфейсе
@app.route('/sys_info', methods=["GET"])
def sys_info():
    global rover
    return jsonify(rover.get_odom())

# Изменение скорости движения ровера ползунками
@app.route('/update_velocity', methods=["POST"])
def update_velocity():
	global rover
	type = request.form.get('type')
	val = float(request.form.get('val'))
	if type == None or val == None:
			return ('', 400)
	if type == 'linear':
			print('Updated linear speed to:', val)
			rover.set_velocity(linear_vel=val)
	if type == 'angular':
			print('Updated angular speed to:', val)
			rover.set_velocity(ang_vel=val)
	return ('', 204)

# Управление движением ровера и ПН с клавиатуры
@app.route('/update_key', methods=["POST"])
def update_key():
    global rover
    key = request.form.get('key').lower().strip()
    if key == None:
            print("Error key")
            return ('', 400)
    # обработка движения робота
    elif key in ['z', 'reset']:
        print("Stopping")
        rover.update_direction(key)
        rover.cmd_msg = Twist()
    elif key in 'wasd':
        print(rover.binds[key][1], rover.binds[key][0])
        rover.update_direction(key)
        rover.cmd_msg.linear.x = rover.binds[key][0][0]
        rover.cmd_msg.angular.z = rover.binds[key][0][1]
    # обработка управления ПН
    elif key in 'opklnmbv':
        print(rover.binds[key][1], rover.binds[key][0])
        data = rover.binds[key][0]
        rover.can_send(data=data)

    return ('', 204)


@app.route('/', methods=["POST", "GET"])
def index():
    global rover
    return render_template('index.html')	


if __name__ == '__main__':
    rospy.init_node('interface')
    rate = rospy.Rate(30)
    rover = Brover()
    video_handler = Video()
    app.run()