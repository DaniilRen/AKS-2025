# Подключаем библиотеки для работы с интерфейсом rospy
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_srvs.srv import Empty

# Подключаем вспомогательные библиотеки
from math import *
import time
from flask import Flask, request

# Класс для задания фрагментов маршрута 
class Task:
	MIN_LINEAR_VEL = 0.1
	MIN_ANGULAR_VEL = 0.1

	MIN_LINEAR_ERROR = 0.2
	MIN_ANGULAR_ERROR = 0.15

	def __init__(self,
                    distance: float = 0.0,
                    theta: float = 0.0,
                    linear_vel: float = 0.0,
                    angular_vel: float = 0.0,
                    linear_err: float = 0.2,
                    angular_err: float = 0.15
				):

			# Одометрия задается относительно последней позиции!
			self.distance = distance
			self.theta = theta

			# Скорости движения
			self.linear_vel = max(abs(linear_vel), self.MIN_LINEAR_VEL)
			self.angular_vel = max(abs(angular_vel), self.MIN_ANGULAR_VEL)

			# Погрешности позиционирования
			self.linear_err = max(abs(linear_err), self.MIN_LINEAR_ERROR)
			self.angular_err = max(abs(angular_err), self.MIN_ANGULAR_ERROR)

# Класс, которые обрабатывает маршрут и рассчитывает скорости движения ровера
class Rover:
	# Конструктор, подписываемся на топики одометрии, управления, инициализируем сервис для сброса одометрии
	def __init__(self):
			self.tasks = []

			self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
			self.odom_sub = rospy.Subscriber('/odom_pose2d', Pose2D, self.odom_cb)

			rospy.wait_for_service('/reset')
			self.reset_srv = rospy.ServiceProxy('/reset', Empty)

			self.cmd_msg = Twist()
			self.odom_msg = Pose2D()

			# Сбрасываем одометрию
			self.reset()

	# Метод для публикации текущего сообщения со скоростями
	def publish(self):
			self.cmd_pub.publish(self.cmd_msg)

	# Callback-функция для получения текущей одометрии ровера
	def odom_cb(self, msg):
			self.odom_msg = msg

	# Метод для обработки участка маршрута
	def process_tasks(self):
			if len(self.tasks) == 0:
					return

			# Получаем параметры этого участка
			task = self.tasks[0]
			distance = sqrt(pow(self.odom_msg.x, 2) + pow(self.odom_msg.y, 2))
			self.cmd_msg = Twist()

			# Если необходимо двигаемся вперед или назад для достижения точки
			linear_err = abs(distance - task.distance)
			if linear_err > task.linear_err:
					self.cmd_msg.linear.x = copysign(task.linear_vel, task.distance - distance)

			# Если необходимо, то поворачиваем ровер для достижения заданного угла поворота (рысканья)
			theta_error = self.normalize_theta(task.theta - self.odom_msg.theta)
			if abs(theta_error) > task.angular_err:
					self.cmd_msg.angular.z = copysign(task.angular_vel, theta_error)

			self.publish()
	
			# Если робот приехал на точку (завершил участок маршрута), то переходим к следующему участку
			if self.cmd_msg == Twist():
					print(f'Point with distance={task.distance} and theta={task.theta} was reached!')
					self.tasks.pop(0)

					self.reset()

	# Метод для добавления нового участка маршрута
	def insert_task(self, task: Task):
			print(f'Point with distance={task.distance} and theta={task.theta} was added to queue!')
			self.tasks.append(task)

	# Вспогательный метод для нормализации угла в интервале от -pi до pi
	def normalize_theta(self, x):
			result = fmod(x + pi, 2.0 * pi)
			return (result + pi if result <= 0 else result - pi)

	# Метод для сброса одометрии
	def reset(self):
			print(f'Reset odometry!')
			self.reset_srv()
			time.sleep(3)

	# Проверяем, движется ли ровер по маршруту или весь маршрут уже был пройдем
	def has_task(self):
			return (len(self.tasks) > 0)

def main():
	global rover
	# Добавляем участки маршрута для прохождения
	rover.insert_task(Task(distance=1.5, linear_vel=0.2)) # Движение вперед на 1.5м со скорость. 0.2м/с
	# rover.insert_task(Task(theta=pi/2, angular_vel=0.8)) # Поворот на 180 градусов с угловой скоростью 0.8 рад/с

	try:
			# Выполняем маршрут
			while not rospy.is_shutdown():
					rover.process_tasks()

					if not rover.has_task():
							break

					rate.sleep()

	except Exception as e:
			print(e)

	finally:
			# В конце всегда останавливаем ровер
			rover.cmd_msg = Twist()
			rover.publish()


app = Flask(__name__)
WALL_STATE = "CLOSED"

# Инициализируем ноду и запускаем класс для автономного прохождения маршрута
rospy.init_node('automate')

rover = Rover()
rate = rospy.Rate(20)


@app.route('/', methods=["POST", "GET"])
def index():
    return ('Rover automate movement. Main page', 204)


@app.route('/wall_state', methods=["POST", "GET"])
def wall_state():
    print()
    if request.method == "POST":
        global WALL_STATE
        if request.json['data'] == "open" and WALL_STATE != "OPEN":
            WALL_STATE = "OPEN"
            main()
            return (f'Wall state: {WALL_STATE}', 204)
        else:
            return ('', 400)
    return (f'Wall state: {WALL_STATE}', 203)


if __name__ == "__main__":
    app.run(host='0.0.0.0', port=6969, debug=False)
