# -*- coding: utf-8 -*- 
# Подключаем библиотеки для работы с интерфейсом rospy
import rospy
from geometry_msgs.msg import Twist

# Подключаем библиотеки для получения нажатой клавиши от пользователя
import sys
from select import select
import termios
import tty


# Класс для управления ровером с клавиатуры
class BRover:
	# Конструктор, подписываемся на топик для управления скоростями движения ровера
	def __init__(self):
		self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		self.cmd_msg = Twist()
		self.binds = {
		'w':(0.1,0),
		's':(-0.1,0),
		'a':(0,0.4),
		'd':(0,-0.4)
		}
	# Метод для публикации текущего сообщения со скоростями
	def publish(self):
			self.cmd_pub.publish(self.cmd_msg)

	# Метод для получения нажатой клавиши от пользователя
	def getKey(self, settings, timeout):
		tty.setraw(sys.stdin.fileno())
		# Проверяем, что есть нажатая клавиша
		rlist, _, _ = select([sys.stdin], [], [], timeout)
		if rlist:
				# Если есть, то получаем ее название
				key = sys.stdin.read(1)
		else:
				key = ''
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
		return key

	# Сохраняем настройки ввода для получения нажатых клавиш
	def saveTerminalSettings(self):
		return termios.tcgetattr(sys.stdin)

	# Метод для возврата прошлых настроек ввода
	def restoreTerminalSettings(self, old_settings):
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def main():
	# Инициализируем ноду и класс для управления ровером...
	rospy.init_node('teleop')
	print("Teleop started")
	rover = BRover()
	rate = rospy.Rate(20)

	# Задаем настройки для ввода
	settings = rover.saveTerminalSettings()

	try:
		while not rospy.is_shutdown():
				# Получаем нажатую клавишу
				key = rover.getKey(settings, 10)

				# Если клавиша есть в словаре, то изменяем скорость в соответствии с ним
				if key in rover.binds.keys():
						rover.cmd_msg.linear.x += rover.binds[key][0]
						rover.cmd_msg.angular.z += rover.binds[key][1]

				# Если нажата клавиша "z", то останавливаем робота
				if key == 'z':
						rover.cmd_msg = Twist()
				# Если нажата клавиша "q", то выходим из программы
				elif key == 'q':
						break

				# Публикуем сообщение со скоростями
				rover.publish()
				rate.sleep()

	except Exception as e:
			print(e)

	finally:
		# В конце программы всегда отправляем сообщение с нулевыми скоростями, чтобы остановить робота
		rover.cmd_msg = Twist()
		rover.publish()
		# Востанавливаем найстроки ввода
		rover.restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()