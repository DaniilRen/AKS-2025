# Подключаем библиотеки для работы с интерфейсом rospy
import rospy
from sensor_msgs.msg import CompressedImage

# Подключаем библиотеки для работы с изображениями из топика
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Вспомогательные библиотеки
import numpy as np
from datetime import datetime

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

        cv_image = cv2.resize(cv_image, self.image_size)
        
        # Выводим полученный кадр
        cv2.imshow('Rover image', cv_image)
        key = cv2.waitKey(5)

        # Если была нажата клавиша "с", то сохраняем текущий кадр на диск. Задаем уникальное имя для кадра
        if key == ord('c'):
            now = datetime.now()
            cv2.imwrite(f'{now.strftime("%H-%M-%S")}.jpg', cv_image)

def main():
    # Инициализируем ноду и запускаем работу класса...
    rospy.init_node('capture', anonymous=True)

    handler = Video()
    rospy.spin()

if __name__ == '__main__':
    main()