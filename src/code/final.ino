#include <NewPing.h>
#include <VBCoreG4_arduino_system.h>
#include <Servo.h>


#define camera_up PB12
#define camera_right PB13
#define panel1 PB10
#define panel2 PB11

Servo camera_right_left, camera_up_down;
Servo panel_up1, panel_up2;
NewPing sonar(PB1, PB2, 200);
bool flag_open = false;


uint8_t box_open_packet[2] = { 1, 1 }, box_close_packet[2] = { 0, 0 };


unsigned long t;
FDCAN_HandleTypeDef* hfdcan1;  // создаем переменную типа FDCAN_HandleTypeDef
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;  //FDCAN_TxHeaderTypeDef TxHeader;
void setup() {
  Serial.begin(115200);

  pinMode(PA0, OUTPUT);
  pinMode(PA1, OUTPUT);
  digitalWrite(PA0, LOW);
  digitalWrite(PA1, LOW);


  pinMode(camera_up, OUTPUT);
  pinMode(camera_right, OUTPUT);
  pinMode(panel1, OUTPUT);
  pinMode(panel2, OUTPUT);


  camera_right_left.attach(camera_right);
  camera_up_down.attach(camera_up);
  panel_up1.attach(panel1);
  panel_up2.attach(panel2);

  camera_right_left.write(90);
  camera_up_down.write(90);
  panel_up1.write(10);
  panel_up2.write(170);


  pinMode(LED2, OUTPUT);
  pinMode(PC5, OUTPUT);
  digitalWrite(PC5, HIGH);

  // Настройка FD CAN

  SystemClock_Config();           // Настройка тактирования
  canfd = new CanFD();            // Создаем управляющий класс
  canfd->init();                  // Инициализация CAN
  canfd->write_default_params();  // Записываем дефолтные параметры для FDCAN (500000 nominal / 4000000 data)
  canfd->apply_config();          // Применяем их
  hfdcan1 = canfd->get_hfdcan();  // Сохраняем конфиг
  canfd->default_start();


  TxHeader.Identifier = 0x12;
  TxHeader.DataLength = FDCAN_DLC_BYTES_2;
  TxHeader.IdType = FDCAN_EXTENDED_ID;



  delay(12000);  // время, чтобы убрать ровер в короб
}


// функция для отправки пакета

void Send_data(uint8_t data[]) {
  if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan1) != 0) {

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan1, &TxHeader, data) != HAL_OK) {
      Error_Handler();
    } else {
      digitalWrite(LED2, !digitalRead(LED2));
    }
  }
}


// функция для чтения пакета

uint8_t* Get_data(int length) {
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan1, FDCAN_RX_FIFO0) > 0) {
    FDCAN_RxHeaderTypeDef Header;           // хидер для входящего сообщения
    uint8_t* RxData = new uint8_t[length];  // длина входящего сообщения - length байт, length<=64
    if (HAL_FDCAN_GetRxMessage(hfdcan1, FDCAN_RX_FIFO0, &Header, RxData) != HAL_OK) {
      delete[] RxData;
      Error_Handler();
      return nullptr;
    } else {
      return RxData;
    }
  }
  return nullptr;
}

void MOTORS(int t) {
  if (t == 1) {
    digitalWrite(PA0, HIGH);
    digitalWrite(PA1, LOW);
  } else {
    digitalWrite(PA0, LOW);
    digitalWrite(PA1, HIGH);
  }
}


int const cam_up = 5, cam_right = 5, pannels_servo = 5;  // стандартные углы поворота для серво от полезной нагрузки

int cam_up_angel = 90, cam_right_angel = 90;  // начальные углы поворота серво полезной нагрузики
int current_pannelsservo_angel = 0;

bool f = true;  // флаг, чтобы определять, выматывать солнечную панель или остановить вымотку


void loop() {


  while (!flag_open) {  // пока сонар не заметил открывания двери, не выходим из цикла

    float dist_to_box = sonar.ping_cm();
    Serial.println(dist_to_box);
    if (dist_to_box > 30) {  // если крышку короба открыли, отправляем пакет открытия и выходим из цикла
      flag_open = !flag_open;
      delay(2000);
      Send_data(box_open_packet);
      delay(100);
      Send_data(box_open_packet);
      delay(100);
      Send_data(box_open_packet);
      delay(100);
      Serial.println("Open");

    } else {
      Send_data(box_close_packet);
      delay(100);
    }
  }

  uint8_t* data = Get_data(2);


  if (data != nullptr) {
    int type = data[0];
    int where = data[1];
    // тут будет лютая сортировка пакетов
    // type 1 - camera right-left
    // type 2 - camera up-down
    // type 3 - panels up-down
    // type 4 - motor for solar pannels
    if (where == 0) {  // отвечает за направление поворота серв и мотора
      where = -1;
    }

    if (type == 1) {

      cam_right_angel = constrain(cam_right_angel + where * cam_right, 0, 180);

      camera_right_left.write(cam_right_angel);

      delay(100);

    } else if (type == 2) {



      cam_up_angel = constrain(cam_up_angel + where * cam_up, 0, 180);

      camera_up_down.write(cam_up_angel);

      delay(100);

    } else if (type == 3) {

      current_pannelsservo_angel = constrain(current_pannelsservo_angel + where * pannels_servo, 0, 180);

      panel_up1.write(current_pannelsservo_angel);
      panel_up2.write(180 - current_pannelsservo_angel);

      delay(100);

    } else if (type == 4) {
      Serial.println(where);

      if (!f) {
        digitalWrite(PA0, LOW);
        digitalWrite(PA1, LOW);

      } else {
        if (where == 1) {
          digitalWrite(PA0, HIGH);
          digitalWrite(PA1, LOW);
        } else {
          digitalWrite(PA0, LOW);
          digitalWrite(PA1, HIGH);
        }
      }
      f = !f;
      delay(100);
    }

    delete[] data;
  } else {
    delay(100);
  }
}
