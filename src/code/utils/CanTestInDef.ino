#include <VBCoreG4_arduino_system.h>



uint8_t data[4] = { 90, 13, 14, 15 };  // 5A D E F
///uint8_t box_open_packet[3] = { 1, 1, 1 }, box_close_packet[3] = { 0, 0, 0 };


unsigned long t;
FDCAN_HandleTypeDef* hfdcan1;  // создаем переменную типа FDCAN_HandleTypeDef
CanFD* canfd;
FDCAN_TxHeaderTypeDef TxHeader;  //FDCAN_TxHeaderTypeDef TxHeader;
void setup() {
  Serial.begin(115200);


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
  TxHeader.DataLength = FDCAN_DLC_BYTES_4;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
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


void loop() {

  Send_data(data);
  uint8_t* Rx_data = Get_data(1);
  if (Rx_data != nullptr){
    Serial.println(Rx_data[0]);
  }
  delete[] Rx_data;

  delay(100);
}
