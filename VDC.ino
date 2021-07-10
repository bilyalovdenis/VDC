/**
 * Проект "Ветеринарный диагностический комплекс".
 * Автор кода Билялов Денис.
 */
 /**
 * Информация о проекте:
 * Комплекс призван решить проблему ветеринарных клиник, которая заключается в отсутвии возможности оперативно получать информаию, касающуюся показателей жизнедеятельностим животного.
 * Вследствие чего персонал клиники не успевает препринять действия, направленные стабилизирование состояние животного, что приводит к неблагоприятным последствиям.
 * 
 * Программа является продуктом опытной образовательной деятельности, поэтому многие ее части "неуместны"!!! 
 */
 /**
  * Комплектующие:
  *   - контроллер Arduino UNO
  *   - питание Battery Shield Li-po 
  *   - Wi-Fi Troyka-модуль
  *   - GPS модуль VK16E 
  *   - датчик температуры DS18B20
  *   - датчик пульса PulseSensor
  *   - датчик мышечной активности EMG
  *   - датчик частоты дыхания Резистивный датчик давления + НХ711
  *   -
  */
  /*
   * Проект находится в стадии разработки.
   * Задачи:
   * 1.Оформить класс для работы с датчиком дыхания:
   *    - создать алгорим измерения частоты дыхания с помощью изменения показателей давления в области груди животного;
   *    - произвести калибровку и выяснить требуемые константы.
   * 2.Оформить класс для работы с датчиком мышечной активности:
   *    - создать шкалу активности животного;
   *    - Продумать время измерения активности;
   *    - произвести калибровку и выяснить требуемые константы
   * 3.Изучить работу системы и модуля GPS, найти или создать алгоритм обработки данных с разных спутнкиов для уточнения координат.
   * 4.Подумать над добавлением оповещений об ошибках. 
   * 5.Заменить String в функциях на статические массивы char для экономии динамиеской памяти
   * 
   */
   
//=== Макрос таймера ===

#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flag = millis() - tmr >= (x);\
  if (flag) tmr += (x);\
  if (flag)

//=== Библиотеки ===//:

#include <SoftwareSerial.h>                                // Библиотка для работы имитирующая UART на цифровых пинах(wi=fi, GPS)
#include <TroykaGPS.h>                                     // библиотека для работы с GPS.
 // Датчики:
#include "OneWire.h"                                       // библиотека для работы с протоколом 1-Wire
#include "DallasTemperature.h"                             // библиотека для работы с датчиком DS18B20 
#include "iarduino_SensorPulse.h"                          // датчик пульса(ИЗМЕНИЛ БИБИЛИОТЕКУ, пожробнее строка с классом)
#include "HX711.h"                                         // библиотека ля работы с датчиком дыхания

//=== Пины ===//:

//Цифровые
#define TEMP_PIN 5                                         // 1.цифровой пин датчика температуры
#define DT_PIN  7                                          // 3.указываем номер вывода, к которому подключен вывод DT (Датчик дыхания)
#define SCK_PIN 6                                          // 4.указываем номер вывода, к которому подключен вывод SCK (Датчик дыхания)
#define WIFI_TX 8                                          // 5.пид для выхода wifi модуля TX
#define WIFI_RX 9                                          // 6.пид для выхода wifi модуля RX
#define GPS_TX 10                                          // 7.TX GPS
#define GPS_RX 11                                          // 8.RX GPS
                                                           
//Аналоговые
#define PULSE_PIN A0                                       // аналоговый пин пульсометра

// === Объекты(часть 1)/ GPS переменные ===

SoftwareSerial wifiSerial(WIFI_TX, WIFI_RX);               // объект для работы с Wi-Fi 
SoftwareSerial GPSSerial(GPS_TX, GPS_RX);                  // объект для работы с GPS
#define GPS_SERIAL    GPSSerial    
GPS gps(GPS_SERIAL);                                       

// === Функции ===//
//WiFi
void initializationWiFi (){                                    // Инициализация Wifi-модуля
  Serial.println("подклбчаем модуль");
 wifiSerial.begin(9600);
 String Status;// открываем порт соединения с модулем
  wifiSerial.println("AT+SLEEP=0");                            // Выключаем режим сна
  delay(100);
  while (wifiSerial.available()>1)Serial.write(wifiSerial.read()); //считывание/освобождение порта
  wifiSerial.println("AT+CWMODE_DEF=1");//_DEF                 // Записываем режим работы "приемник" во флш память 
  delay(100);
  while (wifiSerial.available()>1)Serial.write(wifiSerial.read()); // считывание/освобождение порта
  wifiSerial.println("AT+CIPSTATUS");                         // проверка подключен ли модуль к wifi?
  delay(100);
   // Проверка подключения модуля к сети
    while (wifiSerial.available()>1)
    {
      
      //Status.concat(wifiSerial.read());
      char c = wifiSerial.read();
      Status.concat(c);
    }
   // Проверка подключения модуля к сети
 
    if (Status.indexOf('2')>0|| Status.indexOf('3')>0)          // Если подключен
    {
      Serial.println ("WIFI модель подключен к сети");
      
    }
    else if (Status.indexOf('4')>0|| Status.indexOf('5')>0)     // Если не подключен спрашиваем ssid и пароль сети
    {
       ////////Ввод SSID//////
     Serial.print ("Введите название сети: ");
     while (Serial.available()==0){}                              // Ждем, пока пользователь введет SSID
     String ssid = Serial.readString();
     ssid.replace("\r\n","");                                     // Избавление от символов переноса строки и картеки
     Serial.println(ssid);
      ///////Ввод пароля//////
     Serial.print ("Введите пароль: ");
     while (Serial.available()==0){}                              // Ждем, пока пользователь введет пароль
     String pass = Serial.readString();
     pass.replace("\r\n",""); 
     Serial.println(pass);
     wifiSerial.print("AT+CWJAP=\"" + ssid+ "\",\"" + pass + "\"\r\n"); //Вывод
    }
    else Serial.println ("Хуета не работает");
    while (wifiSerial.available())wifiSerial.read();  
}
   
void sendToWIFI(char sensorName, char number, float value) {             // Принимает название датчика, его порядковый номер и значение. После чего отправляет их в базу данных.
     String request = "POST /uno?value=" +String(value)+"&type=" +String(sensorName)+String(number)+"&timestamp=2021-04-08T18:44:07 HTTP/1.1\r\nHost: send.vetdk.ru\r\n\r\n\r\n\r\n";
   /*   wifiSerial.println("AT+CIPSTART=\"TCP\",\"send.vetdk.ru\",80");
      delay(500);
      if (wifiSerial.available())
        {
        while (wifiSerial.available()>1)Serial.write(wifiSerial.read());
        }
      wifiSerial.println("AT+CIPSEND="+String(request.length()+ (int)11));
      delay(500);
      if (wifiSerial.available())
        {
        while (wifiSerial.available()>1)Serial.write(wifiSerial.read());
        }
      wifiSerial.println(request);
      delay(5000);  
       if (wifiSerial.available())
        {
        while (wifiSerial.available()>1)Serial.write(wifiSerial.read());
        }
      wifiSerial.println("AT+CIPCLOSE");*/
         Serial.print(request);  
}

//=== Классы ===// 

class TemperatureVDC {                                     // Класс для работы с датчиком температуры

  public:
   TemperatureVDC(uint8_t pin) {                          // конструктор
      _oneWire = OneWire(pin);
      _sensor = DallasTemperature(&_oneWire);
      
    }

  
   void initialize() {                                    // функция инициализации датчика
      _sensor.begin();                                     // начинаем работу с датчиком
      _sensor.setResolution(12);                           // устанавливаем разрешение датчика от 9 до 12 бит
    }

   float getTemperature () {                              // Получение значения температуры(Защита от ошибков вычислинения реализовано установкой интервала возможных значений температуры от +10 до +60)
      float temperature = 0;
      float sumTemp = 0;
      uint8_t count = 0;
      float temperatureCount;
      for (uint8_t i = 0; i < 20; i++) {
        _sensor.requestTemperatures();
        temperatureCount=_sensor.getTempCByIndex(0);
        if(temperatureCount<60.00&temperatureCount>10.00)
        {
        sumTemp = sumTemp + temperatureCount;
        count++;
        }
        delayMicroseconds(5);
      }
      temperature = sumTemp / count;
      if (count < 15){
      Serial.print("Некорректная температура. Попробуйте изменить положение датчика(TemperatureVDC.getTemperature ())");
      return 0;
      }
      else return temperature;
    }
   void sendToWifi (float value){
    sendToWIFI('T','1',value);
   }
   
  private:

    OneWire _oneWire;                                      // инициализация объекта класса OneWire
    DallasTemperature _sensor;                             // инициализация объекта класса DallasTemperature
   
};
class PulseVDC {                                           // Класс датчика пульса (ИЗМЕНИЛ БИБИЛИОТЕКУ "iarduino_SensorPulse.h", МОЖЕТ НЕ РАБОТАТЬ в class iarduino_SensorPulse{) добавлена строка "iarduino_SensorPulse(){}", нужна калибровка()
  public:
    PulseVDC (uint8_t pin) {
      _pulse =  iarduino_SensorPulse (pin);
    }
    void  initialize() {                                   // Функция инициализации датчика
      _pulse.begin();
    }
    uint8_t getPulse() {                                   // Функция для получения значений с датчика

      int pulse;
      if (_pulse.check(ISP_VALID) == ISP_CONNECTED) {      // если датчик подключен к пальцу

        pulse = _pulse.check(ISP_PULSE);                   // печать значения пульса
        return pulse;
      }
      
      else{ 
      Serial.print("Подключите датчик пульса. Попробуйте изменить положение датчика(PulseVDC.getPulse())");
      return 0;
    }}
 void sendToWifi (int value){
    sendToWIFI('P','0',value);
   }
  private:
    iarduino_SensorPulse _pulse;                           // инициализация объекта pulse
   
};
class BreathVDC {                                          // Класс датчика дыхания(!!!РАЗОБРАТЬСЯ С КАЛИБРОВКОЙ, а после нее добавить функцию снятия показателей)
  
  public:
    BreathVDC (uint8_t dt, uint8_t sck) {                  // конструктор
      DT = dt;
      SCK = sck;
    }
    void  initialize() {                                   // Функция инициализации датчика
      _scale.begin(DT, SCK);                               // инициируем работу с датчиком
      _scale.set_scale();                                  // выполняем измерение значения без калибровочного коэффициента
      _scale.tare();                                       // сбрасываем значения веса на датчике в 0
      //_scale.set_scale(calibration_factor);        // !!!устанавливаем калибровочный коэффициент
    }
  void sendToWifi (int value){
    sendToWIFI('D','0',value);
   }
  private:
    float weight_of_standard = 167.8;                      // указываем эталонный вес
    float conversion_rate = 0.035274;                      // указываем коэффициент для перевода из унций в граммы
    const int z = 10;                                      // указываем количество измерений, по которым будет найдено среднее значение
    //  float calibration_value[z];           // !!!создаём массив для хранения считанных значений
    float calibration_factor = 0;                          // переменная фактора температуры, которая влияет на показания датчика (ее мы и получаем после калибровки)
    float units;                                           // задаём переменную для измерений в граммах
    float ounces;                                          // задаём переменную для измерений в унциях
    uint8_t DT;                                            // переменныя для хранения пина
    uint8_t SCK;                                           // переменныя для хранения пина
    HX711 _scale;                                          // внутренний объект
   
};

//=== Объекты ===//:
TemperatureVDC tempSensor(TEMP_PIN);                       // датчик температуры
PulseVDC pulseSensor(PULSE_PIN);                           // датчик пульса
BreathVDC breathSensor (DT_PIN, SCK_PIN);                  // датчик частоты дыхания



//=== Переменные ===//

// задаём размер массива для времени, даты, широты и долготы
#define MAX_SIZE_MASS 16
// массив для хранения текущего времени
char strTime[MAX_SIZE_MASS];
// массив для хранения текущей даты
char strDate[MAX_SIZE_MASS];
// массив для хранения широты в градусах, минутах и секундах
char latitudeBase60[MAX_SIZE_MASS];
// массив для хранения долготы в градусах, минутах и секундах
char longitudeBase60[MAX_SIZE_MASS];

void setup() {

  Serial.begin(9600);                                     // устанавливаем последовательное соединение
  Serial.println("hello");
  //Инициализируем устройства
  // tempSensor.initialize();                                // датчсик температуры
  // pulseSensor.initialize();                               // датчик пульса
         
//initializationWiFi ();
//tempSensor.sendToWifi(13);
GPS_SERIAL.begin(115200);
  // печатаем строку
  Serial.println("GPS init is OK on speed 115200");
  // изменяем скорость обещение GPS-модуля с управляющей платой на 9600 бод
  // используем NMEA-команду «$PMTK251,9600*17\r\n»
  GPS_SERIAL.write("$PMTK251,9600*17\r\n");
  // закрываем Serial-соединение с GPS-модулем
  GPS_SERIAL.end();
  // открываем Serial-соединение с GPS-модулем на скорости 9600 бод
  GPS_SERIAL.begin(9600);
  // печатаем строку
  Serial.print("GPS init is OK on speed 9600");
 
}


void loop() {
   if (gps.available()) {
    // считываем данные и парсим
    gps.readParsing();
    // проверяем состояние GPS-модуля
    switch(gps.getState()) {
      // всё OK
      case GPS_OK:
        Serial.println("GPS is OK");
        // выводим координаты широты и долготы
        // 1. в градусах, минутах и секундах
        // 2. градусах в виде десятичной дроби
        Serial.println("GPS Coordinates: ");
        gps.getLatitudeBase60(latitudeBase60, MAX_SIZE_MASS);
        gps.getLongitudeBase60(longitudeBase60, MAX_SIZE_MASS);
        Serial.print("Latitude\t");
        Serial.print(latitudeBase60);
        Serial.print("\t\t");
        Serial.println(gps.getLatitudeBase10(), 6);
        Serial.print("Longitude\t");
        Serial.print(longitudeBase60);
        Serial.print("\t\t");
        Serial.println(gps.getLongitudeBase10(), 6);
        // выводим количество видимых спутников
        Serial.print("Sat: ");
        Serial.println(gps.getSat());
        // выводим текущую скорость
        Serial.print("Speed: ");
        Serial.println(gps.getSpeedKm());
        // выводим высоту над уровнем моря
        Serial.print("Altitude: ");
        Serial.println(gps.getAltitude());
        // выводим текущее время
        Serial.print("Time: ");
        gps.getTime(strTime, MAX_SIZE_MASS);
        gps.getDate(strDate, MAX_SIZE_MASS);
        Serial.write(strTime);
        Serial.println();
        // выводим текущую дату
        Serial.print("Date: ");
        Serial.write(strDate);
        Serial.println("\r\n");
        // каждую переменную дату и времени можно выводить отдельно
  /*    Serial.print(gps.getHour());
        Serial.print(gps.getMinute());
        Serial.print(gps.getSecond());
        Serial.print(gps.getDay());
        Serial.print(gps.getMonth());
        Serial.print(gps.getYear());
  */   
        break;
      // ошибка данных
      case GPS_ERROR_DATA:
        Serial.println("GPS error data");
        break;
      // нет соединение со спутниками
      case GPS_ERROR_SAT:
        Serial.println("GPS no connect to satellites!!!");
        break;
    }
  }
}
