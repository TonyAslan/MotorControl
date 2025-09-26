#define PWM 17  //pwm
#define DIR2 16 //方向
#define DIR1 15
#define ENA 5   //编码器
#define ENB 4
//蓝牙相关
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" //读操作特征值
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E" //写操作特征值
#define CHARACTERISTIC_UUID_NAME "6E400004-B5A3-F393-E0A9-E50E24DCCA9E" // 新增加 设备名称修改特征

void initBLE();
void restartBLE();
void processNameChange();

// EEPROM相关配置
#define EEPROM_SIZE 256          // 分配的EEPROM空间大小(字节)
#define NAME_STORAGE_ADDR 0      // 名称存储的起始地址
#define NAME_VALID_FLAG_ADDR 250 // 名称有效性标志地址(使用地址250-253)
#define VALID_FLAG 0x55AAAA55    // 自定义有效性标志

//电机类
#include "DCMotor.h"
//蓝牙相关
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h> // 包含EEPROM库
#include <ArduinoQueue.h>

ArduinoQueue<String> dataQueue(10);

DCMotor motor;
//定时器
hw_timer_t* timer0 = NULL;
long en_cnt = 0; //编码器计数
long en_lastcnt = 0; //编码器上次计数

double currentSpeed = 0; //当前速度
double targetSpeed = 0;  //目标速度
double speedError = 0;   //速度误差
double speedErrorIntegral;  //积分
double speedErrorDifferential; //微分
double lastSpeedError;   //上次速度误差
double kp = 2, ki = 1, kd = 0; //pid参数
int pwmOutput = 0; //pwm输出
//蓝牙相关定义
//服务
BLEServer *pServer = NULL;
//写操作特征指针
BLECharacteristic * pTxCharacteristic;
//读操作特征
BLECharacteristic * pRxCharacteristic;
//修改名称的特征指针
BLECharacteristic *pNameCharacteristic;
 // 初始设备名称
std::string deviceName = "Default_Name";
// 名称接收缓冲区
std::string nameBuffer; 
unsigned long lastNameWriteTime = 0;
bool nameChangePending = false;
//本次连接状态
bool deviceConnected = false;
//上次连接状态
bool oldDeviceConnected = false;
//成功发送8字节
uint8_t txValueSuccess[8] = {0x01, 0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//失败
uint8_t txValueFail[8] = {0x00, 0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//芯片Id
String chipId;

// 在setup()中初始化EEPROM
void initEEPROM() {
  EEPROM.begin(EEPROM_SIZE); // 初始化EEPROM
  //Serial.printf("EEPROM initialized with size %d bytes\n", EEPROM_SIZE);
}

// 保存名称到EEPROM
void saveNameToEEPROM(const std::string& name) {
  // 1. 存储名称长度(首个字节)
  uint8_t nameLength = name.length();
  EEPROM.write(NAME_STORAGE_ADDR, nameLength);
  // 2. 存储名称内容
  for (int i = 0; i < nameLength; i++) {
    EEPROM.write(NAME_STORAGE_ADDR + 1 + i, name[i]);
  }
  // 3. 写入有效性标志
  uint32_t flag = VALID_FLAG;
  EEPROM.put(NAME_VALID_FLAG_ADDR, flag);
  // 4. 提交更改
  if (EEPROM.commit()) {
    //Serial.println("Name saved to EEPROM successfully");
  } else {
    //Serial.println("Failed to save name to EEPROM");
  }
}

// 从EEPROM加载名称
std::string loadNameFromEEPROM() {
  // 1. 检查有效性标志
  uint32_t storedFlag = 0;
  EEPROM.get(NAME_VALID_FLAG_ADDR, storedFlag);
  if (storedFlag != VALID_FLAG) {
    //Serial.println("No valid name stored in EEPROM");
    return deviceName; // 返回默认名称
  }
  // 2. 读取名称长度
  uint8_t nameLength = EEPROM.read(NAME_STORAGE_ADDR);  
  // 3. 验证长度合理性
  if (nameLength == 0 || nameLength > 247) {
    //Serial.println("Invalid name length in EEPROM");
    return deviceName;
  } 
  // 4. 读取名称内容
  std::string loadedName = "";
  for (int i = 0; i < nameLength; i++) {
    char c = EEPROM.read(NAME_STORAGE_ADDR + 1 + i);
    loadedName += c;
  } 
  //Serial.printf("Loaded name from EEPROM: %s\n", loadedName.c_str());
  return loadedName;
}

//服务回调函数  确认连接状态
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
//返回的Str
String resStr;
//特征值回调函数
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      //接收信息
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        //Serial.println("*********");
        //Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++){
          //Serial.print(rxValue[i]);
          resStr += rxValue[i];
        }
        //串口打印接收到的
        //Serial.print(resStr);
        //Serial.println();
        //0x03 查询下位机ID
        if(resStr[0] == 3){
          pTxCharacteristic->setValue(chipId.c_str());
          pTxCharacteristic->notify();
          delay(10);
        //0x0 停止电机
        }else if(resStr[0] == 0){
          //确认指令
          pTxCharacteristic->setValue(txValueSuccess, 8);
          pTxCharacteristic->notify();
          delay(10);
        //0x01 启动电机
        }else if(resStr[0] == 1){
          //确认指令
          pTxCharacteristic->setValue(txValueSuccess, 8);
          pTxCharacteristic->notify();
          delay(10);
        //0x02 设置电机 转速等
        }else if(resStr[0] == 2){
          //确认指令
          pTxCharacteristic->setValue(txValueSuccess, 8);
          pTxCharacteristic->notify();
          dataQueue.enqueue(resStr);
          delay(10);
        }else if(resStr[0] == 4)
        {
           //
          pTxCharacteristic->setValue(txValueSuccess, 8);
          pTxCharacteristic->notify();
          delay(10);
        }else
        {
          //txValueSuccess[0] = resStr[0];
          //失败指令
          pTxCharacteristic->setValue(txValueFail, 8);
          pTxCharacteristic->notify();
          delay(10);
        }
        resStr = "";
      }
    }
};


class NameChangeCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (!value.empty()) {
      // 1. 打印原始接收数据（十六进制格式）
      //Serial.print("Received raw data (hex): ");
      for (int i = 0; i < value.length(); i++) {
        //Serial.printf("%04X ", (uint16_t)value[i]);
      }
      //Serial.println();
       // 添加到缓冲区
        nameBuffer.append(value);
        lastNameWriteTime = millis();
        
        // 检查是否收到结束符 (0x00)
        if (std::find(value.begin(), value.end(), '\0') != value.end()) {
          nameChangePending = true;
        }
    }
    }
};

void initBLE()
{
  //Serial.println("Initializing BLE...");
  //创建一个BLE设备
  BLEDevice::init(deviceName);
  //创建gatt服务器
  pServer = BLEDevice::createServer();
  //创建服务器的回调函数  确认连接状态
  pServer->setCallbacks(new MyServerCallbacks());
  //创建服务
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  //创建特征对象(Tx)通知
  pTxCharacteristic = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pTxCharacteristic->addDescriptor(new BLE2902());

  //创建特征对象（Rx）写
  pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
                                          );
  //设置读特征值回调函数  读到之后在该回调函数里回复确认指令
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // 创建名称修改特征  写
  pNameCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_NAME,
      BLECharacteristic::PROPERTY_WRITE
  );
  pNameCharacteristic->setCallbacks(new NameChangeCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  //Serial.printf("BLE started with name: %s\n", deviceName.c_str());
}

void restartBLE()
{
  //Serial.println("Restarting BLE...");
  BLEDevice::deinit();
  delay(500); // 确保完全释放资源
  initBLE();
}
// 处理名称变化
void processNameChange()
{
  if (nameBuffer.empty()) return;
  // 查找结束符位置
  size_t nullPos = nameBuffer.find('\0');
  if (nullPos != std::string::npos) {
    deviceName = nameBuffer.substr(0, nullPos);
  } else {
    deviceName = nameBuffer;
  }
  // 修剪尾部空白
  size_t end = deviceName.find_last_not_of(" \r\n\t");
  if (end != std::string::npos) {
    deviceName = deviceName.substr(0, end + 1);
  }
  // 长度限制
  if (deviceName.length() > 247) {
    deviceName = deviceName.substr(0, 247);
  }
  //Serial.printf("Setting new name: %s (length: %d)\n", deviceName.c_str(), deviceName.length());
  // 保存新名称到EEPROM
  saveNameToEEPROM(deviceName);
  // 清空缓冲区
  nameBuffer.clear();
  nameChangePending = false;
  // 重启BLE
}

//编码器A相边沿触发
void IRAM_ATTR ENA_IRQ(){
    if(digitalRead(ENA)^digitalRead(ENB))//亦或
    {
      en_cnt--;
    }else {
      en_cnt++;
    } 
}

//编码器B相边沿触发
void IRAM_ATTR ENB_IRQ()
{
   if(digitalRead(ENA)^digitalRead(ENB))//亦或
   {
    en_cnt++;
   }else {
    en_cnt--;
   } 
}

void IRAM_ATTR timer0_IRQ()
{
  //3120 60减速比电机每转的脉冲数 en_cnt就是脉冲数
  double angle = en_cnt * 360.0/3120;
  //当前速度 定时器是50ms触发一次 计算每分钟多少转
  currentSpeed = (en_cnt - en_lastcnt)* 1000.0 * 60 / 3120 / 50;
 
  //计算误差 目标速度减去当前速度
  speedError = targetSpeed - currentSpeed;
  //积分项 误差*时间 求和
  speedErrorIntegral += (speedError * 0.05); //时间间隔 0.05s
  //微分项 误差的变化量/时间
  speedErrorDifferential = (speedError - lastSpeedError) / 0.05;
  //计算 pid 输出
  double pidOutput = (kp*speedError) + (ki*speedErrorIntegral) + (kd*speedErrorDifferential);
  // Serial.print("pidOutput = ");
  // Serial.println(pidOutput);
  //输出限幅
  pwmOutput = pidOutput;
  if(pidOutput > 255) pwmOutput = 255;
  if(pidOutput < -255) pwmOutput = -255;
  //输出到motor
  if(pwmOutput > 0)
  {
    motor.setSpeed(1,pwmOutput);
  }else if(pwmOutput < 0)
  {
    pwmOutput = -pwmOutput;
    motor.setSpeed(0,pwmOutput);
  }

  //
  en_lastcnt = en_cnt;
  lastSpeedError = speedError;
}
//编码器不单独成类

void setup() {
  // put your setup code here, to run once:
  //设置串口
  Serial.begin(115200);
  //设置编码器
  pinMode(ENA,INPUT);
  pinMode(ENB,INPUT);
  //连接电平变化的中断函数
  attachInterrupt(ENA, ENA_IRQ, CHANGE);
  attachInterrupt(ENB, ENB_IRQ, CHANGE);
  //设置电机
  motor.attach(DIR2, DIR1, PWM);
  //设置定时器 有四个定时器，分频系数80，即将80MHz变成1MHz,true递增计数
  timer0 = timerBegin(0, 80, true);
  if(timer0 == NULL)
  {
    Serial.println("定时器配置失败");
  }else{
    Serial.println("定时器配置成功");
    //连接中断函数
    timerAttachInterrupt(timer0, timer0_IRQ, true);
    //设置定时器 (50000/1000000)s 即50ms 自动重载 
    timerAlarmWrite(timer0,50000,true);
    //启动定时器
    timerAlarmEnable(timer0);
  }

  // 初始化EEPROM
  initEEPROM(); 
  // 尝试从EEPROM加载名称
  deviceName = loadNameFromEEPROM(); 
  // Serial.println("deviceName from eeprom");
  // Serial.println(deviceName);
  //得到ID
  chipId = String((uint32_t)ESP.getEfuseMac(), HEX);
  chipId.toUpperCase();
  Serial.println("chipId:");
  Serial.println(chipId);
  initBLE();

}

uint16_t combineBytes(uint8_t lowByte, uint8_t highByte) {
  // 将高字节左移8位，然后与低字节组合
  return (static_cast<uint16_t>(highByte) << 8) | lowByte;
}

void loop() {
  // put your main code here, to run repeatedly:
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    //Serial.println("start advertising");
    oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
    // do stuff here on connecting
    oldDeviceConnected = deviceConnected;
  }
  int actualContiueTime = 500;
  if(!dataQueue.isEmpty())
  {
    String cmdData = dataQueue.dequeue();
    int16_t speed = combineBytes(cmdData[5], cmdData[4]);
    // Serial.print("speed = ");
    // Serial.print(speed);
    uint8_t continueTime = cmdData[6];
    // Serial.print("continueTime = ");
    // Serial.println(continueTime);
    int16_t actualSpeed = speed * 0.06;
    actualContiueTime = continueTime * 100;
    //设置目标速度
    targetSpeed = actualSpeed;

  }  
    Serial.print("targetSpeed = ");
    Serial.print(targetSpeed);
    Serial.print("   rpm  ");
    Serial.print("currentSpeed = ");
    Serial.print(currentSpeed);
    Serial.print("  rpm  ");
    Serial.print("currentPWM = ");
    Serial.println(pwmOutput);
    delay(actualContiueTime);
}
