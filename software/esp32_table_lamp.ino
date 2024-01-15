#define BLINKER_WIFI
#define BLINKER_MIOT_LIGHT   //支持小爱同学
#define BLINKER_WITHOUT_SSL  //非SSL加密通信接入，省堆栈
#include <Blinker.h>
#include "OneButton.h"  //按键控制库
#include <ESP32Encoder.h>
//nvs区域调用接口
#include <Preferences.h>

#define LED_PIN 16    // 板子上的灯
#define EC11_K_PIN 0  // 板子上的按键
#define EC11_A_PIN 34
#define EC11_B_PIN 35

// setting PWM properties
const int ledChannel = 0;
uint8_t dutyCycle = 5;  //初始亮度值  5

ESP32Encoder encoder;
OneButton SW(EC11_K_PIN, true);

int lastEncoderValue = 0;
int now_count = 0;
bool activate = true;

// 新建组件对象
BlinkerButton Button1("btn-233");  //注意：要和APP组件’数据键名’一致
char auth[] = "beac3900967c";      // blinker app提供的秘钥
char ssid[] = "Huaijin233";        // wifi 名字
char pswd[] = "1915409630";        // wifi 密码

bool wsState;  //灯的当前状态
uint8_t wsMode = BLINKER_CMD_MIOT_DAY;

void set_lamp_state(bool state) {

  if (state == true)  //开灯的状态
  {
    ledcWrite(ledChannel, dutyCycle);  //根据当前的亮度值来设置
  } else {
    ledcWrite(ledChannel, 0);
  }
  wsState = state;
}


// 调用外部的全局变量，把新的数据写入nvs区域
uint8_t get_and_updata_dutyCycle(uint8_t dutyCycle_value) {

  Preferences prefs;
  uint8_t ret;
  prefs.begin("DeviceInfo");

  //如果没有声明地址空间，则表示该设备未被初始化
  if (prefs.isKey("dutyCycle") != true) {
    prefs.clear();  //清空空间
    prefs.putUChar("dutyCycle", 5);
  }

  //当亮度值不为0时，则写入当前值
  if (dutyCycle_value != 0) {
    prefs.putUChar("dutyCycle", dutyCycle_value);
  }

  //获取存储的亮度信息
  ret = prefs.getUChar("dutyCycle");
  prefs.end();
  return ret;
}

// app 端按下按键即会执行该函数 回调函数
void button1_callback(const String& state) {

  BLINKER_LOG("get button state: ", state);
  if (state == "on") {  //开灯
    set_lamp_state(true);
    //digitalWrite(LED_PIN, LOW);
    // 反馈开关状态
    Button1.print("on");
  } else if (state == "off") {  //关灯

    set_lamp_state(false);
    //digitalWrite(LED_PIN, HIGH);
    // 反馈开关状态
    Button1.print("off");
  }
  Blinker.vibrate();
}

//小爱电源类操作的回调函数:
//当小爱同学向设备发起控制, 设备端需要有对应控制处理函数
void miotPowerState(const String& state) {
  BLINKER_LOG("need set power state: ", state);
  if (state == BLINKER_CMD_ON) {
    set_lamp_state(true);
    // digitalWrite(LED_PIN, HIGH);  //高电平点灯
    // wsState = true;
    BlinkerMIOT.powerState("on");
    BlinkerMIOT.print();

  } else if (state == BLINKER_CMD_OFF) {
    set_lamp_state(false);
    //digitalWrite(LED_PIN, LOW);  //低电平关灯
    //wsState = false;
    BlinkerMIOT.powerState("off");
    BlinkerMIOT.print();
  }
}


//小爱同学 的回调查询函数，照抄即可。主要是查询 当前灯的状态
void miotQuery(int32_t queryCode) {
  BLINKER_LOG("MIOT Query codes: ", queryCode);
  switch (queryCode) {
    case BLINKER_CMD_QUERY_ALL_NUMBER:
      BLINKER_LOG("MIOT Query All");
      BlinkerMIOT.powerState(wsState ? "on" : "off");
      BlinkerMIOT.color(0);
      BlinkerMIOT.mode(0);
      BlinkerMIOT.colorTemp(1000);
      BlinkerMIOT.brightness(1);
      BlinkerMIOT.print();
      break;
    case BLINKER_CMD_QUERY_POWERSTATE_NUMBER:
      BLINKER_LOG("MIOT Query Power State");
      BlinkerMIOT.powerState(wsState ? "on" : "off");
      BlinkerMIOT.print();
      break;
    default:
      BlinkerMIOT.powerState(wsState ? "on" : "off");
      BlinkerMIOT.color(0);
      BlinkerMIOT.mode(0);
      BlinkerMIOT.colorTemp(1000);
      BlinkerMIOT.brightness(1);
      BlinkerMIOT.print();
      break;
  }
}

//按键单击回调函数
void click() {
  Serial.println("click!");

  //单击按键开关灯
  if (wsState == true)
    set_lamp_state(false);
  else
    set_lamp_state(true);
}

//按键长按回调函数
void longclick() {
  Serial.println("longclick!");
}

//按键双击回调函数
void doubleclick() {
  Serial.println("doubleclick!");
}

void ec11_process(void) {

  //led的亮度10个等级
  now_count = encoder.getCount();
  if (now_count != lastEncoderValue) {  //EC11旋转值有更新

    if (wsState == true) {  //在开灯的时候，才可以调整灯的亮度

      if (now_count >= lastEncoderValue) {  //正向旋转
        if (dutyCycle < 255)
          dutyCycle += 25;
      } else {  //反向旋转
        if (dutyCycle > 5)
          dutyCycle -= 25;
      }
      get_and_updata_dutyCycle(dutyCycle);  //更新本地存储的亮度值
      ledcWrite(ledChannel, dutyCycle);     //根据当前的亮度值来设置
      Serial.println(dutyCycle);
    }

    if (!SW.isIdle()) {  //检测按键是否按下
    }
    lastEncoderValue = now_count;
  }
}

void setup() {
  // 初始化串口，并开启调试信息，调试用可以删除
  Serial.begin(115200);
  BLINKER_DEBUG.stream(Serial);

  ledcSetup(ledChannel, 5000, 8);           // configure LED PWM functionalitites
  ledcAttachPin(LED_PIN, ledChannel);       // attach the channel to the GPIO to be controlled
  dutyCycle = get_and_updata_dutyCycle(0);  //获取亮度信息
  set_lamp_state(true);                     //初始状态为开灯

  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachSingleEdge(EC11_A_PIN, EC11_B_PIN);

  pinMode(EC11_K_PIN, INPUT_PULLUP);

  //初始化按键事件检测
  SW.attachClick(click);
  SW.attachDoubleClick(doubleclick);
  SW.attachLongPressStop(longclick);
  SW.setDebounceTicks(20);  //滤波(ms)
  SW.setClickTicks(200);
  SW.setPressTicks(500);


  // 初始化blinker
  Blinker.begin(auth, ssid, pswd);
  Button1.attach(button1_callback);

  //小爱同学务必在回调函数中反馈该控制状态
  BlinkerMIOT.attachPowerState(miotPowerState);  //注册回调函数
  BlinkerMIOT.attachQuery(miotQuery);

  //开机后的初始化完成，设备关灯
  delay(500);
  set_lamp_state(false);
}

void loop() {

  ec11_process();

  SW.tick();

  Blinker.run();
}
