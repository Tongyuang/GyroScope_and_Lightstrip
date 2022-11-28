#include <Kalman.h>
#include <Wire.h>
#include <Math.h>
#include "FastLED.h"            // 此示例程序需要使用FastLED库
#define NUM_LEDS 2             // LED灯珠数量
#define DATA_PIN 5              // Arduino输出控制信号引脚
#define LED_TYPE WS2812         // LED灯带型号
#define COLOR_ORDER GRB         // RGB灯珠中红色、绿色、蓝色LED的排列顺序

#define LED_POWER 13  
#define COLOR_NUMBER 7

uint8_t max_bright = 128;       // LED亮度控制变量，可使用数值为 0 ～ 255， 数值越大则光带亮度越高
CRGB leds[NUM_LEDS]; 

int colorList_R[COLOR_NUMBER] = {255,255,255,0,0,0,139};
int colorList_G[COLOR_NUMBER] = {0,165,255,255,127,0,0};
int colorList_B[COLOR_NUMBER] = {0,0,0,0,255,255,255};
int color_ctr = 0;

float fRad2Deg = 57.295779513f; //将弧度转为角度的乘数
const int MPU = 0x68; //MPU-6050的I2C地址
const int nValCnt = 7; //一次读取寄存器的数量

const int nCalibTimes = 200; //校准时读数的次数
int calibData[nValCnt]; //校准数据

unsigned long nLastTime = 0; //上一次读数的时间
float fLastRoll = 0.0f; //上一次滤波得到的Roll角
float fLastPitch = 0.0f; //上一次滤波得到的Pitch角
Kalman kalmanRoll; //Roll角滤波器
Kalman kalmanPitch; //Pitch角滤波器

//const int gesture_detect_interval = 20; //
//int gesture_ctr = 0; 
int next_gesture = 0; // 0 for left, 1 for right

void setup() {
  Serial.begin(9600); //初始化串口，指定波特率
  Wire.begin(); //初始化Wire库
  WriteMPUReg(0x6B, 0); //启动MPU6050设备

  Calibration(); //执行校准
  nLastTime = micros(); //记录当前时间
  
  pinMode(LED_POWER,OUTPUT); 
  digitalWrite(LED_POWER,HIGH);

  LEDS.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS); 
  FastLED.setBrightness(max_bright);

  for(int i = 0; i<NUM_LEDS; i++){
    leds[i] = CRGB::Blue;          // 设置灯带中第一个灯珠颜色为红色，leds[0]为第一个灯珠，leds[1]为第二个灯珠
  }

  FastLED.show();       
}

void loop() {
  int readouts[nValCnt];
  ReadAccGyr(readouts); //读出测量值
  
  float realVals[7];
  Rectify(readouts, realVals); //根据校准的偏移量进行纠正

  //计算加速度向量的模长，均以g为单位
  float fNorm = sqrt(realVals[0] * realVals[0] + realVals[1] * realVals[1] + realVals[2] * realVals[2]);
  float fRoll = GetRoll(realVals, fNorm); //计算Roll角
  if (realVals[1] > 0) {
    fRoll = -fRoll;
  }
  float fPitch = GetPitch(realVals, fNorm); //计算Pitch角
  if (realVals[0] < 0) {
    fPitch = -fPitch;
  }

  //计算两次测量的时间间隔dt，以秒为单位
  unsigned long nCurTime = micros();
  float dt = (double)(nCurTime - nLastTime) / 1000000.0;
  //对Roll角和Pitch角进行卡尔曼滤波
  float fNewRoll = kalmanRoll.getAngle(fRoll, realVals[4], dt);
  float fNewPitch = kalmanPitch.getAngle(fPitch, realVals[5], dt);
  //跟据滤波值计算角度速
  float fRollRate = (fNewRoll - fLastRoll) / dt;
  float fPitchRate = (fNewPitch - fLastPitch) / dt;
  

 //更新Roll角和Pitch角
  fLastRoll = fNewRoll;
  fLastPitch = fNewPitch;
  //更新本次测的时间
  nLastTime = nCurTime;


  //向串口打印输出Roll角和Pitch角，运行时在Arduino的串口监视器中查看

    // detect gesture
  if(fPitchRate>100){ // TODO: Change this
      if(next_gesture==1){
      Serial.print("right!");
      Serial.print("\n");
      next_gesture = 0;
      // color
      color_ctr = (color_ctr+1)%COLOR_NUMBER;
      for(int i = 0; i<NUM_LEDS; i++){
          leds[i] = CRGB(colorList_R[color_ctr],colorList_G[color_ctr],colorList_B[color_ctr]);
        }
      FastLED.show();       
      }


   }
  else if(fPitchRate<-120){
      if(next_gesture==0){
      Serial.print("left!");
      Serial.print("\n");
      next_gesture = 1;
      //color
      color_ctr = (color_ctr+1)%COLOR_NUMBER;
      for(int i = 0; i<NUM_LEDS; i++){
          //leds[i] = CRGB::Red;          // 设置灯带中第一个灯珠颜色为红色，leds[0]为第一个灯珠，leds[1]为第二个灯珠
          leds[i] = CRGB(colorList_R[color_ctr],colorList_G[color_ctr],colorList_B[color_ctr]);
        }
      FastLED.show();       
      }
  }
  
 
  
  // Serial.print("Roll:");
  // Serial.print(fNewRoll); Serial.print('(');
  // Serial.print(fRollRate); Serial.print("),\tPitch:");
  // Serial.print(fNewPitch); Serial.print('(');
  // Serial.print(fPitchRate); Serial.print(")\n");
  // delay(10);

}

//向MPU6050写入一个字节的数据
//指定寄存器地址与一个字节的值
void WriteMPUReg(int nReg, unsigned char nVal) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.write(nVal);
  Wire.endTransmission(true);
}

//从MPU6050读出一个字节的数据
//指定寄存器地址，返回读出的值
unsigned char ReadMPUReg(int nReg) {
  Wire.beginTransmission(MPU);
  Wire.write(nReg);
  Wire.requestFrom(MPU, 1, true);
  Wire.endTransmission(true);
  return Wire.read();
}

//从MPU6050读出加速度计三个分量、温度和三个角速度计
//保存在指定的数组中
void ReadAccGyr(int *pVals) {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.requestFrom(MPU, nValCnt * 2, true);
  Wire.endTransmission(true);
  for (long i = 0; i < nValCnt; ++i) {
    pVals[i] = Wire.read() << 8 | Wire.read();
  }
}

//对大量读数进行统计，校准平均偏移量
void Calibration()
{
  float valSums[7] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0};
  //先求和
  for (int i = 0; i < nCalibTimes; ++i) {
    int mpuVals[nValCnt];
    ReadAccGyr(mpuVals);
    for (int j = 0; j < nValCnt; ++j) {
      valSums[j] += mpuVals[j];
    }
  }
  //再求平均
  for (int i = 0; i < nValCnt; ++i) {
    calibData[i] = int(valSums[i] / nCalibTimes);
  }
  calibData[2] += 16384; //设芯片Z轴竖直向下，设定静态工作点。
}

//算得Roll角。算法见文档。
float GetRoll(float *pRealVals, float fNorm) {
  float fNormXZ = sqrt(pRealVals[0] * pRealVals[0] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormXZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//算得Pitch角。算法见文档。
float GetPitch(float *pRealVals, float fNorm) {
  float fNormYZ = sqrt(pRealVals[1] * pRealVals[1] + pRealVals[2] * pRealVals[2]);
  float fCos = fNormYZ / fNorm;
  return acos(fCos) * fRad2Deg;
}

//对读数进行纠正，消除偏移，并转换为物理量。公式见文档。
void Rectify(int *pReadout, float *pRealVals) {
  for (int i = 0; i < 3; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 16384.0f;
  }
  pRealVals[3] = pReadout[3] / 340.0f + 36.53;
  for (int i = 4; i < 7; ++i) {
    pRealVals[i] = (float)(pReadout[i] - calibData[i]) / 131.0f;
  }
}

