#include <Arduino.h>
#include <SPI.h>
#include "ad5941_board_glue.h"      // 引入 Glue 层头文件
#include "spi_hal.h"                // 引入 SPI HAL 头文件
#include "2Wire_Service.h"           // 您的应用层
extern "C" {
#include "ad5940.h"
}
/* ====== 缓冲区和对象实例 ====== */
#define APPBUFF_SIZE 512
uint32_t AppBuff[APPBUFF_SIZE];

/* ====== 共用 SPI 总线引脚 ====== */
static const int PIN_SCLK = 14; // ESP32 SPI 时钟线
static const int PIN_MISO = 12; // ESP32 SPI MISO线
static const int PIN_MOSI = 13; // ESP32 SPI MOSI线

/* ====== AD5941 特定引脚 ====== */
static const int CS_AD5941    = 26; // AD5941 片选引脚
static const int RESET_AD5941 = 25; // AD5941 复位引脚

void setup() {
  Serial.begin(115200); // 初始化串口
  delay(200);
  Serial.println("======================================");
  Serial.println(" AD5941 Setup Start ");
  Serial.println("======================================");

  // 1) 初始化 SPI 总线
  SpiHAL::beginBus(PIN_SCLK, PIN_MISO, PIN_MOSI);
  Serial.println("SPI bus initialized.");

  // 2) 创建 AD5941 的 SpiDevice 对象
  static SpiDevice ad5941_dev(CS_AD5941, 8000000 /* 8MHz */, MSBFIRST, SPI_MODE0);
  Serial.println("SpiDevice for AD5941 created.");

  // 3) 配置并初始化 Ad5941Glue 层
  Ad5941Glue::Config cfg;
  cfg.spi = &ad5941_dev;
  cfg.pin_reset = RESET_AD5941;
  Ad5941Glue::setup(cfg);
  Serial.println("Ad5941Glue setup complete.");

  // 4) 执行硬件复位
  Ad5941Glue::hardware_reset(1000, 100);
  Serial.println("AD5941 hardware reset performed.");
  AppBIOZCfg_init();
  // 5. 初始化AD5940芯片 (时钟等)
  AD5940PlatformCfg();
  
  // 6. 配置您的2线应用参数 (修改Rcal和AIN1)
  AD5940BIOZStructInit();

  Serial.println("Running RTIA Calibration... This may take a moment.");
  if(AppBIOZInit(AppBuff, APPBUFF_SIZE) != AD5940ERR_OK) {
    Serial.println("!!! AppBIOZInit Failed. Check connections. !!!");
    while(1); // 失败则停止
  }
  Serial.println("RTIA Calibration complete. Sequences loaded.");
  Serial.println("======================================");
  Serial.println(" Starting Measurement Loop... ");
  Serial.println("======================================");
}

void loop() {
  uint32_t temp;
  
  // 1. 手动触发一次测量序列 (SEQID_0)
  AD5940_SEQMmrTrig(SEQID_0);

  // 2. 轮询等待FIFO阈值中断标志位
  //    (注意：AD5940_INTCTestFlag 会通过SPI唤醒芯片来检查)
  while(AD5940_INTCTestFlag(AFEINTC_0, AFEINTSRC_DATAFIFOTHRESH) == bFALSE)
  {
    // 芯片在序列结束后会休眠，
    // 这里的轮询会短暂唤醒它检查标志，然后它会再次休眠
    delay(10); // 轮询间隔，例如10ms
  }

  // 3. 标志已置位，数据已准备好。
  //    调用 AppBIOZISR() 来处理数据。
  //    这个函数内部会:
  //    - 唤醒芯片 (AD5940_WakeUp)
  //    - 读FIFO (AD5940_FIFORd)
  //    - 清除中断标志 (AD5940_INTCClrFlag)
  //    - 为下一次扫频做准备 (AppBIOZRegModify)
  //    - 让芯片休眠 (AD5940_EnterSleepS)
  //    - 计算阻抗 (AppBIOZDataProcess)
  
  temp = APPBUFF_SIZE;
  AppBIOZISR(AppBuff, &temp); 

  // 4. 显示结果 (这个函数在 2Wire_Service.cpp 中，确保它是 public 的)
  BIOZShowResult(AppBuff, temp);

  // 5. 等待下一次测量
  //    这个延迟决定了您的采样率 (ODR)
  //    例如 200ms -> 5Hz (匹配原代码的 BIOZODR = 5)
  delay(200);
}
