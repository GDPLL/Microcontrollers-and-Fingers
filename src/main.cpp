#include <Arduino.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "lda_model.h"
#include <iostream>
#include <cstdio>

// ========== 引脚定义 ==========
#define EMG_PIN1 4  // 肌电通道1
#define EMG_PIN2 5  // 肌电通道2
#define SERVO_PIN 6 // 舵机信号线
#define BTN_BOOT 0  // 板载BOOT按钮（低电平有效）

// ========== 采样参数 ==========
#define SAMPLE_RATE_HZ 2000                             // 采样频率 2000Hz
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)   // 500微秒
#define WINDOW_MS 100                                   // 窗口长度 100ms
#define WINDOW_SIZE (SAMPLE_RATE_HZ * WINDOW_MS / 1000) // 200

// ========== 全局变量 ==========
// 关键参数
int rest1 = 0, rest2 = 0; // 放松基准值1，2
int max1 = 0, max2 = 0;   // 收紧基准值1，2
bool calibrated = false;

// ===== BLE =====
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer *pServer = NULL;                   // BLE服务器
BLECharacteristic *pTxCharacteristic = NULL; // 全局参数特征
bool deviceConnected = false;

// BLE 发送定时器
unsigned long lastBLESendTime = 0;
const unsigned long BLE_SEND_INTERVAL_MS = 100; // 每100ms发一次

// 控制模式: 0=比例, 1=阈值, 2=手势
int mode = 0;

// 舵机参数
Servo servo; // 舵机类
int currentAngle = 90;
int targetAngle = 90;
const int SMOOTH_STEP = 2; // 平滑步长（度/更新）

// 滑动窗口缓冲区
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufIdx = 0;
bool windowReady = false;

// 时间控制
unsigned long lastSampleTime = 0;
unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 20; // 舵机更新周期20ms

// LDA 预测函数
int lda_predict(float *feat)
{
    // 标准化
    float normFeat[LDA_N_FEATURES];
    for (int i = 0; i < LDA_N_FEATURES; i++)
    {
        normFeat[i] = (feat[i] - FEATURE_MEAN[i]) / FEATURE_STD[i];
    }
    // 计算判别函数值
    float scores[LDA_N_CLASSES - 1];
    for (int i = 0; i < LDA_N_CLASSES - 1; i++)
    {
        scores[i] = LDA_INTERCEPT[i];
        for (int j = 0; j < LDA_N_FEATURES; j++)
        {
            scores[i] += normFeat[j] * LDA_COEF[i][j];
        }
    }
    // 取最大分数对应的类别
    int pred = 0;
    float maxScore = scores[0];
    for (int i = 1; i < LDA_N_CLASSES - 1; i++)
    {
        if (scores[i] > maxScore)
        {
            maxScore = scores[i];
            pred = i;
        }
    }
    return pred;
}

// 本地BLE连接服务，提供连接断链处理
class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("✅ BLE 设备已连接");
    };
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("❌ BLE 设备已断开，重新广播...");
        pServer->startAdvertising();
    }
};

// 肌电采集与永久写入程序（>阻塞程序，误差大 >添加完成事件到前端）
void calibrate()
{
    Serial.println("\n=== CALIBRATION ===");

    // 放松采集
    Serial.println("1. Relax muscles, then press BOOT button");
    while (digitalRead(BTN_BOOT) == HIGH)
        delay(10); // 获取板载按钮电平，等待开始
    delay(200);
    long sum1 = 0, sum2 = 0;
    for (int i = 0; i < 100; i++) // 读取两路肌电ADC转换数值，取平均
    {
        sum1 += analogRead(EMG_PIN1);
        sum2 += analogRead(EMG_PIN2);
        delay(5);
    }
    rest1 = sum1 / 100;
    rest2 = sum2 / 100;
    Serial.printf("Rest values: CH1=%d, CH2=%d\n", rest1, rest2);

    // 收缩肌肉1采集最大值
    Serial.println("2. Contract muscle 1 (e.g., flex wrist), then press BOOT");
    while (digitalRead(BTN_BOOT) == HIGH)
        delay(10);
    delay(200);
    max1 = 0;
    for (int i = 0; i < 100; i++)
    {
        int v = analogRead(EMG_PIN1);
        if (v > max1)
            max1 = v;
        delay(5);
    }
    Serial.printf("Max CH1: %d\n", max1);

    // 收缩肌肉2采集最大值
    Serial.println("3. Contract muscle 2 (e.g., extend wrist), then press BOOT");
    while (digitalRead(BTN_BOOT) == HIGH)
        delay(10);
    delay(200);
    max2 = 0;
    for (int i = 0; i < 100; i++)
    {
        int v = analogRead(EMG_PIN2);
        if (v > max2)
            max2 = v;
        delay(5);
    }
    Serial.printf("Max CH2: %d\n", max2);

    // 写入数据
    calibrated = true;
    EEPROM.begin(512);
    EEPROM.put(0, rest1);
    EEPROM.put(4, rest2);
    EEPROM.put(8, max1);
    EEPROM.put(12, max2);
    EEPROM.put(16, calibrated);
    EEPROM.commit();
    EEPROM.end();
    Serial.println("Calibration saved.");
}

// 读取永久数据
void loadCalibration()
{
    EEPROM.begin(512);
    EEPROM.get(0, rest1);
    EEPROM.get(4, rest2);
    EEPROM.get(8, max1);
    EEPROM.get(12, max2);
    EEPROM.get(16, calibrated);
    EEPROM.end();
    if (calibrated)
    {
        Serial.printf("Loaded calibration: rest(%d,%d) max(%d,%d)\n", rest1, rest2, max1, max2);
    }
}

// 向订阅客户端发送数据
void sendBLEMessage(const String &msg)
{
    if (deviceConnected && pTxCharacteristic)
    {
        pTxCharacteristic->setValue(msg.c_str());
        pTxCharacteristic->notify();
        // 可选：串口打印调试，避免频繁打印可注释掉
        // Serial.println("📤 BLE: " + msg);
    }
}

// 安全设置目标角度
void setServoAngle(int angle)
{
    targetAngle = constrain(angle, 0, 180);
}

// 舵机平滑控制
void updateServo()
{
    if (currentAngle < targetAngle)
    {
        currentAngle += SMOOTH_STEP;
        if (currentAngle > targetAngle)
            currentAngle = targetAngle;
    }
    else if (currentAngle > targetAngle)
    {
        currentAngle -= SMOOTH_STEP;
        if (currentAngle < targetAngle)
            currentAngle = targetAngle;
    }
    servo.write(currentAngle);
}

// 控制逻辑（20ms）
void computeControl()
{
    if (!calibrated)
        return;

    // 数据处理
    float rms1 = 0, rms2 = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) // 从滑动窗口用RMS处理
    {
        rms1 += buffer1[i] * buffer1[i];
        rms2 += buffer2[i] * buffer2[i];
    }
    rms1 = sqrt(rms1 / WINDOW_SIZE);
    rms2 = sqrt(rms2 / WINDOW_SIZE);
    float norm1 = constrain((float)(rms1 - rest1) / (max1 - rest1), 0.0f, 1.0f); // 归一化计算发力比
    float norm2 = constrain((float)(rms2 - rest2) / (max2 - rest2), 0.0f, 1.0f);

    // 计算目标舵机角度
    int angle = 90;
    if (mode == 0)
    { // 比例模式
        float diff = norm1 - norm2;
        angle = 90 + (int)(diff * 90);
        angle = constrain(angle, 0, 180);
    }
    else if (mode == 1)
    { // 阈值模式
        if (norm1 > 0.4)
            angle = 0;
        else if (norm2 > 0.4)
            angle = 180;
        else
            angle = 90;
    }
    else if (mode == 2)
    { // 手势模式（LDA）
        float feat[LDA_N_FEATURES] = {rms1, rms2};
        int gesture = lda_predict(feat);
        Serial.printf("Gesture: %d\n, gesture");
        // 根据手势映射角度（根据你的模型类别调整）
        switch (gesture)
        {
        case 0:
            angle = 90;
            break; // 静息
        case 1:
            angle = 0;
            break; // 握拳
        case 2:
            angle = 180;
            break; // 伸腕
        default:
            angle = currentAngle;
            break; // 其他类别（如共收缩）保持
        }
    }
    setServoAngle(angle);

    // 调试输出（每100ms一次）
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100)
    {
        lastPrint = millis();
        Serial.printf("RMS:%.1f %.1f  Norm:%.2f %.2f  Angle:%d\n", rms1, rms2, norm1, norm2, targetAngle);
    }
}

// 采样逻辑（2000Hz）
void sampling()
{
    int v1 = analogRead(EMG_PIN1); // 读取EMG数据
    int v2 = analogRead(EMG_PIN2);
    buffer1[bufIdx] = v1; // 写入滑动缓存区
    buffer2[bufIdx] = v2;
    bufIdx++;
    if (bufIdx >= WINDOW_SIZE)
    {
        bufIdx = 0;
        windowReady = true;
    }
}

// 串口调试指令
void handleSerial()
{
    if (!Serial.available())
        return;
    char c = tolower(Serial.read());
    switch (c)
    {
    case 'c':
        calibrate();
        break;
    case 'm':
        mode = (mode + 1) % 3;
        Serial.printf("Mode: %s\n", mode == 0 ? "PROPORTIONAL" : (mode == 1 ? "THRESHOLD" : "GESTURE"));
        break;
    case '0':
        setServoAngle(0);
        Serial.println("Manual 0°");
        break;
    case '5':
        setServoAngle(90);
        Serial.println("Manual 90°");
        break;
    case '9':
        setServoAngle(180);
        Serial.println("Manual 180°");
        break;
    case 'd':
        Serial.printf("Rest: %d %d, Max: %d %d, Calibrated: %d\n", rest1, rest2, max1, max2, calibrated);
        break;
    }
    while (Serial.available())
        Serial.read();
}

// ========== 初始化 ==========
void setup()
{
    // 硬件初始化
    Serial.begin(115200); // 波特
    delay(2000);
    Serial.println("\n2-Channel EMG Gesture Control with LDA");
    pinMode(EMG_PIN1, INPUT); // 初始化输入
    pinMode(EMG_PIN2, INPUT);
    pinMode(BTN_BOOT, INPUT_PULLUP);
    servo.attach(SERVO_PIN); // 初始化舵机
    servo.write(90);
    currentAngle = 90;
    targetAngle = 90;
    loadCalibration(); // 读取数据
    Serial.println("Commands: c=calibrate, m=mode, 0/5/9=manual, d=debug");
    Serial.println("Mode 0=PROP, 1=THRESH, 2=GESTURE (LDA)");

    // 初始化 BLE
    BLEDevice::init("ESP32_Bridge");                             // 启动 BLE，命名与 Python 代码中的 ESP32_NAME 一致
    pServer = BLEDevice::createServer();                         // 启用 BLE 服务器
    pServer->setCallbacks(new MyServerCallbacks());              // 设置回调
    BLEService *pService = pServer->createService(SERVICE_UUID); // 创建服务
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_READ);       // 创建新特征
    pTxCharacteristic->addDescriptor(new BLE2902()); // 数据流描述符
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // 获取广播
    pAdvertising->addServiceUUID(SERVICE_UUID); //添加服务到广播
    pAdvertising->setScanResponse(true);        //启用响应包
    BLEDevice::startAdvertising();              //启用广播
    Serial.println("✅ BLE 初始化完成，等待连接...");
}

// ========== 主循环 ==========
void loop()
{
    // 串口指令
    handleSerial();

    // 高速采样（2000Hz）
    unsigned long nowUs = micros(); // 获取当前时间
    if (nowUs - lastSampleTime >= SAMPLE_INTERVAL_US)
    {
        lastSampleTime = nowUs;
        sampling(); // 采样逻辑
    }

    // 每20ms执行一次控制（舵机更新、特征提取、推理）
    if (windowReady && (millis() - lastControlTime >= CONTROL_INTERVAL_MS))
    {
        lastControlTime = millis();
        computeControl(); // 中央逻辑
    }

    // 舵机平滑更新（高频，无延迟）
    updateServo();
    // ===== BLE 数据定时发送（每 100ms） =====
    if (deviceConnected && (millis() - lastBLESendTime >= BLE_SEND_INTERVAL_MS))
    {
        lastBLESendTime = millis();

        // 获取最新的 RMS 值（需从 computeControl 中提取，这里我们重新计算一次，但为了效率，可以在 computeControl 中保存全局变量）
        // 方法1：在 computeControl 中计算 rms1, rms2 后存入全局变量，这里直接使用
        // 方法2：临时重新计算（会消耗一点时间，但窗口数据还在，可以接受）
        // 这里采用方法1，需要新增几个全局变量：float lastRms1, lastRms2; 并在 computeControl 中赋值。
        // 下面给出采用方法2的示例（简单但稍耗CPU）
        float rms1 = 0, rms2 = 0;
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            rms1 += buffer1[i] * buffer1[i];
            rms2 += buffer2[i] * buffer2[i];
        }
        rms1 = sqrt(rms1 / WINDOW_SIZE);
        rms2 = sqrt(rms2 / WINDOW_SIZE);

        // 格式: M=模式 C=当前角度 T=目标角度 A=静息1 B=静息2
        char bleBuf[64];
        snprintf(bleBuf, sizeof(bleBuf), "M%dC%dT%dA%fB%.1f",
                 mode, currentAngle, targetAngle, rms1, rms2);
        sendBLEMessage(bleBuf);
    }
    // 短暂延迟，让出CPU
    delay(1);
}