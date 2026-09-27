#include <Arduino.h>
#include <ESP32Servo.h>
#include <EEPROM.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

#include "lda_model.h"
#include <iostream>
#include <cstdio>

// ========== 引脚定义 ==========
#define EMG_PIN1 4  // 肌电通道1
#define EMG_PIN2 5  // 肌电通道2
#define SERVO_PIN 6 // 舵机信号线
#define BTN_BOOT 0  // 板载BOOT按钮

// ========== 采样参数 ==========
#define SAMPLE_RATE_HZ 2000                             // 采样频率 2000Hz
#define SAMPLE_INTERVAL_US (1000000 / SAMPLE_RATE_HZ)   // 500微秒
#define WINDOW_MS 50                                    // 窗口长度 50ms
#define WINDOW_SIZE (SAMPLE_RATE_HZ * WINDOW_MS / 1000) // 100

// ========== 串口日志（USB CDC + UART0 双通道） ==========
// ESP32-S3 开发板通常有两个 USB 口：
//   1) 原生 USB 口（USB-Serial-JTAG）：本工程 ARDUINO_USB_CDC_ON_BOOT=1，Serial 走这里
//   2) UART 调试口（CP2102/CH340 桥接，GPIO43/44）：对应 Serial0
// 日志同时输出、命令同时接收，避免"接错口 + 命令没反应"的问题。
#define ENABLE_SERIAL_ECHO 1 // 回显收到的每个命令，便于确认输入到底有没有进芯片

#define LOGLN(x)           \
    do                     \
    {                      \
        Serial.println(x); \
        Serial0.println(x);\
    } while (0)

#define LOGF(...)                     \
    do                                \
    {                                 \
        Serial.printf(__VA_ARGS__);   \
        Serial0.printf(__VA_ARGS__);  \
    } while (0)

// ========== 全局变量 ==========
// 关键参数
int rest1 = 0, rest2 = 0; // 放松基准值1，2
int max1 = 0, max2 = 0;   // 收紧基准值1，2
bool calibrated = false;

// ===== BLE =====
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define MODEL_CHAR_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E" // 模型通道
#define CMD_CHAR_UUID "6E400004-B5A3-F393-E0A9-E50E24DCCA9E"   // 指令通道

BLEServer *pServer = NULL;                      // BLE服务器
BLECharacteristic *pTxCharacteristic = NULL;    // 全局参数特征
BLECharacteristic *pModelCharacteristic = NULL; // 模型通道特征
BLECharacteristic *pCmdCharacteristic = NULL;   // 指令通道特征
bool deviceConnected = false;

// BLE 发送定时器
unsigned long lastBLESendTime = 0;
const unsigned long BLE_SEND_INTERVAL_MS = 100; // 每100ms发一次

// 控制模式: 0=比例, 1=阈值, 2=手势
int mode = 0;
int lastGesture = -1; // 最近一次 LDA 手势，用于调试打印

// 舵机参数
Servo servo; // 舵机类
int currentAngle = 90;
int targetAngle = 90;
const int SMOOTH_STEP = 2; // 平滑步长（度/更新）

// 归一化调整值
#define NORM_GAIN 2.0f

// 滑动窗口缓冲区
float buffer1[WINDOW_SIZE];
float buffer2[WINDOW_SIZE];
int bufIdx = 0;
bool windowReady = false;

// 时间控制
unsigned long lastSampleTime = 0;
unsigned long lastControlTime = 0;
const unsigned long CONTROL_INTERVAL_MS = 10; // 舵机/控制更新周期10ms

// ===== LDA 系数（NVS 可覆盖编译期默认值） =====
#define LDA_NVS_NS "lda"          // NVS 命名空间
#define LDA_NVS_KEY "blob"        // 模型键名
#define LDA_BLOB_MAGIC "LDA1"     // 模型标记
#define LDA_DISC_COUNT LDA_N_CLASSES       // 判别函数个数
#define LDA_BLOB_SIZE (6 + LDA_DISC_COUNT * LDA_N_FEATURES * 4 + LDA_DISC_COUNT * 4 + LDA_N_FEATURES * 8)
#define MODEL_END_MARK "__MODEL_END__" // 模型结束标记
#define MODEL_BUF_SIZE 512             // 模型分片缓冲上限

float ldaCoef[LDA_DISC_COUNT][LDA_N_FEATURES]; // 判别系数
float ldaIntercept[LDA_DISC_COUNT];            // 判别常数
float featMean[LDA_N_FEATURES];                // 特征均值
float featStd[LDA_N_FEATURES];                 // 特征标准差

uint8_t modelBuf[MODEL_BUF_SIZE]; // 模型分片缓冲
size_t modelLen = 0;              // 已收字节数

// LDA 预测函数
int lda_predict(float *feat)
{
    // 标准化
    float normFeat[LDA_N_FEATURES];
    for (int i = 0; i < LDA_N_FEATURES; i++)
    {
        normFeat[i] = (feat[i] - featMean[i]) / featStd[i];
    }
    // 计算判别函数值
    float scores[LDA_DISC_COUNT];
    for (int i = 0; i < LDA_DISC_COUNT; i++)
    {
        scores[i] = ldaIntercept[i];
        for (int j = 0; j < LDA_N_FEATURES; j++)
        {
            scores[i] += normFeat[j] * ldaCoef[i][j];
        }
    }
    // 取最大分数对应的类别
    int pred = 0;
    float maxScore = scores[0];
    for (int i = 1; i < LDA_DISC_COUNT; i++)
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
        LOGLN("BLE connected");
    };
    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        LOGLN("BLE disconnected, re-advertising...");
        pServer->startAdvertising();
    }
};

// 肌电采集与永久写入程序（>阻塞程序，误差大 >添加完成事件到前端）
void calibrate()
{
    LOGLN("\n=== CALIBRATION ===");

    // 放松采集
    LOGLN("1. Relax muscles, then press BOOT button");
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
    LOGF("Rest values: CH1=%d, CH2=%d\n", rest1, rest2);

    // 收缩肌肉1采集最大值
    LOGLN("2. Contract muscle 1 (e.g., flex wrist), then press BOOT");
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
    LOGF("Max CH1: %d\n", max1);

    // 收缩肌肉2采集最大值
    LOGLN("3. Contract muscle 2 (e.g., extend wrist), then press BOOT");
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
    LOGF("Max CH2: %d\n", max2);

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
    LOGLN("Calibration saved.");
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
        LOGF("Loaded calibration: rest(%d,%d) max(%d,%d)\n", rest1, rest2, max1, max2);
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

// 校验并解包 LDA 模型，校验失败返回 false
bool unpackLdaBlob(const uint8_t *blob, size_t len)
{
    if (len != LDA_BLOB_SIZE)
    {
        LOGF("main.cpp|unpackLdaBlob|模型长度错误:%u\n", (unsigned)len);
        return false;
    }
    if (memcmp(blob, LDA_BLOB_MAGIC, 4) != 0)
    {
        LOGLN("main.cpp|unpackLdaBlob|模型标记错误");
        return false;
    }
    if (blob[4] != LDA_N_FEATURES || blob[5] != LDA_DISC_COUNT)
    {
        LOGF("main.cpp|unpackLdaBlob|模型维度不符:%u %u\n", blob[4], blob[5]);
        return false;
    }
    size_t off = 6;
    memcpy(ldaCoef, blob + off, sizeof(ldaCoef));
    off += sizeof(ldaCoef);
    memcpy(ldaIntercept, blob + off, sizeof(ldaIntercept));
    off += sizeof(ldaIntercept);
    memcpy(featMean, blob + off, sizeof(featMean));
    off += sizeof(featMean);
    memcpy(featStd, blob + off, sizeof(featStd));
    return true;
}

// 载入 LDA 系数，NVS 有有效模型则覆盖编译期默认值
void loadLdaModel()
{
    for (int i = 0; i < LDA_DISC_COUNT; i++)
    {
        ldaIntercept[i] = LDA_INTERCEPT[i];
        for (int j = 0; j < LDA_N_FEATURES; j++)
            ldaCoef[i][j] = LDA_COEF[i][j];
    }
    for (int j = 0; j < LDA_N_FEATURES; j++)
    {
        featMean[j] = FEATURE_MEAN[j];
        featStd[j] = FEATURE_STD[j];
    }

    Preferences prefs;
    prefs.begin(LDA_NVS_NS, true);
    size_t n = prefs.getBytesLength(LDA_NVS_KEY);
    if (n == 0)
    {
        prefs.end();
        LOGLN("LDA model: header defaults");
        return;
    }
    uint8_t blob[LDA_BLOB_SIZE];
    n = prefs.getBytes(LDA_NVS_KEY, blob, sizeof(blob));
    prefs.end();
    if (!unpackLdaBlob(blob, n))
        return;
    LOGF("LDA model: NVS %u bytes\n", (unsigned)n);
}

// 落地 LDA 模型并立即生效
void applyLdaModel(const uint8_t *blob, size_t len)
{
    modelLen = 0; // 无论成败都清空缓冲，避免残包累积
    if (!unpackLdaBlob(blob, len))
        return;

    Preferences prefs;
    prefs.begin(LDA_NVS_NS, false);
    prefs.putBytes(LDA_NVS_KEY, blob, len);
    prefs.end();

    LOGF("LDA model applied, %u bytes\n", (unsigned)len);
    sendBLEMessage("ACK:MODEL:OK");
}

// 切换控制模式，0比例 1阈值 2手势
void setMode(int m)
{
    if (m < 0 || m > 2)
    {
        LOGF("main.cpp|setMode|模式越界:%d\n", m);
        return;
    }
    mode = m;
    lastGesture = -1;
    LOGF("Mode: %s\n", mode == 0 ? "PROPORTIONAL" : (mode == 1 ? "THRESHOLD" : "GESTURE"));
}

// 解析指令通道文本命令
void handleBleCommand(const String &cmd)
{
    if (cmd.startsWith("CMD:MODE:"))
    {
        setMode(cmd.substring(9).toInt());
        sendBLEMessage("ACK:MODE:" + String(mode));
        return;
    }
    LOGF("main.cpp|handleBleCommand|未知命令:%s\n", cmd.c_str());
}

// 指令通道写入回调
class CmdWriteCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pChar)
    {
        String cmd = String(pChar->getValue().c_str());
        cmd.trim();
        if (cmd.length() == 0)
        {
            LOGLN("main.cpp|CmdWriteCallbacks|收到空指令");
            return;
        }
        handleBleCommand(cmd);
    }
};

// 模型通道写入回调，分片累积到结束标记后落地
class ModelWriteCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pChar)
    {
        std::string chunk = pChar->getValue();
        if (chunk.empty())
        {
            LOGLN("main.cpp|ModelWriteCallbacks|收到空分片");
            return;
        }
        if (chunk == MODEL_END_MARK)
        {
            if (modelLen == 0)
            {
                LOGLN("main.cpp|ModelWriteCallbacks|模型数据为空");
                return;
            }
            applyLdaModel(modelBuf, modelLen);
            return;
        }
        if (modelLen + chunk.size() > MODEL_BUF_SIZE)
        {
            LOGF("main.cpp|ModelWriteCallbacks|模型超长:%u\n", (unsigned)(modelLen + chunk.size()));
            modelLen = 0;
            return;
        }
        memcpy(modelBuf + modelLen, chunk.data(), chunk.size());
        modelLen += chunk.size();
    }
};

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

    // 数据处理：先去直流 rest 再算 RMS。
    // 原实现直接对原始 ADC（直流偏置约 rest≈2048）求 RMS 再减 rest，
    // 结果近似 ac_rms²/(2·rest)，是幅度的二次项——小发力时几乎不变，
    // 表现为"发力后指头反应慢、要很用力才有反应"。
    float rms1 = 0, rms2 = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) // 去直流后求 RMS
    {
        float x1 = buffer1[i] - rest1;
        float x2 = buffer2[i] - rest2;
        rms1 += x1 * x1;
        rms2 += x2 * x2;
    }
    rms1 = sqrt(rms1 / WINDOW_SIZE);
    rms2 = sqrt(rms2 / WINDOW_SIZE);

    // 归一化：分母为校准时的交流幅度，配合 NORM_GAIN 补偿波峰因数
    float range1 = (max1 > rest1) ? (float)(max1 - rest1) : 1.0f;
    float range2 = (max2 > rest2) ? (float)(max2 - rest2) : 1.0f;
    float norm1 = constrain(NORM_GAIN * rms1 / range1, 0.0f, 1.0f);
    float norm2 = constrain(NORM_GAIN * rms2 / range2, 0.0f, 1.0f);

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
        lastGesture = lda_predict(feat);
        // 根据手势映射角度
        switch (lastGesture)
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
        if (mode == 2)
            LOGF("RMS:%.1f %.1f  Norm:%.2f %.2f  Gesture:%d  Angle:%d\n", rms1, rms2, norm1, norm2, lastGesture, targetAngle);
        else
            LOGF("RMS:%.1f %.1f  Norm:%.2f %.2f  Angle:%d\n", rms1, rms2, norm1, norm2, targetAngle);
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

// 命令帮助
void printHelp()
{
    LOGLN("-------- Serial commands --------");
    LOGLN("  c : calibrate (press BOOT at each step)");
    LOGLN("  m : switch mode (0=PROP 1=THRESH 2=GESTURE)");
    LOGLN("  0 : servo 0 deg (flex)");
    LOGLN("  5 : servo 90 deg (center)");
    LOGLN("  9 : servo 180 deg (extend)");
    LOGLN("  r : back to rest (90 deg)");
    LOGLN("  s : emergency stop (hold current position)");
    LOGLN("  d : dump calibration values");
    LOGLN("  h : this help");
    LOGLN("---------------------------------");
}

// 执行单条命令
void execCommand(char c)
{
#if ENABLE_SERIAL_ECHO
    LOGF(">> %c\n", c); // 回显：如果这里没输出，说明输入根本没进芯片（多半是监视器开在另一个 USB 口）
#endif
    switch (c)
    {
    case 'c':
        calibrate();
        break;
    case 'm':
        setMode((mode + 1) % 3);
        break;
    case '0':
        setServoAngle(0);
        LOGLN("Manual 0 deg");
        break;
    case '5':
        setServoAngle(90);
        LOGLN("Manual 90 deg");
        break;
    case '9':
        setServoAngle(180);
        LOGLN("Manual 180 deg");
        break;
    case 'r':
        setServoAngle(90);
        LOGLN("Manual 90 deg (rest)");
        break;
    case 's':
        targetAngle = currentAngle; // 急停：停在当前位置
        LOGLN("EMERGENCY STOP (hold current position)");
        break;
    case 'd':
        LOGF("Rest: %d %d, Max: %d %d, Calibrated: %d\n", rest1, rest2, max1, max2, calibrated);
        break;
    case 'h':
        printHelp();
        break;
    default:
        LOGF("Unknown command '%c' (press h for help)\n", c);
        break;
    }
}

// 取走某个串口上所有待处理字符。
// 原实现"读 1 个字符后 while(available) read() 清空缓存"，
// 一次到达多个字符时后面的命令会被直接丢掉，这里改为全部逐个处理。
void pollCommands(Stream &s)
{
    while (s.available() > 0)
    {
        int v = s.read();
        if (v < 0)
            break;
        char c = (char)tolower(v);
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
            continue; // 忽略换行/空格（不同终端行尾可能是 \r、\n 或 \r\n）
        execCommand(c);
    }
}

// 串口调试指令：USB CDC(Serial) 与 UART0(Serial0) 两个口同时监听
void handleSerial()
{
    pollCommands(Serial);
    pollCommands(Serial0);
}

// ========== 初始化 ==========
void setup()
{
    // 硬件初始化
    Serial.begin(115200);  // USB CDC（ESP32-S3 原生 USB 口）
    Serial0.begin(115200); // UART0（GPIO43/44，板载 USB 转串口芯片那一侧）
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0);   // 主机没打开 USB 串口时 CDC 写入会阻塞，置 0 表示不等待，避免拖慢 loop
#endif
    delay(2000);
    LOGLN("\n2-Channel EMG Gesture Control with LDA");
    pinMode(EMG_PIN1, INPUT); // 初始化输入
    pinMode(EMG_PIN2, INPUT);
    pinMode(BTN_BOOT, INPUT_PULLUP);
    servo.attach(SERVO_PIN); // 初始化舵机
    servo.write(90);
    currentAngle = 90;
    targetAngle = 90;
    loadCalibration(); // 读取数据
    loadLdaModel();    // 读取 LDA 系数
    printHelp();
    LOGLN("Mode 0=PROP, 1=THRESH, 2=GESTURE (LDA)");

    
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
    pModelCharacteristic = pService->createCharacteristic(
        MODEL_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR);      // 模型通道
    pModelCharacteristic->setCallbacks(new ModelWriteCallbacks());
    pCmdCharacteristic = pService->createCharacteristic(
        CMD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR);      // 指令通道
    pCmdCharacteristic->setCallbacks(new CmdWriteCallbacks());
    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // 获取广播
    pAdvertising->addServiceUUID(SERVICE_UUID); //添加服务到广播
    pAdvertising->setScanResponse(true);        //启用响应包
    BLEDevice::startAdvertising();              //启用广播
    LOGLN("BLE advertising...");
    // 看到下面这行说明 setup() 已跑完、可以正常接收串口命令
    LOGLN(">> Ready. Serial commands on USB CDC + UART0 (type h + Enter for help).");
    Serial.flush();
}

// ========== 主循环 ==========
void loop()
{
    // 串口指令
    handleSerial();

    // 高速采样2000Hz
    while ((micros() - lastSampleTime) >= SAMPLE_INTERVAL_US)
    {
        lastSampleTime += SAMPLE_INTERVAL_US;
        sampling(); // 采样逻辑
    }

    // 中央控制 10ms
    if (windowReady && (millis() - lastControlTime >= CONTROL_INTERVAL_MS))
    {
        lastControlTime = millis();
        computeControl(); // 中央逻辑
    }

    // 舵机平滑更新
    updateServo();


    //  BLE 数据定时发送100ms
    if (deviceConnected && (millis() - lastBLESendTime >= BLE_SEND_INTERVAL_MS))
    {
        lastBLESendTime = millis();

        // 获取最新的 RMS 值重新计算一次
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
        snprintf(bleBuf, sizeof(bleBuf), "M%dC%dT%dA%.1fB%.1f",
                 mode, currentAngle, targetAngle, rms1, rms2);
        sendBLEMessage(bleBuf);
    }
}