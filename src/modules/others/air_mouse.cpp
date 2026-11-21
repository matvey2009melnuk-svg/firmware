#if defined(HAS_AIR_MOUSE)

#include "air_mouse.h"
#include "core/display.h"
#include "core/bus_HAL.h"
#include <Preferences.h>
#include "modules/badusb_ble/ducky_typer.h"
#include <globals.h>

// Мышь M5StickC Plus2: кнопки на тех же GPIO, что и в boards/m5stack-cplus2/interface.cpp
#define AM_BTN_A   37   // передняя (M5)
#define AM_BTN_B   39   // боковая
#define AM_BTN_PWR 35   // питание

// --- Оси: если курсор едет не туда, меняйте эти флаги ---
#define AM_SWAP_XY  0
#define AM_INVERT_X 0
#define AM_INVERT_Y 0

namespace {

// ---------------- Настройки ----------------
const float GAIN_DEFAULT = 12.0f;
const float GAIN_MIN = 1.0f;
const float GAIN_MAX = 40.0f;
const float GAIN_PRESETS[] = {4.0f, 8.0f, 12.0f, 18.0f, 25.0f, 32.0f};
const int GAIN_PRESET_COUNT = sizeof(GAIN_PRESETS) / sizeof(GAIN_PRESETS[0]);

const float DEADZONE = 0.9f;  // °/с
const float ALPHA = 0.50f;    // сглаживание
const int LOOP_MS = 10;
const uint32_t SCROLL_HOLD_MS = 400;
const uint32_t EXIT_HOLD_MS = 1000;
const uint32_t DOUBLE_CLICK_MS = 300;

// ---------------- Общее BLE-устройство ----------------
// Мышь работает через тот же BleKeyboard, что и BLE-клавиатура / медиа / BadBLE / презентер:
// один MAC, одно имя, один HID-дескриптор (клавиатура + медиа + мышь).
// Для компьютера это ОДНО устройство, оно остаётся спаренным и переподключается само.
HIDInterface *kbHid = nullptr;

inline BleKeyboard *kb() { return static_cast<BleKeyboard *>(kbHid); }
inline bool linked() { return kbHid != nullptr && kb()->isConnected(); }

// ---------------- Состояние ----------------
Preferences prefs;
float gain = GAIN_DEFAULT;
float gyroBias[3] = {0, 0, 0};
bool active = true;
bool scrollMode = false;
bool dragging = false;
uint8_t buttons = 0;
float accX = 0, accY = 0, smoothX = 0, smoothY = 0, wheelAcc = 0;
uint32_t stillSince = 0;
uint32_t btnBPressMs = 0;

// ---------------- MPU6886 (I2C 0x68, системная шина Plus2) ----------------
const uint8_t IMU_ADDR = 0x68;

bool imuWrite(uint8_t reg, uint8_t val) {
    TwoWire *w = getSysI2CBus();
    lockSysI2CBus();
    w->beginTransmission(IMU_ADDR);
    w->write(reg);
    w->write(val);
    bool ok = (w->endTransmission() == 0);
    unlockSysI2CBus();
    return ok;
}

bool imuReadGyro(float &gx, float &gy, float &gz) {
    TwoWire *w = getSysI2CBus();
    uint8_t buf[6];
    lockSysI2CBus();
    w->beginTransmission(IMU_ADDR);
    w->write(0x43);
    bool ok = (w->endTransmission(false) == 0);
    if (ok) ok = (w->requestFrom((uint8_t)IMU_ADDR, (uint8_t)6) == 6);
    if (ok) for (int i = 0; i < 6; i++) buf[i] = w->read();
    unlockSysI2CBus();
    if (!ok) return false;
    const float k = 2000.0f / 32768.0f;  // ±2000 °/с
    gx = (int16_t)((buf[0] << 8) | buf[1]) * k;
    gy = (int16_t)((buf[2] << 8) | buf[3]) * k;
    gz = (int16_t)((buf[4] << 8) | buf[5]) * k;
    return true;
}

bool imuInit() {
    imuWrite(0x6B, 0x80);  // reset
    delay(20);
    imuWrite(0x6B, 0x01);  // тактирование от PLL гироскопа
    delay(10);
    imuWrite(0x1C, 0x10);  // акселерометр ±8g (не используется)
    imuWrite(0x1B, 0x18);  // гироскоп ±2000 °/с
    imuWrite(0x1A, 0x01);  // цифровой фильтр
    imuWrite(0x19, 0x05);  // делитель частоты
    imuWrite(0x38, 0x00);
    delay(10);
    float x, y, z;
    return imuReadGyro(x, y, z);
}

// ---------------- Мышь ----------------
void mouseSend(int8_t dx, int8_t dy, int8_t wheel = 0) {
    if (kbHid == nullptr) return;
    kb()->mouseSend(buttons, dx, dy, wheel);
}

void mouseClick(uint8_t mask) {
    buttons |= mask;
    mouseSend(0, 0);
    delay(12);
    buttons &= (uint8_t)~mask;
    mouseSend(0, 0);
}

// ---------------- BLE ----------------
bool bleStart() {
    // functionId 4 = Air Mouse. Тот же MAC и имя, что у остальных BLE-функций Bruce.
    ducky_startKb(kbHid, true, 4);
    return kbHid != nullptr;
}

void bleStop() {
    // Корректно отключает компьютер и гасит BLE. Компьютер переподключится сам,
    // когда любая другая функция (или этот режим) снова запустит BLE с тем же MAC.
    if (kbHid != nullptr) safeCleanupDuckyBLE(kbHid);
}

// ---------------- Калибровка ----------------
bool sampleGyro(uint32_t ms, float mean[3], float sd[3]) {
    double s[3] = {0, 0, 0}, s2[3] = {0, 0, 0};
    int n = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < ms) {
        float x, y, z;
        if (imuReadGyro(x, y, z)) {
            s[0] += x; s[1] += y; s[2] += z;
            s2[0] += (double)x * x; s2[1] += (double)y * y; s2[2] += (double)z * z;
            n++;
        }
        delay(10);
    }
    if (n < 5) return false;
    for (int i = 0; i < 3; i++) {
        mean[i] = (float)(s[i] / n);
        double var = s2[i] / n - (double)mean[i] * mean[i];
        sd[i] = var > 0 ? sqrtf((float)var) : 0.0f;
    }
    return true;
}

void drawStatus(const char *state, uint16_t color);

void calibrate() {
    drawStatus("CALIBRATING", TFT_YELLOW);
    float bestMean[3] = {gyroBias[0], gyroBias[1], gyroBias[2]};
    float bestSd = 1e9f;
    for (int attempt = 0; attempt < 3; attempt++) {
        float mean[3], sd[3];
        if (!sampleGyro(1200, mean, sd)) continue;
        float sdMax = max(sd[0], max(sd[1], sd[2]));
        if (sdMax < bestSd) {
            bestSd = sdMax;
            for (int i = 0; i < 3; i++) bestMean[i] = mean[i];
        }
        if (sdMax < 4.0f) break;
        delay(400);
    }
    for (int i = 0; i < 3; i++) gyroBias[i] = bestMean[i];
    stillSince = 0;
    accX = accY = smoothX = smoothY = wheelAcc = 0;
}

// ---------------- Экран ----------------
uint32_t lastDrawKey = 0xFFFFFFFF;

void drawStatus(const char *state, uint16_t color) {
    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.setTextSize(1);
    tft.drawCentreString("AIR MOUSE", tftWidth / 2, 6, 1);
    tft.drawFastHLine(0, 18, tftWidth, bruceConfig.priColor);
    tft.setTextSize(3);
    tft.setTextColor(color, bruceConfig.bgColor);
    tft.drawCentreString(state, tftWidth / 2, 30, 1);
    lastDrawKey = 0xFFFFFFFF;
}

void drawUi() {
    const char *state;
    uint16_t color;
    uint8_t id;
    if (!linked()) { id = 0; state = "PAIRING"; color = TFT_ORANGE; }
    else if (!active) { id = 1; state = "PAUSED"; color = TFT_YELLOW; }
    else if (scrollMode) { id = 2; state = "SCROLL"; color = TFT_CYAN; }
    else if (dragging) { id = 3; state = "DRAG"; color = TFT_GREEN; }
    else { id = 4; state = "READY"; color = TFT_GREEN; }

    uint32_t key = ((uint32_t)id << 16) | (uint32_t)lroundf(gain * 10.0f);
    if (key == lastDrawKey) return;
    lastDrawKey = key;

    tft.fillScreen(bruceConfig.bgColor);
    tft.setTextSize(1);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawCentreString("AIR MOUSE", tftWidth / 2, 6, 1);
    tft.drawFastHLine(0, 18, tftWidth, bruceConfig.priColor);
    tft.setTextSize(3);
    tft.setTextColor(color, bruceConfig.bgColor);
    tft.drawCentreString(state, tftWidth / 2, 28, 1);

    char buf[40];
    tft.setTextSize(2);
    tft.setTextColor(TFT_WHITE, bruceConfig.bgColor);
    snprintf(buf, sizeof(buf), "SENS %.1f", gain);
    tft.drawCentreString(buf, tftWidth / 2, 66, 1);

    tft.setTextSize(1);
    tft.setTextColor(bruceConfig.priColor, bruceConfig.bgColor);
    tft.drawCentreString("A=LMB B=RMB B hold=scroll", tftWidth / 2, 100, 1);
    tft.drawCentreString("PWR x2=sens  PWR hold=exit", tftWidth / 2, 114, 1);
}

// ---------------- Движение ----------------
void trackGyroBias(float gx, float gy, float gz) {
    const float QUIET = 4.0f, K = 0.03f;
    float dx = gx - gyroBias[0], dy = gy - gyroBias[1], dz = gz - gyroBias[2];
    if (fabsf(dx) < QUIET && fabsf(dy) < QUIET && fabsf(dz) < QUIET) {
        if (stillSince == 0) stillSince = millis();
        if (millis() - stillSince > 500) {
            gyroBias[0] += dx * K; gyroBias[1] += dy * K; gyroBias[2] += dz * K;
        }
    } else {
        stillSince = 0;
    }
}

float applyDeadzone(float v) {
    if (fabsf(v) < DEADZONE) return 0.0f;
    return v > 0 ? v - DEADZONE : v + DEADZONE;
}

void updateMotion(float dt) {
    float gx, gy, gz;
    if (!imuReadGyro(gx, gy, gz)) return;
    trackGyroBias(gx, gy, gz);
    gx -= gyroBias[0]; gy -= gyroBias[1]; gz -= gyroBias[2];

    float yawRate = gz, pitchRate = gx;
#if AM_SWAP_XY
    float t = yawRate; yawRate = pitchRate; pitchRate = t;
#endif
    float vx = -yawRate, vy = -pitchRate;
#if AM_INVERT_X
    vx = -vx;
#endif
#if AM_INVERT_Y
    vy = -vy;
#endif
    vx = applyDeadzone(vx);
    vy = applyDeadzone(vy);
    smoothX += (vx - smoothX) * ALPHA;
    smoothY += (vy - smoothY) * ALPHA;

    if (scrollMode) {
        wheelAcc += smoothY * dt * 0.6f;
        if (wheelAcc >= 1.0f || wheelAcc <= -1.0f) {
            int8_t step = wheelAcc > 0 ? 1 : -1;
            mouseSend(0, 0, step);
            wheelAcc -= step;
        }
        return;
    }

    accX += smoothX * dt * gain;
    accY += smoothY * dt * gain;
    int dx = (int)accX, dy = (int)accY;
    if (dx == 0 && dy == 0) return;
    accX -= dx; accY -= dy;
    dx = constrain(dx, -127, 127);
    dy = constrain(dy, -127, 127);
    mouseSend((int8_t)dx, (int8_t)dy);
}

// ---------------- Чувствительность ----------------
void saveGain() { prefs.putFloat("gain", gain); }

void nextGainPreset() {
    int idx = 0;
    for (int i = 0; i < GAIN_PRESET_COUNT; i++) {
        if (GAIN_PRESETS[i] > gain + 0.01f) { idx = i; break; }
    }
    gain = GAIN_PRESETS[idx];
    saveGain();
}

void adjustGain(float delta) {
    gain = constrain(gain + delta, GAIN_MIN, GAIN_MAX);
    saveGain();
}

}  // namespace

// ======================= ТОЧКА ВХОДА =======================
void airMouseMode() {
    pinMode(AM_BTN_A, INPUT);
    pinMode(AM_BTN_B, INPUT);
    pinMode(AM_BTN_PWR, INPUT);

    prefs.begin("airmouse", false);
    gain = prefs.getFloat("gain", GAIN_DEFAULT);
    if (gain < GAIN_MIN || gain > GAIN_MAX) gain = GAIN_DEFAULT;

    drawStatus("INIT", TFT_YELLOW);
    if (!imuInit()) {
        displayWarning("IMU not found", true);
        prefs.end();
        return;
    }
    calibrate();
    if (!bleStart()) {
        // нет памяти под BLE (ошибка уже показана) — выходим в меню
        prefs.end();
        returnToMenu = true;
        return;
    }

    active = true; scrollMode = false; dragging = false; buttons = 0;
    bool prevA = false, prevB = false, prevPwr = false;
    uint32_t pwrDownMs = 0, lastClickMs = 0, lastTick = millis();
    int pwrClicks = 0;
    bool exitRequested = false;

    // ждём отпускания кнопок, на которых запускали режим
    while (digitalRead(AM_BTN_A) == LOW || digitalRead(AM_BTN_B) == LOW || digitalRead(AM_BTN_PWR) == LOW)
        delay(10);

    while (!exitRequested) {
        uint32_t now = millis();
        bool a = digitalRead(AM_BTN_A) == LOW;
        bool b = digitalRead(AM_BTN_B) == LOW;
        bool p = digitalRead(AM_BTN_PWR) == LOW;

        // ---- PWR ----
        if (p && !prevPwr) pwrDownMs = now;
        if (p && now - pwrDownMs > EXIT_HOLD_MS) { exitRequested = true; break; }
        if (!p && prevPwr) {
            pwrClicks++;
            lastClickMs = now;
        }
        if (pwrClicks > 0 && !p && now - lastClickMs > DOUBLE_CLICK_MS) {
            if (pwrClicks >= 2) {
                nextGainPreset();
            } else {
                active = !active;
                scrollMode = false; dragging = false; buttons = 0;
                mouseSend(0, 0);
                if (active) calibrate();
            }
            pwrClicks = 0;
        }
        prevPwr = p;

        // ---- A ----
        if (a && !prevA) {
            if (!active) adjustGain(0.5f);
            else if (linked()) { buttons |= 0x01; dragging = true; mouseSend(0, 0); }
        }
        if (!a && prevA && dragging) {
            dragging = false; buttons &= (uint8_t)~0x01; mouseSend(0, 0);
        }
        prevA = a;

        // ---- B ----
        if (b && !prevB) {
            if (!active) adjustGain(-0.5f);
            else btnBPressMs = now;
        }
        if (active && b && !scrollMode && now - btnBPressMs > SCROLL_HOLD_MS) {
            scrollMode = true; wheelAcc = 0;
        }
        if (!b && prevB) {
            if (scrollMode) { scrollMode = false; wheelAcc = 0; }
            else if (active && linked()) mouseClick(0x02);
        }
        prevB = b;

        // ---- движение ----
        if (now - lastTick >= (uint32_t)LOOP_MS) {
            float dt = min((now - lastTick) / 1000.0f, 0.05f);
            lastTick = now;
            if (active && linked()) updateMotion(dt);
        }

        static uint32_t lastUi = 0;
        if (now - lastUi > 200) { lastUi = now; drawUi(); }

        delay(1);
    }

    // ---- выход ----
    buttons = 0;
    mouseSend(0, 0);
    bleStop();
    prefs.end();
    while (digitalRead(AM_BTN_PWR) == LOW) delay(10);
    delay(200);
    // сбросить события кнопок, накопленные InputHandler за время работы режима
    check(SelPress); check(NextPress); check(PrevPress); check(EscPress); check(AnyKeyPress);
    returnToMenu = true;
}

#endif
