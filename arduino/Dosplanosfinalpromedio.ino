#include <Wire.h>

// --- CONFIGURACIÓN DE HARDWARE ---
const int SDA_0 = 11; const int SCL_0 = 12;
const int SDA_1 = 17; const int SCL_1 = 18;
const int REF_PIN = 14;

// --- CONSTANTES DE CALIBRACIÓN (Actualizadas) ---
const float CAL_A_A = -6.759f;
const float CAL_B_A = 0.012f;
const float CAL_A_B = -21.112f; 
const float CAL_B_B = 0.0191f;

const uint32_t Fs = 1600;
const uint32_t Ts_us = 625; 
uint32_t t_next = 0;

// --- FILTRO PASABANDA ---
struct BandPass {
    float x_z1 = 0, x_z2 = 0, y_z1 = 0, y_z2 = 0;
    const float b0 = 0.1174f, b1 = 0.0000f, b2 = -0.1174f;
    const float a1 = -1.7321f, a2 = 0.7652f;
    float process(float x) {
        float y = b0 * x + b1 * x_z1 + b2 * x_z2 - a1 * y_z1 - a2 * y_z2;
        x_z2 = x_z1; x_z1 = x;
        y_z2 = y_z1; y_z1 = y;
        return y;
    }
} filterA, filterB;

// --- VARIABLES DE ESTADO ---
volatile uint32_t t_last_pulse = 0;
volatile uint32_t t_turn = 0;
volatile bool new_turn = false;

float I_accA = 0, Q_accA = 0, I_accB = 0, Q_accB = 0;
uint32_t sample_count = 0;
bool lockin_enabled = false;
bool speed_stable = false;

void IRAM_ATTR ref_isr() {
    uint32_t now = micros();
    uint32_t dt = now - t_last_pulse;
    if (dt > 7500) { 
        t_turn = dt;
        t_last_pulse = now;
        new_turn = true;
    }
}

void setup() {
    Serial.begin(500000);
    Wire.begin(SDA_0, SCL_0, 1000000);
    Wire1.begin(SDA_1, SCL_1, 1000000);

    auto initSensor = [](TwoWire &bus) {
        bus.beginTransmission(0x68); bus.write(0x7E); bus.write(0x11); bus.endTransmission();
        delay(50);
        bus.beginTransmission(0x68); bus.write(0x40); bus.write(0x2C); bus.endTransmission();
        bus.beginTransmission(0x68); bus.write(0x41); bus.write(0x03); bus.endTransmission();
    };
    initSensor(Wire);
    initSensor(Wire1);

    pinMode(REF_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(REF_PIN), ref_isr, RISING);
    Serial.println("SISTEMA CONTINUO OK. ENVIA 'L' PARA EMPEZAR.");
}

void loop() {
    uint32_t ahora = micros();

    if ((int32_t)(ahora - t_next) >= 0) {
        uint32_t t_sample = ahora;
        t_next += Ts_us;

        int16_t yA_raw = 0, yB_raw = 0;
        Wire.beginTransmission(0x68); Wire.write(0x14); Wire.endTransmission(false);
        if(Wire.requestFrom(0x68, 2) == 2) yA_raw = Wire.read() | (Wire.read() << 8);

        Wire1.beginTransmission(0x68); Wire1.write(0x14); Wire1.endTransmission(false);
        if(Wire1.requestFrom(0x68, 2) == 2) yB_raw = Wire1.read() | (Wire1.read() << 8);

        float yA = filterA.process((float)yA_raw);
        float yB = filterB.process((float)yB_raw);

        if (Serial.available()) {
            char c = Serial.read();
            if (c == 'L') { lockin_enabled = true; Serial.println("MODO CONTINUO ACTIVADO"); }
            if (c == 'S') { lockin_enabled = false; Serial.println("STOP"); }
        }

        if (new_turn) { new_turn = false; speed_stable = true; }

        if (lockin_enabled && speed_stable && t_turn > 0) {
            float age = (float)(t_sample - t_last_pulse);
            float theta = 6.283185f * (age / (float)t_turn);
            float rpm = 60000000.0f / (float)t_turn;

            float phA = (CAL_A_A + CAL_B_A * rpm) * 0.017453f;
            float phB = (CAL_A_B + CAL_B_B * rpm) * 0.017453f;

            I_accA += yA * cosf(theta - phA);
            Q_accA += yA * sinf(theta - phA);
            I_accB += yB * cosf(theta - phB);
            Q_accB += yB * sinf(theta - phB);
            sample_count++;

            // PROMEDIO CADA 3200 MUESTRAS (~2 SEGUNDOS)
            if (sample_count >= 3200) {
            float rpm_final = 60000000.0f / (float)t_turn;

            // SOLO IMPRIME SI ESTÁ EN LA FRANJA DESEADA (Ejemplo: 4800 a 5000)
            if (rpm_final >= 4800 && rpm_final <= 5000) {
            Serial.print("DATO VALIDO: ");
            Serial.print(I_accA/3200.0f, 2); Serial.print(", ");
            Serial.print(Q_accA/3200.0f, 2); Serial.print(", ");
            Serial.print(I_accB/3200.0f, 2); Serial.print(", ");
            Serial.print(Q_accB/3200.0f, 2); Serial.print(", ");
            Serial.println(rpm_final, 0);
        } else {
        //avisar que la velocidad no es la correcta
        Serial.print("DESCARTADO (RPM fuera de rango): ");
        Serial.println(rpm_final, 0);
    }

    // Reset siempre, se imprima o no, para volver a empezar
    I_accA = 0; Q_accA = 0; I_accB = 0; Q_accB = 0;
    sample_count = 0;
}





            
        }
    }
}
