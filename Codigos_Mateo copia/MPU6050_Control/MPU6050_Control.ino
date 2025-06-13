#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 sensor;

int16_t ax, ay, az;
int16_t gx, gy, gz;
long tiempo_prev;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

void setup() {
    Serial.begin(57600);
    Wire.begin();
    sensor.initialize();

    if (sensor.testConnection()) {
        Serial.println("Sensor iniciado correctamente");
    } else {
        Serial.println("Error al iniciar el sensor");
    }

    tiempo_prev = millis();
}

void loop() {
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);

    dt = (millis() - tiempo_prev) / 1000.0;
    tiempo_prev = millis();

    float accel_ang_x = atan(ay / sqrt(pow(ax, 2) + pow(az, 2))) * (180.0 / 3.14);
    float accel_ang_y = atan(-ax / sqrt(pow(ay, 2) + pow(az, 2))) * (180.0 / 3.14);

    ang_x = 0.98 * (ang_x_prev + (gx / 131.0) * dt) + 0.02 * accel_ang_x;
    ang_y = 0.98 * (ang_y_prev + (gy / 131.0) * dt) + 0.02 * accel_ang_y;

    ang_x_prev = ang_x;
    ang_y_prev = ang_y;

    Serial.print("Rotación en X: ");
    Serial.print(ang_x);
    Serial.print("\tRotación en Y: ");
    Serial.println(ang_y);

    delay(10);
}
