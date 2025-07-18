// Librerías I2C para controlar el MPU6050
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo
// del estado de AD0. Si no se especifica, 0x68 está implícito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerómetro y giroscopio en los ejes x, y, z
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
    Serial.begin(57600); // Iniciando puerto serial
    Wire.begin(); // Iniciando I2C
    sensor.initialize(); // Iniciando el sensor

    if (sensor.testConnection())
        Serial.println("Sensor iniciado correctamente");
    else
        Serial.println("Error al iniciar el sensor");
}

void loop() {
    // Leer las aceleraciones y velocidades angulares
    sensor.getAcceleration(&ax, &ay, &az);
    sensor.getRotation(&gx, &gy, &gz);

    // Mostrar las lecturas separadas por un [tab]
    Serial.print("a[x y z] g[x y z]:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
}
