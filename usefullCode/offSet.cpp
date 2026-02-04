/*
  Copy and paste thhis code in the main file of "src" folder of your PlatformIO project to calibrate the MPU6050 sensor.
  After running this code, it will print the offset values to be used in your main application. 

  You can't uplad this code directly from this file because PlatformIO requires the main code to be in "src" folder with all libraries
*/

#include <Arduino.h>
#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

// Pin I2C ESP32 (Default)
#define SDA_PIN 21
#define SCL_PIN 22

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(2000); // Attendi che la seriale sia pronta

  Serial.println(F("---------------------------------------"));
  Serial.println(F("CALIBRAZIONE MPU6050 - PLATFORMIO"));
  Serial.println(F("---------------------------------------"));
  
  // Inizializza MPU
  mpu.initialize();
  
  // Verifica connessione
  Serial.println(F("Test connessione..."));
  if(mpu.testConnection()){
    Serial.println(F("MPU6050 connesso correttamente!"));
  } else {
    Serial.println(F("MPU6050 NON trovato. Controlla i cavi SDA/SCL/VCC/GND."));
    while(1); // Blocca tutto se non trova il sensore
  }

  Serial.println(F("\nIMPORTANTE: Tieni il sensore ASSOLUTAMENTE FERMO e IN PIANO."));
  Serial.println(F("La calibrazione iniziera' tra 3 secondi..."));
  delay(1000); Serial.print("3.. ");
  delay(1000); Serial.print("2.. ");
  delay(1000); Serial.println("1.. VIA!");

  // --- FASE DI CALIBRAZIONE AUTOMATICA ---
  // Il sensore farà diverse letture per trovare lo zero.
  
  Serial.println(F("Calibrazione Accelerometro in corso..."));
  mpu.CalibrateAccel(6); // 6 iterazioni di calibrazione
  
  Serial.println(F("Calibrazione Giroscopio in corso..."));
  mpu.CalibrateGyro(6);  // 6 iterazioni di calibrazione

  Serial.println(F("\n---------------------------------------"));
  Serial.println(F("CALIBRAZIONE COMPLETATA!"));
  Serial.println(F("Copia questi valori nel tuo codice principale:"));
  Serial.println(F("---------------------------------------"));
  
  // Stampa i risultati formattati per il copia-incolla
  mpu.PrintActiveOffsets();
  
  Serial.println(F("---------------------------------------"));
  Serial.println(F("Verifica: Ora stampo l'accelerazione corretta (Dovrebbe essere ~0 eccetto Z)"));
}

void loop() {
  // Mostra i dati corretti per verificare che la calibrazione abbia funzionato
  // Se il sensore è piatto: X e Y ~0, Z ~16384 (1G)
  Serial.print("X: "); Serial.print(mpu.getAccelerationX());
  Serial.print("\tY: "); Serial.print(mpu.getAccelerationY());
  Serial.print("\tZ: "); Serial.println(mpu.getAccelerationZ());
  delay(200);
}