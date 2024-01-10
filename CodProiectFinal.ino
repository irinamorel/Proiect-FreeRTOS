#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <Arduino.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
const char* ssid = "Galaxy A51F3F0"; // Numele rețelei WiFi
const char* password = "irina2908"; // Parola rețelei WiFi
const char* firebaseUrl = "https://termostat-986e1-default-rtdb.europe-west1.firebasedatabase.app/sensorData.json"; // URL-ul Firebase Realtime Database
const int RELEU_PIN = 5;

SemaphoreHandle_t xSemaphore = NULL;

void setup() {
    Serial.begin(9600);
    Serial.println(F("BME280 test"));

    // Conectarea la WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");

    // Inițializarea senzorului BME280
    if (!bme.begin(0x76)) {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }

    pinMode(RELEU_PIN, OUTPUT);
    digitalWrite(RELEU_PIN, LOW);

    // Inițializarea semaforului
    xSemaphore = xSemaphoreCreateBinary();

    // Verifică dacă semaforul a fost creat cu succes
    if (xSemaphore == NULL) {
        Serial.println("Semaphore creation failed!");
        while (1);
    }

    // Crearea sarcinilor
    xTaskCreate(readSensorData, "ReadSensorTask", 10000, NULL, 1, NULL);
    xTaskCreate(checkReleuState, "CheckReleuStateTask", 10000, NULL, 1, NULL);
}

void loop() {
    // Nimic aici - task-urile FreeRTOS gestionează funcționalitatea
}

void readSensorData(void * parameter) {
    for (;;) {
        float temperature = bme.readTemperature();
        float pressure = bme.readPressure() / 100.0F;
        float altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);
        float humidity = bme.readHumidity();

        sendDataToFirebase(temperature, pressure, altitude, humidity);

        // Dă drumul semaforului
        xSemaphoreGive(xSemaphore);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void checkReleuState(void * parameter) {
    for (;;) {
        // Așteaptă semaforul
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE) {
            if (WiFi.status() == WL_CONNECTED) {
                HTTPClient http;
                http.begin("https://termostat-986e1-default-rtdb.europe-west1.firebasedatabase.app/releuState.json");
                int httpCode = http.GET();

                if (httpCode > 0) {
                    String payload = http.getString();
                    if (payload.toInt() != 0 || payload == "0") {
                        bool releuState = payload.toInt() == 1 ? HIGH : LOW;
                        digitalWrite(RELEU_PIN, releuState);
                    }
                } else {
                    Serial.println("Error on HTTP request");
                }

                http.end();
            }
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void sendDataToFirebase(float temperature, float pressure, float altitude, float humidity) {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        http.begin(firebaseUrl);
        http.addHeader("Content-Type", "application/json");

        String httpRequestData = "{\"temperature\":" + String(temperature) +
                                ",\"pressure\":" + String(pressure) +
                                ",\"altitude\":" + String(altitude) +
                                ",\"humidity\":" + String(humidity) + "}";

        int httpResponseCode = http.PUT(httpRequestData);

        if (httpResponseCode > 0) {
            String response = http.getString();
            Serial.println(httpResponseCode);
            Serial.println(response);
        } else {
            Serial.print("Error on sending PUT: ");
            Serial.println(httpResponseCode);
        }

        http.end();
    }
}
