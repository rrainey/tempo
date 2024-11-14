#include <SdFat.h>
#include <ArduinoJson.h>

#include "Calibration.h"

bool readSensorDataFromSD(SdFs &sd, CalibrationSettings &sensorData) {
  FsFile file = sd.open("/tempo.json", FILE_READ);
  if (!file) {
    Serial.println("Failed to open file");
    return false;
  }

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, file);
  file.close();
  
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return false;
  }

  // Populate SensorData struct
  for (int i = 0; i < 3; i++) {
    sensorData.accelBias[i] = doc["accelBias"][i] | 0;
    sensorData.gyroBias[i] = doc["gyroBias"][i] | 0;
    sensorData.softIronOffset[i] = doc["softIronOffet"][i] | 0;
    sensorData.softIronScale[i] = doc["softIronScale"][i] | 1;
    sensorData.hardIronOffset[i] = doc["hardIronOffset"][i] | 0;

    for (int j = 0; j < 3; j++) {
      sensorData.softIronMatrix[i][j] = doc["softIronMatrix"][i][j] | (i == j ? 1 : 0);
    }
  }

  return true;
}
