

struct CalibrationSettings {
  float accelBias[3];
  float gyroBias[3];
  float softIronOffset[3];
  float softIronScale[3];
  float hardIronOffset[3];
  float softIronMatrix[3][3];
};

extern bool readSensorDataFromSD(SdFs &sd, CalibrationSettings &sensorData);