// including the libraries needed for tflowlite module
#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <Wire.h> 
#include "model.h"
#include <Arduino_LSM9DS1.h>

// tflite error reporter and resolver
tflite::MicroErrorReporter tflErrorReporter;
tflite::AllOpsResolver tflOpsResolver;

// tfmodel intialization
// tflite input, output initialization
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// static memory creation
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// input variables declaration
float X,Y,Z;

// 5 posture labels 
const char* postureLabels[] = { "Supine","Prone", "Side", "Sit", "Unknown"};

// serial communication check
void setup() {
  Serial.begin(9600);
  while (!Serial);

  // initialize the IMU
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // sample rates of IMU data
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.print("Magnetometer sample rate = ");
  Serial.print(IMU.magneticFieldSampleRate());
  Serial.println(" Hz");

  tflModel = tflite::GetModel(tflite_model_tflite);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }

  // model interpreter 
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize); //&tflErrorReporter);

  // allocating interpreter for input/output sensors 
  tflInterpreter->AllocateTensors();

  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);
}


// main loop
void loop() {
  // base station input
  // 1-accelerometer
  // 2-gyroscope
  // 3-magnetometer
  if (Serial.available() > 0) {
    char input = Serial.read();
    switch (input) {
      case '1': // Read accelerometer data
        readAccelerometer();
        Inference();

        break;
      case '2': // Read gyroscope data
        readGyroscope();
        Inference();
        break;
      case '3': // Read magnetometer data
        readMagnetometer();
        Inference();
        break;
      default:
        // Serial.println("Invalid input");
        break;
    }
  }

  
}


// inference function definition
void Inference()
{
    // input data x, y, z 
    tflInputTensor->data.f[0] = X;
    tflInputTensor->data.f[1] = Y;
    tflInputTensor->data.f[2] = Z;
   
    // invoking the tfl interpreter
    int predictedClass = 0;
    TfLiteStatus invokeStatus = tflInterpreter->Invoke();
  
  // sending the output to tensor output
  float maxScore = tflOutputTensor->data.f[0];
  for (int i = 1; i < tflOutputTensor->dims->data[1]; i++) {
    if (tflOutputTensor->data.f[i] > maxScore) {
      maxScore = tflOutputTensor->data.f[i];
      predictedClass = i;
    }
  }
  // predicted posture - output 
  Serial.print("Predicted Posture: ");
  Serial.println(postureLabels[predictedClass]);
  // delay(1000);
}

// reading accelerometer data
void readAccelerometer() {
  float ax, ay, az;
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(ax, ay, az);
    Serial.print("Accelerometer: ");
    Serial.print("ax = "); Serial.print(ax); Serial.print('\t');
    Serial.print("ay = "); Serial.print(ay); Serial.print('\t');
    Serial.print("az = "); Serial.println(az);
    X=ax;
    Y=ay;
    Z=az;
  }
}

// definition to read gyroscope data
void readGyroscope() {
  float gx, gy, gz;
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(gx, gy, gz);
    Serial.print("Gyroscope: ");
    Serial.print("gx = "); Serial.print(gx); Serial.print('\t');
    Serial.print("gy = "); Serial.print(gy); Serial.print('\t');
    Serial.print("gz = "); Serial.println(gz);
    X=gx;
    Y=gy;
    Z=gz;
  }
}

// function to read magnetometer data
void readMagnetometer() {
  float mx, my, mz;
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx, my, mz);
    Serial.print("Magnetometer: ");
    Serial.print("mx = "); Serial.print(mx); Serial.print('\t');
    Serial.print("my = "); Serial.print(my); Serial.print('\t');
    Serial.print("mz = "); Serial.println(mz);
    X=mx;
    Y=my;
    Z=mz;
  }
}


