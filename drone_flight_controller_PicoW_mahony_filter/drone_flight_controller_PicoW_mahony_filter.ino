//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5

#include <Wire.h>
#include "DFRobot_BMM150.h"

#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>

// DNS and Web Server Configuration
const byte DNS_PORT = 53;
IPAddress apIP(172, 217, 28, 1);
DNSServer dnsServer;
WebServer webServer(80);

//#define MPU9250_ADDRESS 0x68
#define MPU9250_ADDRESS 0x69
/* #define MAG_ADDRESS 0x0C */

#define GYRO_FULL_SCALE_250_DPS 0x00
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2_G 0x00
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18
#include <QMC5883LCompass.h>
#include <Servo.h>

QMC5883LCompass compass;
Servo ESC_FrontLeft;     // create servo object to control the ESC
Servo ESC_BackLeft;     // create servo object to control the ESC
Servo ESC_FrontRight;     // create servo object to control the ESC
Servo ESC_BackRight;     // create servo object to control the ESC
// Attach the ESC on pin 9   

#define I2C_ADDRESS_4 0x13

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);
#if defined(ESP32) || defined(ESP8266)
  #define BMM150_CS D3
#elif defined(__AVR__) || defined(ARDUINO_SAM_ZERO)
  #define BMM150_CS 3
#elif (defined NRF5)
  #define BMM150_CS 2  //The corresponding silkscreen on the development board is the pin P2
#endif




const int receiverPin = 2;  // Connect the signal wire to pin 2
unsigned long pulseWidth;



int pin9;
int pin10;
int motorSpeedFrontLeft;
int motorSpeedBackLeft;
int motorSpeedFrontRight;
int motorSpeedBackRight;
int motorSpeedFrontLeft_mapped;
int motorSpeedBackLeft_mapped;
int motorSpeedFrontRight_mapped;
int motorSpeedBackRight_mapped;

float saved_MPU_val[12];
float x_k[6] = {0, 0, 0, 0, 0, 0};
float x_k_prev[6] ={0, 0, 0, 0, 0, 0};

float gyro_pos[3] = {0, 0, 0};
float gyro_acc[3] = {0, 0, 0};

float gyro_angles[3] = {0, 0, 0}; 
float gyro_angles_saved[3] = {0, 0, 0}; 
int gyro_angles_saved_counter = 0;
float ar_saved = 0;

// Covariance matrix
float P_gyro[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

// Process noise covariance
float Q_gyro[3][3] = {
    {0.01, 0, 0},
    {0, 0.01, 0},
    {0, 0, 0.01}
};

// Measurement noise covariance
float R_gyro[3][3] = {
    {0.1, 0, 0},
    {0, 0.1, 0},
    {0, 0, 0.1}
};

// Identity matrix
float I_gyro[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
};

// State Covariance Matrix P
float P[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1},
};

// Process Noise Covariance Matrix Q
float Q[6][6] = {
  {0.01, 0, 0, 0, 0, 0},
  {0, 0.01, 0, 0, 0, 0},
  {0, 0, 0.01, 0, 0, 0},
  {0, 0, 0, 0.01, 0, 0},
  {0, 0, 0, 0, 0.01, 0},
  {0, 0, 0, 0, 0, 0.01},
};

// Measurement Noise Covariance Matrix R
float R[3][3] = {
  {0.1, 0, 0},
  {0, 0.1, 0},
  {0, 0, 0.1}
};

// Measurement Matrix H (for linear measurement model)
float H[3][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
};

// Identity Matrix I
float I[6][6] = {
  {1, 0, 0, 0, 0, 0},
  {0, 1, 0, 0, 0, 0},
  {0, 0, 1, 0, 0, 0},
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1}
};

void predict(float dt);
/* void update(float* z); */
void update();
void matrixMultiply66(float a[6][6], float b[6][6], float result[6][6]);
void matrixMultiply63(float a[6][6], float b[6][3], float result[6][3]);
void matrixMultiply36(float a[3][6], float b[6][6], float result[3][6]);
void matrixMultiply33(float a[6][3], float b[3][6], float result[3][3]);
void matrixAdd66(float a[6][6], float b[6][6], float result[6][6]);
void matrixAdd33(float a[3][3], float b[3][3], float result[3][3]);
void matrixSubtract66(float a[6][6], float b[6][6], float result[6][6]);
void matrixTranspose36(float a[3][6], float result[6][3]);
void matrixTranspose66(float a[6][6], float result[6][6]);
void matrixInverse33(float a[3][3], float result[3][3]);

const int initialSize = 5;  // Maximum size of the array
int maxSize = 4; // Example maximum size
int currentListSize = 0;

int* list = (int*)malloc(sizeof(int) * initialSize);

void appendToList(int element) {
  if (currentListSize < maxSize) {
    list[currentListSize] = element;
    currentListSize++;
  } else {
    maxSize *= 2;  // Double the size of the array
    list = (int*)realloc(list, sizeof(int) * maxSize);
    if (list != NULL) {
      list[currentListSize] = element;
      currentListSize++;
    } else {
      Serial.println("Memory allocation failed.");
    }
  }
}

// Function to print the list
void printList() {
  Serial.println("Current list:");
  for (int i = 0; i < currentListSize; i++) {
    Serial.println(list[i]);
    delay(50);
  }
}

//Funcion auxiliar lectura
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{

Wire.beginTransmission(Address);
Wire.write(Register);
Wire.endTransmission();

Wire.requestFrom(Address, Nbytes);
uint8_t index = 0;
while (Wire.available())
Data[index++] = Wire.read();
}

// Funcion auxiliar de escritura
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
Wire.beginTransmission(Address);
Wire.write(Register);
Wire.write(Data);
Wire.endTransmission();
}

#include "FastIMU.h"
//#define IMU_ADDRESS 0x68    //Change to the address of the IMU
#define IMU_ADDRESS 0x69    //Change to the address of the IMU
#define PERFORM_CALIBRATION //Comment to disable startup calibration
MPU6500 IMU;               //Change to the name of any supported IMU! 
calData calib = { 0 };  //Calibration data
void calibrate_IMU()
{
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    Serial.print("Error initializing IMU: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
#ifdef PERFORM_CALIBRATION
  if (Serial){
  Serial.println("FastIMU calibration & data example");
  }
  if (IMU.hasMagnetometer()) {
    delay(1000);
    if (Serial){
    Serial.println("Move IMU in figure 8 pattern until done.");
    }
    delay(3000);
    IMU.calibrateMag(&calib);
    if (Serial){
    Serial.println("Magnetic calibration done!");
    }
  }
  else {
    delay(5000);
  }

  delay(5000);
  if (Serial){
  Serial.println("Keep IMU level.");
  }
  delay(5000);
  IMU.calibrateAccelGyro(&calib);
  if (Serial){
  Serial.println("Calibration done!");
  Serial.println("Accel biases X/Y/Z: ");
  Serial.print(calib.accelBias[0]);
  Serial.print(", ");
  Serial.print(calib.accelBias[1]);
  Serial.print(", ");
  Serial.println(calib.accelBias[2]);
  Serial.println("Gyro biases X/Y/Z: ");
  Serial.print(calib.gyroBias[0]);
  Serial.print(", ");
  Serial.print(calib.gyroBias[1]);
  Serial.print(", ");
  Serial.println(calib.gyroBias[2]);
  }
/*   if (IMU.hasMagnetometer()) {
    Serial.println("Mag biases X/Y/Z: ");
    Serial.print(calib.magBias[0]);
    Serial.print(", ");
    Serial.print(calib.magBias[1]);
    Serial.print(", ");
    Serial.println(calib.magBias[2]);
    Serial.println("Mag Scale X/Y/Z: ");
    Serial.print(calib.magScale[0]);
    Serial.print(", ");
    Serial.print(calib.magScale[1]);
    Serial.print(", ");
    Serial.println(calib.magScale[2]);
  } */
  delay(5000);
  IMU.init(calib, IMU_ADDRESS);
#endif

  //err = IMU.setGyroRange(500);      //USE THESE TO SET THE RANGE, IF AN INVALID RANGE IS SET IT WILL RETURN -1
  //err = IMU.setAccelRange(2);       //THESE TWO SET THE GYRO RANGE TO ±500 DPS AND THE ACCELEROMETER RANGE TO ±2g
  
  if (err != 0) {
    Serial.print("Error Setting range: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
}

int calibrate_counter = 0;
float ar_cal = 0;
AccelData accelData;    //Sensor data
void calibrate_ar(float ar_setup)
{
  if (ar_setup > 9 && ar_setup < 11){
    ar_cal += ar_setup;
  }
  
}
volatile unsigned long pulseStart = 0;
volatile unsigned long pulseStart_end = 0;
volatile unsigned long pulseEnd = 0;
volatile unsigned long pulseWidth_interrupt = 0;
volatile unsigned long lastInterruptTime = 0; // To debounce the signal
volatile unsigned long pulseStartTime = 0;

unsigned long minPulseWidth = 1000; // Minimum pulse width (in microseconds)
unsigned long maxPulseWidth = 2000; // Maximum pulse width (in microseconds)
// Interrupt Service Routine for the rising edge
void pulseStartISR() {
   unsigned long currentTime = micros();
    if (currentTime - lastInterruptTime > 25000) {
    lastInterruptTime = currentTime;
    pulseStart = micros();  // Record time when pulse starts
    attachInterrupt(digitalPinToInterrupt(receiverPin), pulseEndISR, FALLING);
    pulseStart_end = micros();
    Serial.print("start_dif "); Serial.print(pulseStart_end-pulseStart); 
  }

}

// Interrupt Service Routine for the falling edge
void pulseEndISR() {
  pulseEnd = micros();  // Record time when pulse ends
  pulseWidth_interrupt = pulseEnd - pulseStart_end;  // Calculate pulse width
  attachInterrupt(digitalPinToInterrupt(receiverPin), pulseStartISR, RISING);  // Re-enable interrupt for next pulse
  float pulseend_end = micros();
 Serial.print("pulseend_dif "); Serial.print(pulseend_end-pulseEnd); 
}



float rollMag_cal;
float pitchMag_cal;
float yawMag_cal;
float mx_cal;
float my_cal;
float mz_cal;
void calibrate_mag(float mx, float my, float mz){
/*   rollMag_cal += atan2(my, mz);
  pitchMag_cal += atan2(mx, my);
  yawMag_cal += atan2(mx, mz); // Approximation for yaw from accelerometer */
  mx_cal += mx;
  my_cal += my;
  mz_cal += mz;
}

String mockSensorValue = "ok"; // Replace this with actual sensor reading logic

#define AK8963_ADDR  0x0C  // Internal magnetometer I2C address
#define AK8963_CNTL1 0x0A  // Control register for AK8963
void disableAK8963() {
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(AK8963_CNTL1);
    Wire.write(0x00);  // Power-down mode
    Wire.endTransmission();
}


void setup()
{
Serial.begin(115200);

pinMode(9, OUTPUT);     
pinMode(10, OUTPUT);    
pinMode(11, OUTPUT);    
pinMode(12, OUTPUT);    

ESC_FrontLeft.attach(12,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
ESC_BackLeft.attach(9,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
ESC_FrontRight.attach(11,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 
ESC_BackRight.attach(10,1000,2000); // (pin, min pulse width, max pulse width in microseconds) 

ESC_FrontLeft.write(0);  // Initialize ESC with the lowest throttle position
ESC_BackLeft.write(0);  // Initialize ESC with the lowest throttle position
ESC_FrontRight.write(0);  // Initialize ESC with the lowest throttle position
ESC_BackRight.write(0);  // Initialize ESC with the lowest throttle position
/* ESC_FrontLeft.writeMicroSeconds(1000);  // Initialize ESC with the lowest throttle position
ESC_FrontLeft.writeMicroSeconds(1000);  // Initialize ESC with the lowest throttle position
ESC_FrontLeft.writeMicroSeconds(1000);  // Initialize ESC with the lowest throttle position
ESC_FrontLeft.writeMicroSeconds(1000);  // Initialize ESC with the lowest throttle position */
delay(2000);             // Allow ESC to arm

//LED_PIN

pinMode(1, OUTPUT); 
digitalWrite(1, HIGH);



/* attachInterrupt(digitalPinToInterrupt(receiverPin), pulseStartISR, RISING); */


 // Setup Wi-Fi Access Point
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apIP, IPAddress(255, 255, 255, 0));
  WiFi.softAP("picow", "12345678");

  // DNS Server Configuration
  dnsServer.setTTL(300);
  dnsServer.setErrorReplyCode(DNSReplyCode::ServerFailure);
  dnsServer.start(DNS_PORT, "www.example.com", apIP);

  // Setup Web Server
  webServer.on("/", []() {
    String html = "<!DOCTYPE html><html>";
    html += "<head><title>Sensor Values</title></head>";
    html += "<body>";
    html += "<h1>Sensor Data</h1>";
    html += "<p>Current Sensor Value: ";
    html += mockSensorValue; // Display the current sensor value
    html += " °C</p>";
    html += "</body></html>";

    webServer.send(200, "text/html", html);
  });

  webServer.begin();

/*   disableAK8963();  // Disable internal magnetometer
  Serial.println("AK8963 disabled."); */

  // Initialize BMM150 sensor
  while (bmm150.begin()) {
    if (Serial) {
      Serial.println("BMM150 init failed, please try again!");
    }
    delay(1000);
  }
  if (Serial) {
    Serial.println("BMM150 init success!");
  }
bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
bmm150.setRate(BMM150_DATA_RATE_10HZ);
bmm150.setMeasurementXYZ();
// Initialize I2C bus
Wire.begin();

calibrate_IMU();
//sBmm150MagData_t magData = bmm150.getGeomagneticData();
for (int i = 0; i < 10; i++) {
IMU.update();
IMU.getAccel(&accelData);
float ax_setup = accelData.accelX*9.82;
float ay_setup = accelData.accelY*9.82;
float az_setup = accelData.accelZ*9.82;

float ar_setup = sqrt(ax_setup*ax_setup + ay_setup*ay_setup + az_setup*az_setup);
 if (Serial) {
Serial.print(ax_setup);
Serial.print(ay_setup);
Serial.print(az_setup);
 }
calibrate_ar(ar_setup);

/* float mx_setup = magData.x;
float my_setup = -magData.y;
float mz_setup = magData.z;
calibrate_mag(mx_setup, my_setup, mz_setup); */

delay(100);
}

ar_cal /= 10;


if (9.7 < ar_cal && ar_cal < 10.1){
   if (Serial) {
Serial.print("\n");
Serial.print(ar_cal);
Serial.print("\t");
Serial.print("ar_cal done!");
Serial.print("\n");
   }
delay(2000);
}
else{
  while(1){
     if (Serial) {
    Serial.println("ar calibration gave value other than 9.7 to 10.3, Please try again!");
     }
    delay(1000);
  }
}

//Transmitter pin
pinMode(receiverPin, INPUT);

/* compass.init(); */


// Configurar acelerometro
I2CwriteByte(MPU9250_ADDRESS, 28, ACC_FULL_SCALE_16_G);
// Configurar giroscopio
I2CwriteByte(MPU9250_ADDRESS, 27, GYRO_FULL_SCALE_2000_DPS);
// Configurar magnetometro
/* I2CwriteByte(MPU9250_ADDRESS, 0x37, 0x02);
I2CwriteByte(MAG_ADDRESS, 0x0A, 0x01); */

}

void predict(float dt, float ax, float ay, float az, float gx, float gy, float gz ) {
x_k_prev[3] += ax*dt; //v_final=a_mean*t v_mean=v_final/2
x_k_prev[4] += ay*dt; //ksk ta bort /2 pga rätt strecka men fel hastighet
x_k_prev[5] += az*dt;


gyro_pos[0] += gx*dt;
gyro_pos[1] += gy*dt;
gyro_pos[2] += gz*dt;

int comp_for_g = 9.82;

/* air_dens = 1.225;
Fdrag_x = 0.5*air_dens*x_k_prev[3]^2*0.8*0.04;
Fdrag_y = 0.5*air_dens*x_k_prev[4]^2*0.8*0.04;
Fdrag_z = 0.5*air_dens*x_k_prev[5]^2*0.8*0.04;
 = 9.82;

/* air_dens = 1.225;
Fdrag_x = 0.5*air_dens*x_k_prev[3]^2*0.8*0.04;
Fdrag_y = 0.5*air_dens*x_k_prev[4]^2*0.8*0.04;
Fdrag_z = 0.5*air_dens*x_k_prev[5]^2*0.8*0.04;
x_k_prev[3] += ax*dt/2*10/2048; 
x_k_prev[4] += ay*dt/2*10/2048;
x_k_prev[5] += az*dt/2*10/2048; */

//Orientation


float F[6][6] = {
  {1, 0, 0, dt/2, 0, 0},
  {0, 1, 0, 0, dt/2, 0},
  {0, 0, 1, 0, 0, dt/2}, //bra ned dt/2 ??
  {0, 0, 0, 1, 0, 0},
  {0, 0, 0, 0, 1, 0},
  {0, 0, 0, 0, 0, 1},
};

/* x_k[0] = F*x_k_prev + w_k ; */
  for (int i = 0; i < 6; i++) {
    x_k[i] = 0;
    for (int j = 0; j < 6; j++) {
      x_k[i] += F[i][j] * x_k_prev[j];
    }
  }

/* Serial.print("F[0][3]*x_k_prev[3]: ");
Serial.print(F[0][3]*x_k_prev[3]);
Serial.print("\n");

Serial.print("F[0][0]*x_k_prev[0]: ");
Serial.print(F[0][0]*x_k_prev[0]);
Serial.print("\n");

Serial.print("X_K1: ");
Serial.print(x_k[0]);
Serial.print("\n");

Serial.print("F[3][3]*x_k_prev[3] ");
Serial.print(F[3][3]*x_k_prev[3] );
Serial.print("\n");

Serial.print("X_K3: ");
Serial.print(x_k[3]);
Serial.print("\n"); */

for (int i = 0; i < 6; i++) {
    x_k_prev[i] = x_k[i];
}


// Predict the state covariance: P(6x6) = F(6x6) * P(6x6) * F' + Q
float F_transpose[6][6];
matrixTranspose66(F, F_transpose);

float FP[6][6];
matrixMultiply66(F, P, FP);

float FPFT[6][6];
matrixMultiply66(FP, F_transpose, FPFT);

matrixAdd66(FPFT, Q, P);

}

// Helper functions for matrix operations

void matrixMultiply66(float a[6][6], float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 6; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixMultiply63(float a[6][6], float b[6][3], float result[6][3]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 6; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixMultiply63_33(float a[6][3], float b[3][3], float result[6][3]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixMultiply63_66(float a[6][3], float b[3][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 3; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixMultiply36(float a[3][6], float b[6][6], float result[3][6]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 6; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixMultiply33(float a[3][6], float b[6][3], float result[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = 0;
      for (int k = 0; k < 6; k++) {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }
}

void matrixAdd66(float a[6][6], float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = a[i][j] + b[i][j];
    }
  }
}

void matrixAdd33(float a[3][3], float b[3][3], float result[3][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      result[i][j] = a[i][j] + b[i][j];
    }
  }
}

void matrixSubtract66(float a[6][6], float b[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[i][j] = a[i][j] - b[i][j];
    }
  }
}

void matrixTranspose66(float a[6][6], float result[6][6]) {
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      result[j][i] = a[i][j];
    }
  }
}

void matrixTranspose36(float a[3][6], float result[6][3]) {
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 6; j++) {
      result[j][i] = a[i][j];
    }
  }
}


void matrixInverse33(float a[3][3], float result[3][3]) {
  // Calculate the determinant
  float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
              a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
              a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

/*   Serial.print("det: ");
  Serial.print(det); */
  if (det == 0) return; // Non-invertible

  float invDet = 1.0 / det;

  result[0][0] = invDet * (a[1][1] * a[2][2] - a[2][1] * a[1][2]);
  result[0][1] = invDet * (a[0][2] * a[2][1] - a[0][1] * a[2][2]);
  result[0][2] = invDet * (a[0][1] * a[1][2] - a[0][2] * a[1][1]);

  result[1][0] = invDet * (a[1][2] * a[2][0] - a[1][0] * a[2][2]);
  result[1][1] = invDet * (a[0][0] * a[2][2] - a[0][2] * a[2][0]);
  result[1][2] = invDet * (a[1][0] * a[0][2] - a[0][0] * a[1][2]);

  result[2][0] = invDet * (a[1][0] * a[2][1] - a[2][0] * a[1][1]);
  result[2][1] = invDet * (a[2][0] * a[0][1] - a[0][0] * a[2][1]);
  result[2][2] = invDet * (a[0][0] * a[1][1] - a[1][0] * a[0][1]);
}

void update() {
  // Innovation (or Residual) y = z - H * x
  float y[3];
  for (int i = 0; i < 3; i++) {
    y[i] = x_k_prev[i];
    for (int j = 0; j < 6; j++) {
      y[i] -= H[i][j] * x_k_prev[j];
    }
  }

  // S = H * P * H' + R
  float H_transpose[6][3];
  matrixTranspose36(H, H_transpose);

  float HP[3][6];
  matrixMultiply36(H, P, HP);

  float HPH_transpose[3][3];
  matrixMultiply33(HP, H_transpose, HPH_transpose);

  float S[3][3];
  matrixAdd33(HPH_transpose, R, S);

  // Kalman Gain K = P * H' * S^(-1)
  float S_inverse[3][3];
  matrixInverse33(S, S_inverse);

  float PH_transpose[6][3];
  matrixMultiply63(P, H_transpose, PH_transpose);

  float K[6][3];
  matrixMultiply63_33(PH_transpose, S_inverse, K);

  // Update the state: x = x + K * y
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 3; j++) {
      x_k_prev[i] += K[i][j] * y[j];
    }
  }

  // Update the covariance: P = (I - K * H) * P
  float KH[6][6];
  matrixMultiply63_66(K, H, KH);

  float I_KH[6][6];
  matrixSubtract66(I, KH, I_KH);

/*   for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {

      Serial.print("S: ");
      Serial.print(S[i][j]);
      Serial.print(" HPH_transpose: ");
      Serial.print(HPH_transpose[i][j]);
      Serial.print(" P: ");
      Serial.print(P[i][j]);
      Serial.print(" H: ");
      Serial.print(H[i][j]);
      Serial.print(" I: ");
      Serial.print(I[i][j]);
      Serial.print(" K: ");
      Serial.print(K[i][j]);
      Serial.print("\n"); 
    }
  }
 */

  float newP[6][6];
  matrixMultiply66(I_KH, P, newP);
  for (int i = 0; i < 6; i++) {
 /*    y[i] = z[i]; */
    for (int j = 0; j < 6; j++) {

/*       Serial.print(" newP: ");
      Serial.print(P[i][j]); */

    }
  }



  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) {
      P[i][j] = newP[i][j];
    }
  }
}

// Function to normalize a vector
void normalize(float vec[3]) {
  float magnitude = sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
  vec[0] /= magnitude;
  vec[1] /= magnitude;
  vec[2] /= magnitude;
}

// Rodrigues' rotation formula
void rotateVector(float vector[3], float axis[3], float theta, float result[3]) {
  // Normalize the axis vector
  normalize(axis);

  // Precompute sin and cos
  float cosTheta = cos(theta);
  float sinTheta = sin(theta);

  // Cross-product matrix of the axis vector
  float K[3][3] = {
    {0, -axis[2], axis[1]},
    {axis[2], 0, -axis[0]},
    {-axis[1], axis[0], 0}
  };

  // Calculate K^2 (cross-product matrix squared)
  float K2[3][3] = {
    {K[0][0] * K[0][0] + K[0][1] * K[1][0] + K[0][2] * K[2][0],
     K[0][0] * K[0][1] + K[0][1] * K[1][1] + K[0][2] * K[2][1],
     K[0][0] * K[0][2] + K[0][1] * K[1][2] + K[0][2] * K[2][2]},

    {K[1][0] * K[0][0] + K[1][1] * K[1][0] + K[1][2] * K[2][0],
     K[1][0] * K[0][1] + K[1][1] * K[1][1] + K[1][2] * K[2][1],
     K[1][0] * K[0][2] + K[1][1] * K[1][2] + K[1][2] * K[2][2]},

    {K[2][0] * K[0][0] + K[2][1] * K[1][0] + K[2][2] * K[2][0],
     K[2][0] * K[0][1] + K[2][1] * K[1][1] + K[2][2] * K[2][1],
     K[2][0] * K[0][2] + K[2][1] * K[1][2] + K[2][2] * K[2][2]}
  };

  // Rotation matrix components
  float I[3][3] = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
  };

  float R[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      R[i][j] = I[i][j] + sinTheta * K[i][j] + (1 - cosTheta) * K2[i][j];
    }
  }

  // Rotate the vector
  for (int i = 0; i < 3; i++) {
    result[i] = 0;
    for (int j = 0; j < 3; j++) {
      result[i] += R[i][j] * vector[j];
    }
  }
}

void make_rotation(float vector[3], float axis[3], float theta) {


/*   // Example vector and axis
  float vector[3] = {0, 0, 1};
  float axis[3] = {1, 0, 0};  // Arbitrary axis (you can change this) */
/*   float theta = radians(45);  // Rotation angle in radians */

  // Resultant rotated vector
  float result[3];

  // Perform rotation
  rotateVector(vector, axis, theta, result);

  // Output the results
  Serial.print("Original Vector: [");
  Serial.print(vector[0]);
  Serial.print(", ");
  Serial.print(vector[1]);
  Serial.print(", ");
  Serial.print(vector[2]);
  Serial.println("]");

  Serial.print("Rotated Vector: [");
  Serial.print(result[0]);
  Serial.print(", ");
  Serial.print(result[1]);
  Serial.print(", ");
  Serial.print(result[2]);
  Serial.println("]");
}

void Print_base_parameters(float ax,float ay, float az, float gx, float gy, float gz, float gyro_angles[], float dt, float motorSpeedFrontLeft, float motorSpeedFrontRight, float motorSpeedBackLeft, float motorSpeedBackRight, float roll_mahogny, float pitch_mahogny, float yaw_mahogny){
/*   Serial.print("ax:"); Serial.print(ax); Serial.print(" ");
  Serial.print("ay:"); Serial.print(ay); Serial.print(" ");
  Serial.print("az:"); Serial.print(az); Serial.print(" ");

  Serial.print("dt:"); Serial.print(dt); Serial.print(" ");
  
  Serial.print("gx:"); Serial.print(gx); Serial.print(" ");
  Serial.print("gy:"); Serial.print(gy); Serial.print(" ");
  Serial.print("gz:"); Serial.print(gz); Serial.print(" "); */


/*   Serial.print("w:"); Serial.print(gyro_angles[0]); Serial.print(" ");
  Serial.print("v:"); Serial.print(gyro_angles[1]); Serial.print(" ");
  Serial.print("u:"); Serial.print(gyro_angles[2]); Serial.print(" "); */
  Serial.print("roll_mahogny:"); Serial.print(roll_mahogny); Serial.print(" ");
  Serial.print("pitch_mahogny:"); Serial.print(pitch_mahogny); Serial.print(" ");
  Serial.print("yaw_mahogny:"); Serial.print(yaw_mahogny); Serial.print(" ");

  Serial.print("motorSpeedFrontLeft:"); Serial.print(motorSpeedFrontLeft); Serial.print(" ");
  Serial.print("motorSpeedFrontRight:"); Serial.print(motorSpeedFrontRight); Serial.print(" ");
  Serial.print("motorSpeedBackLeft:"); Serial.print(motorSpeedBackLeft); Serial.print(" ");
  Serial.print("motorSpeedBackRight:"); Serial.print(motorSpeedBackRight); Serial.print(" ");
  Serial.print("\n");
}

// PID parameters
float Kp_roll = 1.0; // Proportional constant for roll
float Ki_roll = 0.0; // Integral constant for roll
float Kd_roll = 0.1; // Derivative constant for roll

float Kp_pitch = 1.0; // Proportional constant for pitch
float Ki_pitch = 0.0; // Integral constant for pitch
float Kd_pitch = 0.1; // Derivative constant for pitch

// Roll and pitch target angles (degrees)
float target_roll = 0.0; // Target roll angle (0 degrees for level)
float target_pitch = 0.0; // Target pitch angle (0 degrees for level)

// Error and previous error variables
float error_roll = 0.0;
float error_pitch = 0.0;
float previous_error_roll = 0.0;
float previous_error_pitch = 0.0;

// Integral terms
float integral_roll = 0.0;
float integral_pitch = 0.0;

// Update PID control at each loop
void updatePID(float roll_mahogny, float pitch_mahogny) {
  // Calculate the PID control for roll
  error_roll = target_roll - roll_mahogny;
  integral_roll += error_roll; // Accumulate the error
  float derivative_roll = error_roll - previous_error_roll;

  // Compute the PID for roll
  float pid_roll = Kp_roll * error_roll + Ki_roll * integral_roll + Kd_roll * derivative_roll;

  // Update previous error for next iteration
  previous_error_roll = error_roll;

  // Calculate the PID control for pitch
  error_pitch = target_pitch - pitch_mahogny;
  integral_pitch += error_pitch; // Accumulate the error
  float derivative_pitch = error_pitch - previous_error_pitch;

  // Compute the PID for pitch
  float pid_pitch = Kp_pitch * error_pitch + Ki_pitch * integral_pitch + Kd_pitch * derivative_pitch;

  // Update previous error for next iteration
  previous_error_pitch = error_pitch;
}




int sign(float x) {
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 1;
}

int motor_on = 0;
float mpc_scaler;

float send_data_to_wifi = 0;
int input_motor_speed = 0;
int input_motor_speed_changed = 0;
int input_motor_speed_changed_counter = 0;
int input_motor_speed_changed_bool = 0;
int calibrate = 0;
int thrust_scaler = 20;
float motor_thrust_dif = 0;
/* float addToMotorSpeedFrontLeft = 0;
float addToMotorSpeedBackLeft = 0;
float addToMotorSpeedFrontRight = 0;
float addToMotorSpeedBackRight = 0; */
//void changeMotorSpeed(float gX, float gY, float gZ, float gyro_angles[], float ar, float mx, float my, float mz, int input_motor_speed_changed) {
void changeMotorSpeed(float gX, float gY, float gZ, float gyro_angles[], float ar, float mx, float my, float mz, int input_motor_speed_changed, float ax, float ay, float az, float gx, float gy, float gz, float dt, float roll_mahogny, float pitch_mahogny, float yaw_mahogny ) {

/*   if (calibrate == 0){
  motorSpeedFrontLeft = 1023;
  motorSpeedBackLeft = 1023;
  motorSpeedFrontRight = 1023;
  motorSpeedBackRight = 1023;
  calibrate = 1;
  motor_on = 1;
  } */

  //safety turn-off
/*   if (input_motor_speed_changed == input_motor_speed){
    input_motor_speed_changed_counter += 1;
    
    if (input_motor_speed_changed_counter == 10000){
    input_motor_speed_changed = 0;
    input_motor_speed_changed_counter = 0;
    }
  }
  else{
    input_motor_speed_changed_counter = 0;
  } */




  if (input_motor_speed_changed < 15){
    input_motor_speed_changed = 0;
    motor_on = 0;
  }
  if (input_motor_speed_changed > 1000){
    input_motor_speed_changed = 1000;
  }

  if (abs(input_motor_speed_changed - input_motor_speed) > 8){

    input_motor_speed_changed_bool = 1;

    motor_thrust_dif = input_motor_speed_changed - input_motor_speed;
    input_motor_speed = input_motor_speed_changed;

    motorSpeedFrontLeft += motor_thrust_dif;
    motorSpeedBackLeft += motor_thrust_dif;
    motorSpeedFrontRight += motor_thrust_dif;
    motorSpeedBackRight += motor_thrust_dif;

    if (input_motor_speed > 0 && input_motor_speed <= 2000) {
      // Set the motor speed

      // Print the value to the serial monitor
/*       Serial.print("\n");
      Serial.print("Motor speed increased by: ");
      Serial.print(motor_thrust_dif); */
      motor_on = 1;
    } else {
      // Print an error message if the value is out of range
     // Serial.println("A lever-value that isn't between 0 and 2000 turns of device.");
      motor_on = 0;
    }
  }
  else{
    input_motor_speed_changed_bool = 0;
  }

/*   if (Serial.available() > 0 ) {
    // Read the incoming byte:
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any extra whitespace

    input_motor_speed = input.toInt();
    // Convert the string to an integer
    //input.toInt()


    motorSpeedFrontLeft = input_motor_speed;
    motorSpeedBackLeft = input_motor_speed;
    motorSpeedFrontRight = input_motor_speed;
    motorSpeedBackRight = input_motor_speed;
    
    // Check if the value is within the valid range
    if (input_motor_speed > 0 && input_motor_speed <= 2000) {
      // Set the motor speed

      // Print the value to the serial monitor
      Serial.print("\n");
      Serial.print("Motor speed set to: ");
      Serial.print(input_motor_speed);
      motor_on = 1;
    } else {
      // Print an error message if the value is out of range
      Serial.println("A value that isn't between 0 and 2000 turns of device.");
      motor_on = 0;
    }
  } */

/*   if (calibrate_counter < 20 && ar > 9 && ar < 11){
    calibrate_counter += 1;
    ar_cal += ar;
  }
  else if (calibrate_counter == 10){
    calibrate_counter += 1;
    ar_cal /= 20;
  } */

/*   if (gyro_angles[1] > 0) {
/*   motorSpeedFrontLeft += 3;
  motorSpeedFrontRight += 3; */
 // }   */
/*   if (ar > 11){
  //  Rising_drone()

  }

  if (abs(ar < 9)){
  //  Falling_drone();

  } */
     
/*   addToMotorSpeedFrontLeft += 20*gyro_angles[0];
  addToMotorSpeedFrontRight += 20*gyro_angles[0];
  addToMotorSpeedBackLeft -= 20*gyro_angles[0];
  addToMotorSpeedBackRight -= 20*gyro_angles[0]; */





// first autopilot--------

/*   mpc_scaler = 1.570796/(abs(gyro_angles[2])+0.001);
// tilting forward-backward  gyro 0

  motorSpeedFrontLeft += mpc_scaler*40*gyro_angles[0]*40*abs(gyro_angles[0]);
  motorSpeedFrontRight += mpc_scaler*40*gyro_angles[0]*40*abs(gyro_angles[0]);
  motorSpeedBackLeft -= mpc_scaler*40*gyro_angles[0]*40*abs(gyro_angles[0]);
  motorSpeedBackRight -= mpc_scaler*40*gyro_angles[0]*40*abs(gyro_angles[0]);

// tilting left-right gyro 1
  motorSpeedFrontRight += mpc_scaler*40*gyro_angles[1]*40*abs(gyro_angles[1]);
  motorSpeedBackRight += mpc_scaler*40*gyro_angles[1]*40*abs(gyro_angles[1]);
  motorSpeedFrontLeft -= mpc_scaler*40*gyro_angles[1]*40*abs(gyro_angles[1]);
  motorSpeedBackLeft -= mpc_scaler*40*gyro_angles[1]*40*abs(gyro_angles[1]); */
// first autopilot end--------

// second autopilot--------
// second autopilot--------
/* if (gyro_angles[2] < 1.1){
  motor_on = 0;

}
else if (gyro_angles[2] < 1.35){
  thrust_scaler = 50;

}
else {
  thrust_scaler = 20;

} */

if (abs(roll_mahogny) > 0.15 || abs(pitch_mahogny) > 0.15){
  motor_on = 0;
  send_data_to_wifi = 1;
/*   Serial.print("Turn off: ");
Print_base_parameters(ax, ay, az, gx, gy, gz, gyro_angles, dt, motorSpeedFrontLeft, motorSpeedFrontRight, motorSpeedBackLeft, motorSpeedBackRight);
Serial.print(", rollAcc_mahogny ");
Serial.print(roll_mahogny, DEC);
Serial.print(", pitchAcc_mahogny ");
Serial.print(pitch_mahogny, DEC);
Serial.print(", yawAcc_mahogny ");
Serial.print(yaw_mahogny, DEC) ;
Serial.print("\n");
Serial.print("----------");
Serial.print("\n");  */
}
else if (abs(pitch_mahogny) > 0.08){
  thrust_scaler = 9;

}
else {
  thrust_scaler = 5;

} 

//tilt forward

if (roll_mahogny > 0.04){
motorSpeedFrontLeft += thrust_scaler;
motorSpeedFrontRight += thrust_scaler;
motorSpeedBackLeft -= thrust_scaler;
motorSpeedBackRight -= thrust_scaler;
}
//tilt backward
else if (roll_mahogny < -0.04){
motorSpeedFrontLeft -= thrust_scaler;
motorSpeedFrontRight -= thrust_scaler;
motorSpeedBackLeft += thrust_scaler;
motorSpeedBackRight += thrust_scaler;
}
/* else if (abs(roll_mahogny) < 0.04){
motorSpeedFrontLeft += 60*gyro_angles[0];
motorSpeedFrontRight += 60*gyro_angles[0];
motorSpeedBackLeft -= 60*gyro_angles[0];
motorSpeedBackRight -= 60*gyro_angles[0]; 
} */
//tilt right
if (pitch_mahogny > 0.04){
  motorSpeedFrontRight -= thrust_scaler;
  motorSpeedBackRight -= thrust_scaler;
  motorSpeedFrontLeft += thrust_scaler;
  motorSpeedBackLeft += thrust_scaler; 
}
//tilt left
else if (pitch_mahogny < -0.04){
  motorSpeedFrontRight += thrust_scaler;
  motorSpeedBackRight += thrust_scaler;
  motorSpeedFrontLeft -= thrust_scaler;
  motorSpeedBackLeft -= thrust_scaler; 
}

/* else if (abs(pitch_mahogny) < 0.04){
  motorSpeedFrontRight -= 60*gyro_angles[2];
  motorSpeedBackRight -= 60*gyro_angles[2];
  motorSpeedFrontLeft += 60*gyro_angles[2];
  motorSpeedBackLeft += 60*gyro_angles[2];  
} */

/* //tilt forward
if (gyro_angles[0] > 0.04){
motorSpeedFrontLeft += thrust_scaler;
motorSpeedFrontRight += thrust_scaler;
motorSpeedBackLeft -= thrust_scaler;
motorSpeedBackRight -= thrust_scaler;
}
//tilt backward
else if (gyro_angles[0] < -0.04){
motorSpeedFrontLeft -= thrust_scaler;
motorSpeedFrontRight -= thrust_scaler;
motorSpeedBackLeft += thrust_scaler;
motorSpeedBackRight += thrust_scaler;
}
else if (abs(gyro_angles[0]) < 0.04){
motorSpeedFrontLeft += 60*gyro_angles[0];
motorSpeedFrontRight += 60*gyro_angles[0];
motorSpeedBackLeft -= 60*gyro_angles[0];
motorSpeedBackRight -= 60*gyro_angles[0]; 
}
//tilt right
if (gyro_angles[2] > 0.04){
  motorSpeedFrontRight += thrust_scaler;
  motorSpeedBackRight += thrust_scaler;
  motorSpeedFrontLeft -= thrust_scaler;
  motorSpeedBackLeft -= thrust_scaler; 
}
//tilt left
else if (gyro_angles[2] < -0.04){
  motorSpeedFrontRight -= thrust_scaler;
  motorSpeedBackRight -= thrust_scaler;
  motorSpeedFrontLeft += thrust_scaler;
  motorSpeedBackLeft += thrust_scaler; 
}

else if (abs(gyro_angles[2]) < 0.04){
  motorSpeedFrontRight += 60*gyro_angles[2];
  motorSpeedBackRight += 60*gyro_angles[2];
  motorSpeedFrontLeft -= 60*gyro_angles[2];
  motorSpeedBackLeft -= 60*gyro_angles[2];  
} */

//tilt left
/* else if (gyro_angles[1] < -0.06){
  motorSpeedFrontRight -= thrust_scaler;
  motorSpeedBackRight -= thrust_scaler;
  motorSpeedFrontLeft += thrust_scaler;
  motorSpeedBackLeft += thrust_scaler; 
} */

/* else if (abs(gyro_angles[1]) < 0.06){
  motorSpeedFrontRight += 60*gyro_angles[1];
  motorSpeedBackRight += 60*gyro_angles[1];
  motorSpeedFrontLeft -= 60*gyro_angles[1];
  motorSpeedBackLeft -= 60*gyro_angles[1];  
} */

// second autopilot end--------


//Extra weight on gyro 2 change
/*   int u_rephased_sign = sign(gyro_angles[1]); */
/*   float u_rephased_sign = 100*gyro_angles[1]*gyro_angles[1];
  float u_rephased = (gyro_angles[2] - 3.141592/2)*u_rephased_sign;

  motorSpeedFrontRight -= 30*u_rephased;
  motorSpeedBackRight -= 30*u_rephased;
  motorSpeedFrontLeft += 30*u_rephased;
  motorSpeedBackLeft += 30*u_rephased; */



/*   if (calibrate_counter == 11){
  motorSpeedFrontRight += 20*(ar_cal-ar);
  motorSpeedBackRight += 20*(ar_cal-ar);
  motorSpeedFrontLeft += 20*(ar_cal-ar);
  motorSpeedBackLeft += 20*(ar_cal-ar);
  } */

/*   motorSpeedFrontRight += 30*(ar_cal-ar);
  motorSpeedBackRight += 30*(ar_cal-ar);
  motorSpeedFrontLeft += 30*(ar_cal-ar);
  motorSpeedBackLeft += 30*(ar_cal-ar); */



if (motor_on == 1){
//logic
if (abs(motorSpeedFrontRight - input_motor_speed) > 50) {
  motorSpeedFrontRight = input_motor_speed + 50*sign(motorSpeedFrontRight - input_motor_speed);
}
if (abs(motorSpeedFrontLeft - input_motor_speed) > 50) {
  motorSpeedFrontLeft = input_motor_speed + 50*sign(motorSpeedFrontLeft - input_motor_speed);
}
if (abs(motorSpeedBackRight - input_motor_speed) > 50) {
  motorSpeedBackRight = input_motor_speed + 50*sign(motorSpeedBackRight - input_motor_speed);
}
if (abs(motorSpeedBackLeft - input_motor_speed) > 50) {
  motorSpeedBackLeft = input_motor_speed + 50*sign(motorSpeedBackLeft - input_motor_speed);
}

//steady state 
/*   if (input_motor_speed + motorSpeedFrontRight > 0){
    motorSpeedFrontRight += 40*(input_motor_speed - motorSpeedFrontRight)/(input_motor_speed + motorSpeedFrontRight);
  }
  if (input_motor_speed + motorSpeedFrontLeft > 0){
    motorSpeedFrontLeft += 40*(input_motor_speed - motorSpeedFrontLeft)/(input_motor_speed + motorSpeedFrontLeft);
  }
  if (input_motor_speed + motorSpeedBackRight > 0){
    motorSpeedBackRight += 40*(input_motor_speed - motorSpeedBackRight)/(input_motor_speed + motorSpeedBackRight);
  }
  if (input_motor_speed + motorSpeedBackLeft > 0){
    motorSpeedBackLeft += 40*(input_motor_speed - motorSpeedBackLeft)/(input_motor_speed + motorSpeedBackLeft);
  } */

/*   if (abs(motorSpeedFrontRight - input_motor_speed) > 150){
    motorSpeedFrontRight = input_motor_speed + 150*sign(motorSpeedFrontRight - input_motor_speed);
  }
  if (abs(motorSpeedFrontLeft - input_motor_speed) > 150){
    motorSpeedFrontLeft = input_motor_speed + 150*sign(motorSpeedFrontLeft - input_motor_speed);
  }
  if (abs(motorSpeedBackRight - input_motor_speed) > 150){
    motorSpeedBackRight = input_motor_speed + 150*sign(motorSpeedBackRight - input_motor_speed);
  }
  if (abs(motorSpeedBackLeft - input_motor_speed) > 150){
    motorSpeedBackLeft = input_motor_speed + 150*sign(motorSpeedBackLeft - input_motor_speed);
  } */

  if (motorSpeedFrontLeft > 800){
    motorSpeedFrontLeft = 800;
  }
  if (motorSpeedFrontLeft < 100){
    motorSpeedFrontLeft = 100;
  }
  if (motorSpeedFrontRight > 800){
    motorSpeedFrontRight = 800;
  }
  if (motorSpeedFrontRight < 100){
    motorSpeedFrontRight = 100;
  }
  if (motorSpeedBackRight > 800){
    motorSpeedBackRight = 800;
  }
  if (motorSpeedBackRight < 100){
    motorSpeedBackRight = 100;
  }
  if (motorSpeedBackLeft > 800){
    motorSpeedBackLeft = 800;
  }
  if (motorSpeedBackLeft < 100){
    motorSpeedBackLeft = 100;
  }
}

/* //test
  motorSpeedBackLeft = 0;
  motorSpeedBackRight = 0;
  motorSpeedFrontRight = 0;


//end of test */

if (motor_on == 0){
  motorSpeedBackLeft = 0;
  motorSpeedBackRight = 0;
  motorSpeedFrontRight = 0;
  motorSpeedFrontLeft = 0;
}

/*   motorSpeedBackLeft = 0; */
 // motorSpeedBackRight = 0;
/*  motorSpeedFrontRight = 0;
   motorSpeedFrontLeft = 0; */

  motorSpeedFrontLeft_mapped = map(motorSpeedFrontLeft, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC_FrontLeft.write(motorSpeedFrontLeft_mapped);    // Send the signal to the ESC

  motorSpeedBackLeft_mapped = map(motorSpeedBackLeft, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC_BackLeft.write(motorSpeedBackLeft_mapped);    // Send the signal to the ESC

  motorSpeedFrontRight_mapped = map(motorSpeedFrontRight, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC_FrontRight.write(motorSpeedFrontRight_mapped);    // Send the signal to the ESC

  motorSpeedBackRight_mapped = map(motorSpeedBackRight, 0, 1023, 0, 180);   // scale it to use it with the servo library (value between 0 and 180)
  ESC_BackRight.write(motorSpeedBackRight_mapped);    // Send the signal to the ESC


}

void print_function(float ax, float ay, float az, float gx, float gy, float gz, float rollAcc, float yawAcc, float pitchAcc, float gX, float gY, float gZ, float x_k_prev[6], 
float gyro_angles[3], int input_motor_speed_changed, int input_motor_speed, int input_motor_speed_changed_bool, int motorSpeedFrontLeft,
int motorSpeedFrontRight, int motorSpeedBackLeft, int motorSpeedBackRight, float ar, float ar_cal, float ar_average){
Serial.print("ax ");
Serial.print(ax, DEC);
Serial.print(", ay ");
Serial.print(ay, DEC);
Serial.print(", az ");
Serial.print(az, DEC);

Serial.print(", gx ");
Serial.print(gx, DEC);

Serial.print(", gy ");
Serial.print(gy, DEC);

Serial.print(", gz ");
Serial.print(gz, DEC);
Serial.print(", roll: "); Serial.print(rollAcc);
Serial.print(", yaw: "); Serial.print(yawAcc);
Serial.print(", pitch: "); Serial.print(pitchAcc);
/*     Serial.print(" cos(v): "); Serial.print(cos(v_Acc));
Serial.print(" sin(w): "); Serial.print(sin(w_Acc)); */
Serial.print(", X: "); Serial.print(gX);
Serial.print(", Y: "); Serial.print(gY);
Serial.print(", Z: "); Serial.print(gZ);


// Print estimated state
Serial.print(", x: "); Serial.print(x_k_prev[0]);
/* Serial.print("x_normal: "); Serial.print(x_k[0]); */
Serial.print(", y: "); Serial.print(x_k_prev[1]);
Serial.print(", z: "); Serial.print(x_k_prev[2]);
Serial.print(", vx: "); Serial.print(x_k_prev[3]);
Serial.print(", vy: "); Serial.print(x_k_prev[4]);
Serial.print(", vz: "); Serial.print(x_k_prev[5]);

Serial.print(", w: "); Serial.print(gyro_angles[0]);
Serial.print(", v: "); Serial.print(gyro_angles[1]);
Serial.print(", u: "); Serial.print(gyro_angles[2]);
Serial.print("\n");
Serial.print("w: "); Serial.print(gyro_angles[0]);
Serial.print(", v: "); Serial.print(gyro_angles[1]);
Serial.print(", u: "); Serial.print(gyro_angles[2]);
Serial.print("\n");

Serial.print("input_motor_speed_changed: ");
Serial.println(input_motor_speed_changed);
Serial.print("input_motor_speed: ");
Serial.println(input_motor_speed);
Serial.print("input_motor_speed_changed_bool: ");
Serial.print(input_motor_speed_changed_bool);
Serial.print("\n");

Serial.print("FrontLeft pwm signal 0-180: ");
Serial.print(motorSpeedFrontLeft);
int motorSpeedFrontLeft_OUTPUT = digitalRead(9);
/* Serial.print(" OUTPUT: ");
Serial.print(motorSpeedFrontLeft_OUTPUT); */

Serial.print(", FrontRight pwm signal 0-180: ");
Serial.print(motorSpeedFrontRight);
Serial.print(", BackLeft pwm signal 0-180: ");
Serial.print(motorSpeedBackLeft);
Serial.print(", BackRight pwm signal 0-180: ");
Serial.print(motorSpeedBackRight);

/* Serial.print("\n");
Serial.print("mX: ");
Serial.print(mx);
Serial.print(" mY: ");
Serial.print(my);
Serial.print(" mZ: ");
Serial.print(mz); */
Serial.print('\n');
Serial.print("ar: ");
Serial.print(ar);
Serial.print('\n');
Serial.print("ar_cal: ");
Serial.print(ar_cal);
Serial.print('\n');
Serial.print("ar_average: ");
Serial.print(ar_average);
Serial.print('\n');


/* Serial.print("mag x = "); Serial.print(mx); Serial.println(" uT");
Serial.print("mag y = "); Serial.print(my); Serial.println(" uT");
Serial.print("mag z = "); Serial.print(mz); Serial.println(" uT");
Serial.print("mag xy= "); Serial.print(sqrt(mx*mx+my*my)); Serial.println(" uT");
Serial.print("mag xyz= "); Serial.print(sqrt(mz*mz+my*my+mx*mx)); Serial.println(" uT"); */
float compassDegree = bmm150.getCompassDegree();
Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
Serial.println(compassDegree);
Serial.print('\n');
Serial.print("--------------------------------");
Serial.print("\n");
}

// Quaternion representing orientation
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Mahony filter parameters
float twoKp = 2.0f * 0.5f;  // Proportional gain
float twoKi = 2.0f * 0.1f;  // Integral gain
/* float twoKp = 2.0f * 2.0f;  // Proportional gain
float twoKi = 2.0f * 0.0f;  // Integral gain */
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // Integral feedback
// Function to update orientation
void mahonyUpdate(float gx, float gy, float gz, 
                  float ax, float ay, float az, 
                  float mx, float my, float mz, 
                  float deltaTime) {
    // Normalize accelerometer measurement
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;  // Prevent division by zero
    ax /= norm;
    ay /= norm;
    az /= norm;

    // Normalize magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return;  // Prevent division by zero
    mx /= norm;
    my /= norm;
    mz /= norm;

    // Reference direction of Earth's magnetic field
    float hx = mx * (q0 * q0 - 0.5f + q1 * q1) + 
               my * (q1 * q2 - q0 * q3) + 
               mz * (q1 * q3 + q0 * q2);
    float hy = mx * (q1 * q2 + q0 * q3) + 
               my * (q0 * q0 - 0.5f + q2 * q2) + 
               mz * (q2 * q3 - q0 * q1);
    float bx = sqrt(hx * hx + hy * hy);
    float bz = mx * (q1 * q3 - q0 * q2) + 
               my * (q2 * q3 + q0 * q1) + 
               mz * (q0 * q0 - 0.5f + q3 * q3);

    // Estimated direction of gravity and magnetic field
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    float wx = 2.0f * bx * (0.5f - q2 * q2 - q3 * q3) + 
               2.0f * bz * (q1 * q3 - q0 * q2);
    float wy = 2.0f * bx * (q1 * q2 - q0 * q3) + 
               2.0f * bz * (q0 * q1 + q2 * q3);
    float wz = 2.0f * bx * (q0 * q2 + q1 * q3) + 
               2.0f * bz * (0.5f - q1 * q1 - q2 * q2);

    // Error is the cross-product between measured and estimated direction of gravity
    float ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    float ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    float ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    // Apply integral feedback if enabled
    if (twoKi > 0.0f) {
        integralFBx += twoKi * ex * deltaTime;  // integral error scaled by Ki
        integralFBy += twoKi * ey * deltaTime;
        integralFBz += twoKi * ez * deltaTime;
        gx += integralFBx;  // apply integral feedback
        gy += integralFBy;
        gz += integralFBz;
    } else {
        integralFBx = 0.0f;  // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * ex ;
    gy += twoKp * ey ;
    gz += twoKp * ez ;

    // Integrate rate of change of quaternion
    gx *= (0.5f * deltaTime);  // pre-multiply common factors
    gy *= (0.5f * deltaTime);
    gz *= (0.5f * deltaTime);
    float qa = q0;
    float qb = q1;
    float qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalize quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
/*     Serial.print("gx ");
    Serial.print(gx);
        Serial.print("gy ");
    Serial.print(gy);
        Serial.print("gz ");
    Serial.print(gz);
        Serial.print("ex ");
    Serial.print(ex);
            Serial.print("ey ");
    Serial.print(ey);
            Serial.print("ez ");
    Serial.print(ez);
            Serial.print("q1 ");
    Serial.print(q1);
            Serial.print("q2 ");
    Serial.print(q2);
            Serial.print("q3 ");
    Serial.print(q3);
            Serial.print("q0 ");
    Serial.print(q0);
    delay(100); */
}

float roll_mahogny;
float pitch_mahogny;
float yaw_mahogny;
// Function to retrieve roll, pitch, and yaw from quaternion
void getEulerAngles(float &roll, float &pitch, float &yaw) {
    roll_mahogny = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    pitch_mahogny = asin(2.0f * (q0 * q2 - q3 * q1));
    yaw_mahogny = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
/*     Serial.print("2.0f * (q0 * q1 + q2 * q3): ");
    Serial.print(2.0f * (q0 * q1 + q2 * q3));
        Serial.print("1.0f - 2.0f * (q1 * q1 + q2 * q2): ");
    Serial.print(1.0f - 2.0f * (q1 * q1 + q2 * q2));
        Serial.print("2.0f * (q0 * q3 + q1 * q2): ");
    Serial.print(2.0f * (q0 * q3 + q1 * q2));
            Serial.print("1.0f - 2.0f * (q2 * q2 + q3 * q3): ");
    Serial.print(1.0f - 2.0f * (q2 * q2 + q3 * q3));
    delay(500); */
}

// Smoothing factor for low-pass filter (adjust between 0.8 and 0.98 based on noise level)
float alpha = 0.96;

// Filtered sensor data
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
float filteredGyroX = 0, filteredGyroY = 0, filteredGyroZ = 0;
float filteredMagX = 0, filteredMagY = 0, filteredMagZ = 0;

void applyLowPassFilter(float rawAccelX, float rawAccelY, float rawAccelZ,
                        float rawGyroX, float rawGyroY, float rawGyroZ,
                        float rawMagX, float rawMagY, float rawMagZ) {
    // Accelerometer
    filteredAccelX = alpha * filteredAccelX + (1.0f - alpha) * rawAccelX;
    filteredAccelY = alpha * filteredAccelY + (1.0f - alpha) * rawAccelY;
    filteredAccelZ = alpha * filteredAccelZ + (1.0f - alpha) * rawAccelZ;

    // Gyroscope
    filteredGyroX = alpha * filteredGyroX + (1.0f - alpha) * rawGyroX;
    filteredGyroY = alpha * filteredGyroY + (1.0f - alpha) * rawGyroY;
    filteredGyroZ = alpha * filteredGyroZ + (1.0f - alpha) * rawGyroZ;

    // Magnetometer
    filteredMagX = alpha * filteredMagX + (1.0f - alpha) * rawMagX;
    filteredMagY = alpha * filteredMagY + (1.0f - alpha) * rawMagY;
    filteredMagZ = alpha * filteredMagZ + (1.0f - alpha) * rawMagZ;
}



float gX;
float gY;
float gZ;
float rollMag;
float pitchMag;
float yawMag;
void Kalman_filter(float gx, float gy, float gz, float dt, float ax, float ay, float az, float mx, float my, float mz ){
//Kalman_filter();
    // 1. Prediction Step
    float w_pred = gyro_angles[0] + gx * dt;
    float v_pred = gyro_angles[1] + gz * dt;
    float u_pred = gyro_angles[2] + gy * dt;
/*     float w = gyro_angles[0] + gx * dt;
    float v = gyro_angles[1] + gz * dt;
    float u = gyro_angles[2] + gy * dt; */
   /*  float w_Acc = atan2(az, ay);
    float v_Acc = atan2(az, ax);
    float u_Acc = atan2(ay, ax); // Approximation for yaw from accelerometer
 */    // Update covariance matrix P

    P_gyro[0][0] += Q_gyro[0][0];
    P_gyro[1][1] += Q_gyro[1][1];
    P_gyro[2][2] += Q_gyro[2][2];
/*     float rollAcc = gyro_angles[0] + gx * dt;
    float pitchAcc = gyro_angles[1] + gz * dt;
    float yawAcc = gyro_angles[2] + gy * dt; */
    // 2. Update Step
/*     if (-11 <= ay && ay <= -1){
    rollAcc = 3.14 - asin(az/sqrt(ay*ay+az*az));
    }
    if (1 <= ay && ay <= 11){
    rollAcc = asin(az/sqrt(ay*ay+az*az));
    }
    
    if (-11 <= ax && ax< -1){
    pitchAcc = 3.14 - asin(ay/sqrt(ay*ay+ax*ax));
    }
    if (1 <= ax && ax< 11){
    pitchAcc = asin(ay/sqrt(ay*ay+ax*ax));
    }

    if (-11 <= ax && ax < -1){
    yawAcc = 3.14 - asin(az/sqrt(az*az+ax*ax));
    }
    if (1 <= ax && ax < 11){
    yawAcc = asin(az/sqrt(az*az+ax*ax));
    } */

/*     rollMag = atan2(my, mz);
    pitchMag = atan2(mx, my);
    yawMag = atan2(mx, mz); // Approximation for yaw from accelerometer */
   // float rollAcc = atan2(ay, az);
   //// float pitchAcc = 0;
   /*  float pitchAcc = atan2(ax, sqrt(ay * ay + az * az));
    float yawAcc = atan2(az, sqrt(ax * ax + ay * ay)); // Approximation for yaw from accelerometer */
   // float yawAcc = atan2(ax, az);

//kalmanfilter ------------

    rollMag = atan2(my, sqrt(mx * mx + mz * mz));
    pitchMag = atan2(my, mz);  ////skip this. Doesnt work if x-y rotation
    yawMag = atan2(mx, sqrt(mz * mz + my * my)); // Approximation for yaw from accelerometer

    float rollAcc = atan2(ay, sqrt(ax * ax + az * az)); //not true pitch 
    float pitchAcc = atan2(ay, az);//skip
    float yawAcc = atan2(ax, sqrt(ay * ay + az * az)); // Approximation for yaw from accelerometer


/*     // Measurement residual (innovation)
    float yRoll = rollAcc - w_pred;
    float yPitch = pitchAcc - v_pred;
    float yYaw = yawAcc - u_pred; //ksk fel */

    // Measurement residual (innovation)
   // float yRoll = rollAcc - w_pred;
   // float yPitch = pitchAcc - v_pred;
   float yRoll = rollAcc - w_pred;
    float yPitch = 0;
    float yYaw = yawAcc - u_pred; //ksk fel
 
/*     float magDifRoll = (rollMag - rollMag_cal) - w_pred;
    float magDifPitch = (pitchMag - pitchMag_cal) - v_pred;
    float magDifYaw = (yawMag - yawMag_cal) - u_pred; //ksk fel  */
/*     float magDifRoll = (rollMag - rollMag_cal) - w_pred;
    float magDifPitch = (pitchMag - pitchMag_cal) - v_pred;
    float magDifYaw = (yawMag - yawMag_cal) - u_pred; //ksk fel */
    float magDifRoll = rollMag - w_pred;
    float magDifPitch = pitchMag - v_pred;
    float magDifYaw = yawMag - u_pred; //ksk fel
    // Kalman gain
    float S_roll = P_gyro[0][0] + R_gyro[0][0];
    float S_pitch = P_gyro[1][1] + R_gyro[1][1];
    float S_yaw = P_gyro[2][2] + R_gyro[2][2];

    float K_roll = P_gyro[0][0] / S_roll;
    float K_pitch = P_gyro[1][1] / S_pitch;
    float K_yaw = P_gyro[2][2] / S_yaw;

/*     // Update state estimate
    gyro_angles[0] = w_pred + K_roll * yRoll;
    gyro_angles[1] = v_pred + K_pitch * yPitch;
    gyro_angles[2] = u_pred + K_yaw * yYaw; */

     // Update state estimate

    gyro_angles[0] = w_pred + K_roll * yRoll + K_roll * magDifRoll;
    gyro_angles[1] = v_pred + K_pitch * yPitch + K_pitch*magDifPitch;
    gyro_angles[2] = u_pred + K_yaw * yYaw + K_yaw*magDifYaw;
/*     gyro_angles[0] = w_pred + K_roll * yRoll + K_roll * magDifRoll;
    gyro_angles[1] = v_pred + K_pitch * yPitch + K_pitch*magDifPitch;
    gyro_angles[2] = u_pred + K_yaw * yYaw + K_yaw*magDifYaw; */

 /*    if (gyro_angles_saved_counter < 50){
    gyro_angles_saved_counter += 1;
    gyro_angles_saved[0] += w_pred + K_roll * yRoll;
    gyro_angles_saved[1] += v_pred + K_pitch * yPitch;
    gyro_angles_saved[2] += u_pred + K_yaw * yYaw;
    
    ar_saved += ar;
    } */


 /*    else{
    gyro_angles_saved_counter = 0;

    gyro_angles_saved[0] += w_pred + K_roll * yRoll;
    gyro_angles_saved[1] += v_pred + K_pitch * yPitch;
    gyro_angles_saved[2] += u_pred + K_yaw * yYaw;
    
    ar_saved += ar;

    gyro_angles_saved[0] /= 50;
    gyro_angles_saved[1] /= 50;
    gyro_angles_saved[2] /= 50;

    ar_saved /= 50;
    } */

/*     ar_average += ar;
    if (ar_average_counter > 1){
      ar_average /= 2;
    }
    else{
      ar_average_counter += 1;
    } */

/*     gyro_angles[0] = rollAcc;
    gyro_angles[1] = pitchAcc;
    gyro_angles[2] = yawAcc; */
/*     gyro_angles[0] = w;
    gyro_angles[1] = v;
    gyro_angles[2] = u; */
    // Update covariance estimate
    P_gyro[0][0] *= (1 - K_roll);
    P_gyro[1][1] *= (1 - K_pitch);
    P_gyro[2][2] *= (1 - K_yaw);

gX = ax - 9.82 * (sin(gyro_angles[1]));
gY = ay - 9.82 * (sin(gyro_angles[0]*cos(gyro_angles[1])));
gZ = az - (cos(gyro_angles[0]) * cos(gyro_angles[1]) )* 9.82;


/* if (gyro_angles_saved_counter == 0){

changeMotorSpeed(gX, gY, gZ, gyro_angles_saved, ar, mx, my, mz, input_motor_speed_changed, ax, ay, az, gx, gy, gz, dt);
gyro_angles_saved[0] = 0;
gyro_angles_saved[1] = 0;
gyro_angles_saved[2] = 0;
ar_saved = 0;

} */

/* float gX = ax - 9.82 * (cos(gyro_angles[2])*cos(gyro_angles[1]));
float gY = ay - 9.82 * (cos(gyro_angles[0])*cos(gyro_angles[1]));
float gZ = az - (sin(gyro_angles[2])*sin(gyro_angles[0]))* 9.82; */
//Ta bort v?
    // Subtract gravity from accelerometer to get linear acceleration
/*     float gX = ax*9.82/2048 - 9.82*(cos(v_Acc)-sin(w_Acc));
    float gY = ay*9.82/2048 - (cos(w_Acc)) * 9.82;
    float gZ = az*9.82/2048 - (sin(w_Acc)-cos(v_Acc)) * 9.82; */
}


//remove when not testing!!
/* int mahony_list_counter = 0; */
/* float roll_mahony_list[1000] = 2.0f*{0};
float pitch_mahony_list[1000] = 2.0f*{0};
float yaw_mahony_list[1000] = 2.0f*{0}; */


int never_run_wifi = 1;

float mx;
float my;
float mz;

float receive_time;
float end_receive_time;
float mag_scale;
float ar_average = 0;
int ar_average_counter = 1;
int pulseWidth_counter = 0;
float dt = 0.005;
float loop_time = 5;
float start_time;
float end_time = 0;
int rf_signal_saved = 0;
int rf_signal_counter = 0;
void loop()
{
receive_time = millis();
/*  Serial.print("Pulse Widthinterupt: ");
  Serial.println(pulseWidth_interrupt); */
if (pulseWidth_counter > 7){ //5 if 15000 or 7 if 25000
pulseWidth = pulseIn(receiverPin, HIGH, 25000); // 25 ms timeout
/* Serial.print("pulseWidth "); Serial.print(pulseWidth);Serial.print(" ");
Serial.print("\n");  */

if ((998 <= pulseWidth) && (pulseWidth <= 1002)){
  pulseWidth = 1000;
}

if (rf_signal_saved == pulseWidth){
  rf_signal_counter += 1;
}
if (rf_signal_counter == 2){
rf_signal_counter = 0;
input_motor_speed_changed = pulseWidth - 1000;

}

rf_signal_saved = pulseWidth;
pulseWidth_counter = 0;

}

else{
  pulseWidth_counter += 1;
}
end_receive_time = millis();
loop_time += end_receive_time - receive_time;
/*   if (pulseWidth_interrupt > 0) {
        Serial.println("pulseWidth_interrupt "); Serial.println(pulseWidth_interrupt); Serial.println(" "); // Output the pulse width
    pulseWidth_interrupt = 0;  // Reset pulse width after reading
  } */
/* float test_time = millis();
if (pulseWidth_interrupt > 0) {
  Serial.print("pulseWidth_interrupt ");   Serial.print(pulseWidth_interrupt);  Serial.print(" "); 

  pulseWidth_interrupt = 0;  // Reset pulse width after reading
}
float end_test_time = millis(); */
/* Serial.print("end_test_time "); Serial.print(end_test_time-test_time);Serial.print(" ");  */

//Main code after receive motor signal

start_time = millis();
sBmm150MagData_t magData = bmm150.getGeomagneticData(); //ngt fel här. Den tar lång tid och ger fel vinklar
if (end_time == 0){
//sBmm150MagData_t magData = bmm150.getGeomagneticData();
for (int i = 0; i < 10; i++) {

float mx_setup = magData.x;
float my_setup = -magData.y;
float mz_setup = magData.z;
calibrate_mag(mx_setup, my_setup, mz_setup);
delay(100);
}

/* rollMag_cal /= 10;
pitchMag_cal /= 10;
yawMag_cal /= 10; */
mx_cal /= 10;
my_cal /= 10;
mz_cal /= 10;
 if (Serial) {
Serial.print("Magnetometer calibrated!");
    
 }
//LED on
digitalWrite(1, LOW);
}



// --- Lectura acelerometro y giroscopio ---
uint8_t Buf[14];
I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);

// Convertir registros acelerometro
int16_t ax_raw = -(Buf[0] << 8 | Buf[1]);
int16_t ay_raw = -(Buf[2] << 8 | Buf[3]);
int16_t az_raw = Buf[4] << 8 | Buf[5];

// Convertir registros giroscopio
int16_t gx_raw = -(Buf[8] << 8 | Buf[9]);
int16_t gy_raw = -(Buf[10] << 8 | Buf[11]);
int16_t gz_raw = Buf[12] << 8 | Buf[13];

// --- Lectura del magnetometro ---
/* uint8_t ST1;
I2CwriteByte(MAG_ADDRESS,0x0A,0x01);
do
{
I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);
} while (!(ST1 & 0x01));

uint8_t Mag[7];
I2Cread(MAG_ADDRESS, 0x03, 7, Mag);

// Convertir registros magnetometro
int16_t mx = -(Mag[3] << 8 | Mag[2]);
int16_t my = -(Mag[1] << 8 | Mag[0]);
int16_t mz = -(Mag[5] << 8 | Mag[4]); */

// --- Mostrar valores ---



// Acelerometro

float ax = ax_raw*9.82/2048 + 9.82*calib.accelBias[0]; //+ bc of how Buf gives values and how the sensor coordiante system is made
float ay = ay_raw*9.82/2048 + 9.82*calib.accelBias[1];
float az = az_raw*9.82/2048 - 9.82*calib.accelBias[2];

float gx = gx_raw/16.4 + calib.gyroBias[0];  // to radians
float gy = gy_raw/16.4 + calib.gyroBias[1];  // to radians 
float gz = gz_raw/16.4 - calib.gyroBias[2];  // to radians

// Convert gyroscope data to radians/sec
gx *= 0.0174533;  // to radians
gy *= 0.0174533;  // to radians
gz *= 0.0174533;  // to radians

// Giroscopio

// Read compass values
/* compass.read(); */
mag_scale = 1600/4096;
// Return XYZ readings
/* mx = compass.getX()*mag_scale;
my = compass.getY()*mag_scale;
mz = compass.getZ()*mag_scale; */

//Getting mag data but this is slow - 5ms
/* sBmm150MagData_t magData = bmm150.getGeomagneticData();
mx = magData.x;
my = magData.y;
mz = magData.z; */

mx = magData.x;
my = -magData.y;
mz = magData.z;
/* mx = mx - mx_cal;
my = my - my_cal;
mz = mz - mz_cal; */
// Magnetometro
/* Serial.print(mx + 200, DEC);
Serial.print("\t");
Serial.print(my - 70, DEC);
Serial.print("\t");
Serial.print(mz - 700, DEC);
Serial.print("\t"); */



// Fin medicion
/* Serial.println(""); */
/* appendToList(ax); */
/* if (pulseWidth_counter < 10){
 dt = 0.005; // Time step (seconds)
}
else{
  dt = 0.03333; // Time step (seconds)
} */


dt = loop_time/1000; // should be 5ms or 33 ms one loop out of 1. This way delays the 33ms loop 1 step but that is ok.

    //polar coordinates
    float ar = sqrt(ax*ax+ay*ay+az*az);
/*     float teta = atan(ay/ax);
    float phi = acos(az/ar); */

/*     float vector[3] = {ax, ay, az};
    float axis[3] = {ax, ay, az};
    float theta = gx; */

/*     make_rotation(vector,axis,theta); */
 /*    make_rotation(float vector[3], float axis[3], float theta)
    make_rotation(float vector[3], float axis[3], float theta) */


applyLowPassFilter(ax, ay, az,
                        gx, gy, gz,
                        mx, my, mz);

/*      mahonyUpdate(gx,gy,gz, 
                  ax, ay, az, 
                  mx, my, mz, 
                  dt); */


mahonyUpdate(filteredGyroX,filteredGyroY,filteredGyroZ, 
                  filteredAccelX, filteredAccelY, filteredAccelZ, 
                  filteredMagX, filteredMagY, filteredMagZ, 
                  dt);

getEulerAngles(roll_mahogny, pitch_mahogny, yaw_mahogny);


//Kalman_filter(gx, gy, gz, dt, ax, ay, az, mx, my, mz );
changeMotorSpeed(gX, gY, gZ, gyro_angles, ar, mx, my, mz, input_motor_speed_changed, ax, ay, az, gx, gy, gz, dt, roll_mahogny, pitch_mahogny, yaw_mahogny);

    // gX, gY, gZ now represent the acceleration in m/s² without gravity
 
/*     Serial.print("Linear Acceleration: "); */
/*     Serial.print(", w: "); Serial.print(w_Acc);
    Serial.print(", v: "); Serial.print(v_Acc);
    Serial.print(", u: "); Serial.print(u_Acc); */

// Predict step
//predict(dt, gX, gY, gZ, gx, gy, gz);

// Simulated measurement [x, y]
/* float z[2] = {10.0, 5.0}; */

// Update step
//update();

//Printing start

/* print_function(ax, ay, az, gx, gy, gz, rollAcc, yawAcc, pitchAcc, gX, gY, gZ, x_k_prev, 
gyro_angles, input_motor_speed_changed, input_motor_speed, input_motor_speed_changed_bool, motorSpeedFrontLeft, 
motorSpeedFrontRight, motorSpeedBackLeft, motorSpeedBackRight, ar, ar_cal, ar_average); */
/* Serial.print("mx ");
Serial.print(mx, DEC);

Serial.print(", my ");
Serial.print(my, DEC);

Serial.print(", mz ");
Serial.print(mz, DEC); */
//delay(1000);

/* 
Serial.print("P0: ");
Serial.print(P_gyro[0][0]);
Serial.print("P1: ");
Serial.print(P_gyro[1][1]);
Serial.print("P2: ");
Serial.print(P_gyro[2][2]);

Serial.print("yRoll: ");
Serial.print(yRoll);
Serial.print("yPitch: ");
Serial.print(yPitch);
Serial.print("yYaw: ");
Serial.print(yYaw);

Serial.print("S_roll: ");
Serial.print(S_roll);
Serial.print("S_pitch: ");
Serial.print(S_pitch);
Serial.print("S_yaw: ");
Serial.print(S_yaw);

Serial.print("K_roll: ");
Serial.print(K_roll);
Serial.print("K_pitch: ");
Serial.print(K_pitch);
Serial.print("K_yaw: ");
Serial.print(K_yaw);
Serial.print("\n"); */

//Print_base_parameters(ax, ay, az, gx, gy, gz, gyro_angles, dt, motorSpeedFrontLeft, motorSpeedFrontRight, motorSpeedBackLeft, motorSpeedBackRight, roll_mahogny, pitch_mahogny, yaw_mahogny);
/* Serial.print("ax ");
Serial.print(ax, DEC);
Serial.print(", ay ");
Serial.print(ay, DEC);
Serial.print(", az ");
Serial.print(az, DEC);
Serial.print("mx ");
Serial.print(mx, DEC);
Serial.print(", my ");
Serial.print(my, DEC);
Serial.print(", mz ");
Serial.print(mz, DEC);
Serial.print("mx_cal ");
Serial.print(mx_cal, DEC);
Serial.print(", my_cal ");
Serial.print(my_cal, DEC);
Serial.print(", mz_cal ");
Serial.print(mz_cal, DEC);
Serial.print("rollMag ");
Serial.print(rollMag, DEC);
Serial.print(", pitchMag ");
Serial.print(pitchMag, DEC);
Serial.print(", yawMag ");
Serial.print(yawMag, DEC);

Serial.print("rollAcc ");
Serial.print(rollAcc, DEC);

Serial.print(", yawAcc ");
Serial.print(yawAcc, DEC);

Serial.print(", pitchAcc ");
Serial.print(pitchAcc, DEC);
Serial.print("\n");
Serial.print(", gx*dt ");
Serial.print(gx*dt, DEC);
Serial.print(", gy*dt ");
Serial.print(gy*dt, DEC);
Serial.print(", gz*dt ");
Serial.print(gz*dt, DEC);
 */
/* float print_time = millis();
Serial.print("dt ");
Serial.print(dt, DEC);
float end_print_time = millis();
Serial.print("print_time: ");
float time = print_time-end_print_time;
Serial.print(time); */
/* Serial.print(", rollAcc_mahogny ");
Serial.print(roll_mahogny, DEC);
Serial.print(", pitchAcc_mahogny ");
Serial.print(pitch_mahogny, DEC);
Serial.print(", yawAcc_mahogny ");
Serial.print(yaw_mahogny, DEC) ;
Serial.print("\n");
Serial.print("----------");
Serial.print("\n");  */

//Printing end
delay(1);

/* printList(); */
//delayMicroseconds(1000000);
//float wifi_time = millis();

/* roll_mahony_list[mahony_list_counter] = roll_mahogny;
Serial.print(roll_mahony_list);
pitch_mahony_list[mahony_list_counter] = pitch_mahogny;
yaw_mahony_list[mahony_list_counter] = yaw_mahogny;
roll_mahony_list[mahony_list_counter + 1] = 0;
pitch_mahony_list[mahony_list_counter + 1] = 0;
yaw_mahony_list[mahony_list_counter + 1] = 0; */
/* mahony_list_counter += 1;
if (mahony_list_counter == 1000){
  mahony_list_counter = 0;
} */


// Send continuous data to the client 
mockSensorValue = "roll_mahogny: " + String(roll_mahogny, 5) + ", pitch_mahogny: " + String(pitch_mahogny, 5) + ", yaw_mahogny: " + String(yaw_mahogny, 5) + "motorSpeedFrontRight" + String(motorSpeedFrontRight) + "motorSpeedFrontLeft" + String(motorSpeedFrontLeft) + "motorSpeedBackRight" + String(motorSpeedBackRight) + "motorSpeedBackLeft"  + String(motorSpeedBackLeft);  // Example: send time in milliseconds
// Process DNS and HTTP requests
dnsServer.processNextRequest();
webServer.handleClient();
  
 // client.print(roll_mahogny);
 /*  client.printf("roll_mahony: %f pitch_mahony: %f yaw_mahony: %f \n", roll_mahogny, pitch_mahogny, yaw_mahogny);
  client.printf("motorSpeedFrontLeft: %f motorSpeedFrontRight: %f motorSpeedBackLeft: %f motorSpeedBackRight: %f \n", motorSpeedFrontLeft, motorSpeedFrontRight, motorSpeedBackLeft, motorSpeedBackRight);
  client.printf("ax: %f, ay: %f, az: %f, gx: %f, gy %f, gz %f, mx %f, my %f, mz %f \n", ax, ay, az, gx, gy, gz, mx, my, mz );*/
end_time = millis();
loop_time = end_time - start_time;

/* Serial.print("input_motor_speed_changed:"); Serial.print(input_motor_speed_changed); Serial.print(" ");

Serial.print("\n"); */

/* Serial.print("roll_mahogny:"); Serial.print(roll_mahogny); Serial.print(" ");
Serial.print("pitch_mahogny:"); Serial.print(pitch_mahogny); Serial.print(" ");
Serial.print("ax:"); Serial.print(ax); Serial.print(" ");
Serial.print("ay:"); Serial.print(ay); Serial.print(" ");
Serial.print("az:"); Serial.print(az); Serial.print(" ");
Serial.print("filteredAccelX:"); Serial.print(filteredAccelX); Serial.print(" ");
Serial.print("filteredAccelY:"); Serial.print(filteredAccelY); Serial.print(" ");
Serial.print("filteredAccelZ:"); Serial.print(filteredAccelZ); Serial.print(" "); */
/* Serial.print("motorSpeedFrontLeft_mapped:"); Serial.print(motorSpeedFrontLeft_mapped); Serial.print(" ");  */
/* Serial.print("\n");
Serial.print("loop_time ");
Serial.print(loop_time);
Serial.print("loop_timestart ");
Serial.print(end_receive_time - receive_time); */
}