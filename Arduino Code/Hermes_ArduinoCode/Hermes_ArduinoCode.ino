#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MadgwickAHRS.h>

#define ICM_ADDR 0x6A

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Madgwick filter;

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

// Structure for 3D Vector
struct Vector3 {
  float x, y, z;
};

Vector3 gyroBias = {0, 0, 0};

// Structure for screen projection
struct Vector2 {
  int x, y;
};

// Define 8 cube vertices, centered at origin
Vector3 cubeVertices[8] = {
  {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
  {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
};

// Define edges as pairs of vertex indices
const uint8_t edges[12][2] = {
  {0,1}, {1,2}, {2,3}, {3,0}, // Draws back face
  {4,5}, {5,6}, {6,7}, {7,4}, // Draws front face
  {0,4}, {1,5}, {2,6}, {3,7} // Draws connecting lines back to front
};

struct Quaternion { float w, x, y, z; };
Quaternion q = {1, 0, 0, 0}; //identity quaternion to start

// Converts roll/pitch/yaw in degrees to a unit quaternion
// Rotation order = Z(yaw) * Y(pitch) * X(roll) which matches Madgwick’s getters
Quaternion eulerDegToQuat(float rollDeg, float pitchDeg, float yawDeg) {
  float r = 0.5f * rollDeg  * DEG_TO_RAD;
  float p = 0.5f * pitchDeg * DEG_TO_RAD;
  float y = 0.5f * yawDeg   * DEG_TO_RAD;

  float cr = cosf(r), sr = sinf(r);
  float cp = cosf(p), sp = sinf(p);
  float cy = cosf(y), sy = sinf(y);

  Quaternion q;
  q.w = cr*cp*cy + sr*sp*sy;
  q.x = sr*cp*cy - cr*sp*sy;
  q.y = cr*sp*cy + sr*cp*sy;
  q.z = cr*cp*sy - sr*sp*cy;
  return q;
}

void initICM() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x02); // CTRL1 Serial Interface and Sensor Enable
  Wire.write(0b01100000); // Enables auto-increment and Big-Endian
  Wire.endTransmission();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x03); // CTRL2 Accelerometer Settings
  Wire.write(0b00010111); // +-4g, 56.05Hz
  Wire.endTransmission();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x04); // CTRL3 Gyroscope Settings
  Wire.write(0b01010111); // +=512dps, 56.05Hz
  Wire.endTransmission();

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x08); // CTRL7 Enable Sensors and Configure Data Reads
  Wire.write(0b00000011); // aEN=1, gEN=1, enables accel/gyro
  Wire.endTransmission();

  //Wire.beginTransmission(ICM_ADDR);
  //Wire.write(0x06); // CTRL5 Sensor Data Processing Settings
  //Wire.write(0b00110011); // Enable accel and gyro Low-Pass-Filter with 3.63Hz
  //Wire.endTransmission();

  delay(200); // Wait for gyro to stabilize
}

void calibrateGyro(int samples = 500) {
  float sumX = 0, sumY = 0, sumZ = 0;
  for (int i = 0; i < samples; i++) {
    float gx, gy, gz;
    readGyro(gx, gy, gz);
    sumX += gx; sumY += gy; sumZ += gz;
    delayMicroseconds(microsPerReading);
  }
  gyroBias.x = sumX / samples;
  gyroBias.y = sumY / samples;
  gyroBias.z = sumZ / samples;
}

void readAccel(float& ax, float& ay, float& az) {
  int16_t ax_raw, ay_raw, az_raw;

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x35); // A[X,Y,Z]_[H,L] Acceleration Output. Register Address: 0x35 – 0x3A
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 6, true);

  int16_t lo, hi;

  lo = Wire.read(); hi = Wire.read(); ax_raw = (hi << 8) | lo;
  lo = Wire.read(); hi = Wire.read(); ay_raw = (hi << 8) | lo;
  lo = Wire.read(); hi = Wire.read(); az_raw = (hi << 8) | lo;

  // +-4g
  ax = (ax_raw * 4) / 32768.0;
  ay = (ay_raw * 4) / 32768.0;
  az = (az_raw * 4) / 32768.0;
}


void readGyro(float& gx, float& gy, float& gz) {
  int16_t gx_raw, gy_raw, gz_raw;

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x3B); // G[X,Y.Z]_[H,L] Angular Rate Output. Register Address: 0x3B – 0x40
  Wire.endTransmission(false); // Stops sending, but keep register open for reading
  Wire.requestFrom(ICM_ADDR, 6, true); // Reads a byte from each gyro register, starting at the first one. 2 Registers per axis

  int16_t lo, hi;

  lo = Wire.read(); hi = Wire.read(); gx_raw = (hi << 8) | lo;
  lo = Wire.read(); hi = Wire.read(); gy_raw = (hi << 8) | lo;
  lo = Wire.read(); hi = Wire.read(); gz_raw = (hi << 8) | lo;

  // +-512dps
  gx = (gx_raw * 512) / 32768.0; // Convert to deg/sec
  gy = (gy_raw * 512) / 32768.0;
  gz = (gz_raw * 512) / 32768.0;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(500);

  microsPerReading = 1000000 / 56.05;
  microsPrevious = micros();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  display.clearDisplay();

  initICM();
  calibrateGyro();
  filter.begin(56.05);
  
  Serial.print("Setup complete");
}

void loop() {
  int aix, aiy, aiz;
  int gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, heading;
  unsigned long microsNow;

  microsNow = micros();
  if (microsNow - microsPrevious >= microsPerReading) {
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);

    filter.updateIMU(
      (gy - gyroBias.y),
      (gx - gyroBias.x),
      (gz - gyroBias.z),
      ax, ay, az
    );

    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    Serial.print("Orientation: ");
    Serial.print(heading);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.println(roll);
    
    q = eulerDegToQuat(roll, pitch, heading);

    static int frameSkip = 0;
    if (++frameSkip >= 5) { // ~22 Hz display
      frameSkip = 0;
        display.clearDisplay();
        drawCube(q);
        display.display();
    }
    // Increment previous time, so we keep proper pace
    microsPrevious = microsPrevious + microsPerReading;
  }
}

// Combines two rotations represented by quaternions
Quaternion qMult(const Quaternion & a, const Quaternion & b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, //w (scalar component)
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, //i (x component)
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, //j (y component)
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  //k (z component)
  };
}

// Flips the sign of the imaginary parts, inverse
Quaternion qConj(const Quaternion& q){
  return {q.w, -q.x, -q.y, -q.z};
}

// Rotate a 3D Vector using a quaternion
Vector3 rotateByQuaternion(const Vector3& v, const Quaternion& q) {
  Quaternion p ={0, v.x, v.y, v.z}; // Represent the vector as a quaternion (pure quaternion, w=0)
  Quaternion qInv = qConj(q); // Invert the quaternion rotation
  Quaternion result = qMult(qMult(q,p), qInv); // Rotate the vector using quaternion sandwitch formula, rotatedVector = q * p * q^-1
  return {result.x, result.y, result.z}; // Return the rotated vector part, ignore scalar w
}

// Project a 3D point to 2D coordinates, orthographic
Vector2 projectTo2D(const Vector3& v) {
  float scale = 20.0; // Adjust for the cube size
  return {
    (int)(v.x * scale) + SCREEN_WIDTH / 2,
    (int)(-v.y * scale) + SCREEN_HEIGHT / 2
  };
}

// Draw the cube from the quaternion rotation
void drawCube(const Quaternion& q){
  Vector2 projected[8];

  for (int i = 0; i < 8; i++) {
    Vector3 rotated = rotateByQuaternion(cubeVertices[i], q);
    projected[i] = projectTo2D(rotated);
  }

  for (int i = 0; i < 12; i++) {
    Vector2 p1 = projected[edges[i][0]];
    Vector2 p2 = projected[edges[i][1]];
    display.drawLine(p1.x, p1.y, p2.x, p2.y, SSD1306_WHITE);
  }
}