#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MadgwickAHRS.h>

#define ICM_ADDR 0x69
#define OLED_ADDR 0x79

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RAD_TO_DEG 57.295779513 // Helps with converting radians to degrees for pitch/yaw

Madgwick filter;

unsigned long lastMicros = 0; // Used for making sure loop runs at constant time
const unsigned long intervalMicros = 10000; // 10ms, 100Hz

// Structure for Quaternion
struct Quaternion {
  float w, x, y, z;
};

// Structure for 3D Vector
struct Vector3 {
  float x, y, z;
};

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
  {0,1}, {1,2}, {2,3}, {3,0}, // draws back face
  {4,5}, {5,6}, {6,7}, {7,0}, // draws front face
  {0,4}, {1,5}, {2,6}, {3,7} // draws connecting lines back to front
};


void setup() {
  Serial.begin(9600);
  Wire.begin();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  delay(100);

  initICM();

  filter.begin(100); // 100Hz
}

void loop() {
  if (micros() - lastMicros >= intervalMicros) {
    lastMicros += intervalMicros;

    float ax, ay, az, gx, gy, gz;
    readAccel(ax, ay, az);
    readGyro(gx, gy, gz);

    filter.updateIMU(
      radians(gx), radians(gy), radians(gz),
      ax, ay, az
    );

    float qw = filter.q0;
    float qx = filter.q1;
    float qy = filter.q2;
    float qz = filter.q3;

    float roll  = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * RAD_TO_DEG;
    float pitch = asin(2.0f * (qw * qy - qz * qx)) * RAD_TO_DEG;
    float yaw   = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * RAD_TO_DEG;
  }
  // Debugging help
  Serial.print("Quat: ");
  Serial.print(qw, 4); Serial.print(", ");
  Serial.print(qx, 4); Serial.print(", ");
  Serial.print(qy, 4); Serial.print(", ");
  Serial.println(qz, 4);

  Serial.print("Euler [deg] - Roll: ");
  Serial.print(roll); Serial.print(" Pitch: ");
  Serial.print(pitch); Serial.print(" Yaw: ");
  Serial.println(yaw);
  
  Quaternion orientation = {qw , qx, qy, qz};

  diplay.clearDisplay();
  drawCube(orientation);
  display.display;
}

void initICM() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x4E); // Power management address
  Wire.write(0x0F); // Low noise accelerometer+gyro, idle off, temp sensor on
  delayMicroseconds(200); // Delay before writing to any more registers per datasheet

  Wire.write(0x4F); // Gyro config address
  Wire.write(0x26); // Set Gyro settings, 000(+-2000dps) 001(+-1000dps) 010(+-500dps) 011(+-250dps) etc. then 0110 for default Sets for 500dps
  
  Wire.write(0x50); // Accel config address
  Wire.write(0x06); // Set accell settings, 000(+-16g) 001(+-8g) 010(+-4g) then 0110 for default sets for 16g

  Wire.endTransmission(); // Ends transmission
}

void readAccel(float& ax, float& ay, float& az) {
  Wire.beginTransmission(ISM_ADDR);
  Wire.write(1F); // Registor for first accel data
  Wire.endTransmission(false); // Stops sending, but keep register open for reading

  Wire.requestFrom(ICM_ADDR, 6); // Reads a byte from each accelerometer register, starting at the first one. 2 Register for each axis.

  int16_t accelX, accelY, accelZ;
  ax_raw = (Wire.read() << 8 | Wire.read(); // Shifts the High byte 8 bits to the left, adding 8 zeros to the end. The OR operation combines this with the Low byte, making a 16 bit number representing the whole result.
  ay_raw = (Wire.read() << 8 | Wire.read();
  az_raw = (Wire.read() << 8 | Wire.read();
  // Default range, +- 16g
  float ax = ax_raw * (16.0 / 32768.0); // Correlates the raw value to a g value
  float ay = ay_raw * (16.0 / 32768.0);
  float az = az_raw * (16.0 / 32768.0);
}

void readGyro(float& gx, float& gy, float& gz) {
  Wire.beginTransmission(ISM_ADDR);
  Wire.write(25); // Registor for first gyro data
  Wire.endTransmission(false); // Stops sending, but keep register open for reading

  Wire.requestFrom(ICM_ADDR, 6); // Reads a byte from each gyro register, starting at the first one. 2 Register for each axis.

  int16_t gyroX, gyroY, gyroZ;
  gx_raw = (Wire.read() << 8 | Wire.read(); // Shifts the High byte 8 bits to the left, adding 8 zeros to the end. The OR operation combines this with the Low byte, making a 16 bit number representing the whole result.
  gy_raw = (Wire.read() << 8 | Wire.read();
  gz_raw = (Wire.read() << 8 | Wire.read();

  // Default range, +-500dps
  float gx = gx_raw * (500.0 / 32768.0); // Correlates the raw value to the degrees per second
  float gy = gy_raw * (500.0 / 32768.0);
  float gz = gz_raw * (500.0 / 32768.0);
}

// Combines two rotations represented by quaternions
Quaternion qMult(const Quaternion & a, const Quaternion & b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, // w (scalar component)
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, // i (x component)
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, // j (y component)
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  // k (z component)
  }
}

// Flips the sign of the imaginary parts, this is the inverse
Quaternion qConj(const Quaternion& q){
  return {q.w, -q,x, -q,y, -q,z};
}

// Rotate a 3D Vector using a Quaternion
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

  for int i = 0; i < 8; i++) {
    Vector3 rotated = rotateByQuaternion(cubeVertices[i], q);
    projected[i] = projectTo2D(rotated);
  }

  for (int i = 0; i < 12; i++) {
    Vector2 p1 = projected[edges[i][0]];
    Vector2 p2 = projected[edges[i][1]]:
    display.drawLine(p1.x, p2.y, p2.x, p2.y, SSD1306_WHITE);
  }
}









