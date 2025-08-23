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

Madgwick filter;

unsigned long lastMicros = 0; //used for making sure loop runs at constant time
const unsigned long intervalMicros = 10000; //100Hz

//structure for Quaternion
//struct Quaternion {
  //float w, x, y, z;
//};

//structure for 3D Vector
struct Vector3 {
  float x, y, z;
};

//structure for screen projection
struct Vector2 {
  int x, y;
};

//define 8 cube vertices, centered at origin
Vector3 cubeVertices[8] = {
  {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1},
  {-1, -1, 1}, {1, -1, 1}, {1, 1, 1}, {-1, 1, 1}
};

//define edges as pairs of vertex indices
const uint8_t edges[12][2] = {
  {0,1}, {1,2}, {2,3}, {3,0}, //draws back face
  {4,5}, {5,6}, {6,7}, {7,0}, //draws front face
  {0,4}, {1,5}, {2,6}, {3,7} //draws connecting lines back to front
};

struct Quaternion { float w, x, y, z; };
Quaternion q = {1, 0, 0, 0}; //identity quaternion to start

//converts roll/pitch/yaw in degrees to a unit quaternion
//rotation order = Z(yaw) * Y(pitch) * X(roll) which matches Madgwick’s getters
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

void setup() {
  Serial.begin(9600);
  Wire.begin();

  display.begin();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  delay(100);

  initICM();

  filter.begin(100); //100Hz
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

    float roll  = filter.getRoll();   // degrees
    float pitch = filter.getPitch();  // degrees
    float yaw   = filter.getYaw();    // degrees  (will drift without a magnetometer)

    Quaternion q = eulerDegToQuat(roll, pitch, yaw);
  }
 
  float roll, pitch, yaw;
  quaternionToEuler(q, roll, pitch, yaw);

  Serial.print("Roll: ");  Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: ");   Serial.println(yaw);

  
  //Quaternion orientation = {qw , qx, qy, qz};

  display.clearDisplay();
  drawCube(q);
  //drawCube(orientation);
  display.display();
}

void initICM() {
  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x4E); //power management address
  Wire.write(0x0F); //low noise accel+gyro, idle off, temp sensor on
  delayMicroseconds(200); //delay before writing to any more registers per datasheet

  Wire.write(0x4F); //gyro config address
  Wire.write(0x26); //set Gyro settings, 000(+-2000dps) 001(+-1000dps) 010(+-500dps) 011(+-250dps) etc. then 0110 for default sets for 500dps
  
  Wire.write(0x50); //accel config address
  Wire.write(0x06); //set accell settings, 000(+-16g) 001(+-8g) 010(+-4g) then 0110 for default sets for 16g

  Wire.endTransmission(); //ends transmission
}

void readAccel(float& ax, float& ay, float& az) {
  int16_t ax_raw, ay_raw, az_raw;

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(0x3B);  // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(ICM_ADDR, 6, true);

  ax_raw = (Wire.read() << 8) | Wire.read();
  ay_raw = (Wire.read() << 8) | Wire.read();
  az_raw = (Wire.read() << 8) | Wire.read();

  ax = ax_raw * (16.0 / 32768.0);  
  ay = ay_raw * (16.0 / 32768.0);
  az = az_raw * (16.0 / 32768.0);
}


void readGyro(float& gx, float& gy, float& gz) {
  int16_t gx_raw, gy_raw, gz_raw;

  Wire.beginTransmission(ICM_ADDR);
  Wire.write(25); //registor for first gyro data
  Wire.endTransmission(false); //stops sending, but keep register open for reading
  Wire.requestFrom(ICM_ADDR, 6, true); //reads a byte from each gyro register, starting at the first one. 2 Registers per axis

  gx_raw = (Wire.read() << 8) | Wire.read(); //shifts the High byte 8 bits to the left, adding 8 zeros to the end. The OR operation combines this with the Low byte, making a 16 bit number representing the whole result.
  gy_raw = (Wire.read() << 8) | Wire.read();
  gz_raw = (Wire.read() << 8) | Wire.read();

  //default range, +-500dps
  gx = gx_raw * (500.0 / 32768.0); //correlates the raw value to the degrees per second
  gy = gy_raw * (500.0 / 32768.0);
  gz = gz_raw * (500.0 / 32768.0);
}

//combines two rotations represented by quaternions
Quaternion qMult(const Quaternion & a, const Quaternion & b) {
  return {
    a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, //w (scalar component)
    a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, //i (x component)
    a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, //j (y component)
    a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w  //k (z component)
  };
}

//flips the sign of the imaginary parts, inverse
Quaternion qConj(const Quaternion& q){
  return {q.w, -q.x, -q.y, -q.z};
}

//rotate a 3D Vector using a quaternion
Vector3 rotateByQuaternion(const Vector3& v, const Quaternion& q) {
  Quaternion p ={0, v.x, v.y, v.z}; //Represent the vector as a quaternion (pure quaternion, w=0)
  Quaternion qInv = qConj(q); //Invert the quaternion rotation
  Quaternion result = qMult(qMult(q,p), qInv); //Rotate the vector using quaternion sandwitch formula, rotatedVector = q * p * q^-1
  return {result.x, result.y, result.z}; //Return the rotated vector part, ignore scalar w
}

//project a 3D point to 2D coordinates, orthographic
Vector2 projectTo2D(const Vector3& v) {
  float scale = 20.0; //Adjust for the cube size
  return {
    (int)(v.x * scale) + SCREEN_WIDTH / 2,
    (int)(-v.y * scale) + SCREEN_HEIGHT / 2
  };
}

//draw the cube from the quaternion rotation
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

void quaternionToEuler(const Quaternion& q, float& roll, float& pitch, float& yaw) {
  //roll
  roll = atan2(2.0 * (q.w*q.x + q.y*q.z),
               1.0 - 2.0 * (q.x*q.x + q.y*q.y));

  //pitch
  pitch = asin(2.0 * (q.w*q.y - q.z*q.x));

  //yaw
  yaw = atan2(2.0 * (q.w*q.z + q.x*q.y),
              1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}








