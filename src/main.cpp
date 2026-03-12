#include <Arduino.h>
#include <AsyncTCP.h>
#include <WiFi.h>
#include <Wire.h>

#include <ESPAsyncWebServer.h>

#include <VescUart.h>

const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static AsyncWebServer server(80);
static AsyncWebSocket ws("/ws");

VescUart motorL;
VescUart motorR;

// put function declarations here:
void ReadMPU();

float M = 4;            // Bonler maass
float m = 2.9;          // Wheel mass
float Jb = pow(10,-3);  // body inertia
float Jw = pow(10,-3);  // Wheel inertia
float g = 9.82;
float l = 0.05;         // body COM length from wheel axis
float r = 0.085;        // Wheel radius

void setup() {

  Serial.begin(115200);
  Serial2.begin(115200);
  // Serial.println("Starting");
    Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  return;

  Model.Ac << 0, 0, 1, 0,
              0, 0, 0, 1,
              0, -(pow(M,2)*g*pow(l,2)*pow(r,2))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)), 0, 0,
              0, (M*l*(2*Jw*g + 2*g*m*pow(r,2) + M*g*pow(r,2)))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)), 0, 0;


  Model.Bc << 0, 0,
              0, 0,
              (r*(M*pow(l,2) + Jb))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)), (r*(M*pow(l,2) + Jb))/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)),
              -(M*l*r)/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2)), -(M*l*r)/(2*Jb*Jw + 2*Jw*M*pow(l,2) + Jb*M*pow(r,2) + 2*Jb*m*pow(r,2) + 2*M*pow(l,2)*m*pow(r,2));

  Model.discretize_state_matricies(); // creates Ad and Bd
  Model.solveRiccati(); // Creates K_lqr if solution is found
  
  WiFi.mode(WIFI_AP);
  // Serial.println("Starting AP");
  // WiFi.setTxPower(WIFI_POWER_8_5dBm); 
  WiFi.softAP("pro-bono", "ihardlyknowher");
  // Serial.println("Is ok?");
  

  return;
  
  while(!Serial) {;}
  while(!Serial2){;}
  
  motorL.setSerialPort(&Serial);
  motorR.setSerialPort(&Serial2);
  
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    (void)len;

    if (type == WS_EVT_CONNECT) {
      ws.textAll("new client connected");
      client->setCloseClientOnQueueFull(false);

    } else if (type == WS_EVT_DISCONNECT) {
      ws.textAll("client disconnected");

    } else if (type == WS_EVT_ERROR) {

    } else if (type == WS_EVT_PONG) {

    } else if (type == WS_EVT_DATA) {
      motorL.setCurrent(((float)data[0]) / 255.0f * 3.0f);
      motorR.setCurrent(-((float)data[1]) / 255.0f * 3.0f);
    }
  });

  /*server.addHandler(&ws).addMiddleware([](AsyncWebServerRequest *request, ArMiddlewareNext next) {
    if (ws.count() >= 1) {
      request->send(503, "text/plain", "Server is busy");
    } else {
      next();
    }
  });*/

  server.addHandler(&ws);

  server.begin();
}

float t = 0.0f;

void loop() {
  // motorL.setDuty(sin(t)* 1.0f);
  // t += 0.001;
  for(int i = 0; i < 100; i++){
    ReadMPU();
  }

  Serial.println(AcX);
  Serial.println(AcY);
  Serial.println(AcZ);
  Serial.println(GyX);
  Serial.println(GyY);
  Serial.println(GyZ);
}

void ReadMPU(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14);
  AcX = (Wire.read() << 8) | Wire.read();
  AcY = (Wire.read() << 8) | Wire.read();
  AcZ = (Wire.read() << 8) | Wire.read();
  Tmp = (Wire.read() << 8) | Wire.read();
  GyX = (Wire.read() << 8) | Wire.read();
  GyY = (Wire.read() << 8) | Wire.read();
  GyZ = (Wire.read() << 8) | Wire.read();
}