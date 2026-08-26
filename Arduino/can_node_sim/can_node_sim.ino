#include <SPI.h>
#include <mcp_can.h>

const int CAN_CS_PIN = 10;          // MCP2515 CS pin
MCP_CAN CAN(CAN_CS_PIN);


#define MCP_CRYSTAL  MCP_8MHZ

#define ID_BMS_1  0x180
#define ID_BMS_2  0x181
#define ID_BMS_3  0x182
#define ID_MOT_1  0x190
#define ID_MOT_2  0x191
#define ID_MOT_3  0x192
#define ID_CHG_1  0x1A0
#define ID_CHG_2  0x1A1
#define ID_CHG_3  0x1A2

static void putU16LE(uint8_t *p, uint16_t v) { p[0] = v & 0xFF; p[1] = v >> 8; }
static uint16_t encOff(int16_t v)  { return (uint16_t)(v + 32000); } 
static uint8_t  encTemp(int8_t c)  { return (uint8_t)(c + 40); }

/* BMS */
uint16_t bmsVoltage_dV = 520;   // 52.0 V
int16_t  bmsCurrent_dA = 0;     // 0.1 A: + discharge / - charge
int8_t   bmsTemp_C     = 25;
uint8_t  bmsSoc        = 60;    // %
uint8_t  bmsState      = 1;     // 0 OFF,1 STANDBY,2 DISCHARGE,3 CHARGE,4 FAULT
uint8_t  bmsFlags      = 0x0C;  // bit0 fault, bit1 chargeAllowed, bit2 dischargeAllowed, bit3 contactor

/* Motor */
int16_t  motRpm        = 0;
int16_t  motCurrent_dA = 0;     // 0.1 A: + drive / - regen
int8_t   motTemp_C     = 30;
uint8_t  motState      = 1;     // 0 OFF,1 READY,2 RUNNING,3 FAULT
uint8_t  motDir        = 0;     // 0 neutral,1 forward,2 reverse
uint8_t  motFlags      = 0x00;  // bit0 fault
uint8_t  motAlive      = 0;

/* Charger */
uint16_t chgVoltage_dV = 0;
uint16_t chgCurrent_dA = 0;     // 0.1 A, no offset (always >= 0)
uint8_t  chgState      = 0;     // 0 DISCONNECTED,1 CONNECTED,2 READY,3 CHARGING,4 FAULT
uint8_t  chgPlug       = 0;     // 0 no plug -> drive scenario, 1 -> charge scenario
uint8_t  chgFlags      = 0x00;  // bit0 fault
uint8_t  chgAlive      = 0;


static void sendAll()
{
  uint8_t d[8];

  /* BMS frame 1: voltage + current */
  putU16LE(&d[0], bmsVoltage_dV);
  putU16LE(&d[2], encOff(bmsCurrent_dA));
  d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_BMS_1, 0, 8, d);

  /* BMS frame 2: temp + soc + state */
  d[0] = encTemp(bmsTemp_C); d[1] = bmsSoc; d[2] = bmsState;
  d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_BMS_2, 0, 8, d);

  /* BMS frame 3: flags */
  d[0] = bmsFlags; d[1] = d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_BMS_3, 0, 8, d);

  /* Motor frame 1: rpm + current */
  putU16LE(&d[0], encOff(motRpm));
  putU16LE(&d[2], encOff(motCurrent_dA));
  d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_MOT_1, 0, 8, d);

  /* Motor frame 2: temp + state + direction */
  d[0] = encTemp(motTemp_C); d[1] = motState; d[2] = motDir;
  d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_MOT_2, 0, 8, d);

  /* Motor frame 3: flags + alive */
  d[0] = motFlags; d[1] = motAlive; d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_MOT_3, 0, 8, d);

  /* Charger frame 1: voltage + current (no offset) */
  putU16LE(&d[0], chgVoltage_dV);
  putU16LE(&d[2], chgCurrent_dA);
  d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_CHG_1, 0, 8, d);

  /* Charger frame 2: state + plug */
  d[0] = chgState; d[1] = chgPlug; d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_CHG_2, 0, 8, d);

  /* Charger frame 3: flags + alive */
  d[0] = chgFlags; d[1] = chgAlive; d[2] = d[3] = d[4] = d[5] = d[6] = d[7] = 0;
  CAN.sendMsgBuf(ID_CHG_3, 0, 8, d);

  motAlive++;
  chgAlive++;
}


void setup()
{
  Serial.begin(115200);

  while (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_CRYSTAL) != CAN_OK)
  {
    Serial.println(F("MCP2515 init FAIL - check wiring / crystal"));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("MCP2515 ready @ 250 kbps"));
}

void loop()
{
  sendAll();
  delay(100);
}
