/*
  SLC Modbus RTU Master - Arduino Mega 2560

  Hardware:
    Mega2560 Serial1
      RX1 = D19  <- MAX485 RO
      TX1 = D18  -> MAX485 DI

    MAX485:
      DE + /RE tied together -> D2

  Modbus:
    Slave ID : 1
    Baud     : 9600
    Format   : 8N1
    Function : FC03 Read Holding Registers

  Query is intentionally limited to a small selection of values.
  Total returned register data is kept below 200 bytes per query.
  Loop interval: 10 seconds.
*/

#define RS485_DE_RE_PIN 2

#define MODBUS_SLAVE_ID 1
#define MODBUS_BAUDRATE 9600

#define QUERY_INTERVAL 10000UL
#define RESPONSE_TIMEOUT 2000UL

// -----------------------------------------------------------------------------
// Selected queries from the real SLC response.
// Each query is <= 100 registers and total response is well below 200 bytes.
//
// Diagnostic:
//   2000, 17 registers = 34 data bytes
//
// Counters L1:
//   5000, 36 registers = 72 data bytes
//
// S/L/F Components Last:
//   8022, 18 registers = 36 data bytes
//
// We therefore process each query separately.
// -----------------------------------------------------------------------------

struct QueryBlock {
  uint16_t address;
  uint16_t quantity;
  const char* name;
};

QueryBlock queries[] = {
  {2000, 17, "Diagnostic 2000-2016"},
  {5000, 36, "Counters L1 5000-5035"},
  {8022, 18, "S/L/F Components Last 8022-8039"}
};

const uint8_t QUERY_COUNT = sizeof(queries) / sizeof(queries[0]);

// -----------------------------------------------------------------------------
// Modbus CRC16
// -----------------------------------------------------------------------------

uint16_t modbusCRC(const uint8_t* data, uint16_t length) {
  uint16_t crc = 0xFFFF;

  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= data[pos];

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

// -----------------------------------------------------------------------------
// RS485 direction
// -----------------------------------------------------------------------------

void rs485ReceiveMode() {
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

void rs485TransmitMode() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
}

// -----------------------------------------------------------------------------
// Print HEX
// -----------------------------------------------------------------------------

void printHexByte(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

void printHex(const uint8_t* data, uint16_t length) {
  for (uint16_t i = 0; i < length; i++) {
    printHexByte(data[i]);
    if (i < length - 1) Serial.print(' ');
  }
}

// -----------------------------------------------------------------------------
// Send FC03 request
// -----------------------------------------------------------------------------

bool sendReadRequest(uint16_t address, uint16_t quantity) {

  uint8_t request[8];

  request[0] = MODBUS_SLAVE_ID;
  request[1] = 0x03;

  request[2] = (address >> 8) & 0xFF;
  request[3] = address & 0xFF;

  request[4] = (quantity >> 8) & 0xFF;
  request[5] = quantity & 0xFF;

  uint16_t crc = modbusCRC(request, 6);

  request[6] = crc & 0xFF;
  request[7] = (crc >> 8) & 0xFF;

  Serial.print("[TX] ");
  printHex(request, 8);
  Serial.println();

  rs485TransmitMode();

  // Give MAX485 a moment to enable transmitter.
  delayMicroseconds(100);

  Serial1.write(request, 8);
  Serial1.flush();

  // Return MAX485 to receive mode immediately after transmission.
  rs485ReceiveMode();

  return true;
}

// -----------------------------------------------------------------------------
// Read response
// -----------------------------------------------------------------------------

bool readModbusResponse(uint16_t address, uint16_t quantity) {

  // Normal FC03 response:
  // Slave + FC + ByteCount + Data + CRC
  const uint16_t expectedLength = 5 + (quantity * 2);

  // Maximum response handled here.
  // Current queries are far below this limit.
  uint8_t response[256];
  uint16_t length = 0;

  uint32_t startTime = millis();

  while (length < expectedLength) {

    if (Serial1.available()) {

      int value = Serial1.read();

      if (value >= 0) {
        if (length < sizeof(response)) {
          response[length++] = (uint8_t)value;
        } else {
          Serial.println("[ERROR] Response buffer overflow.");
          return false;
        }
      }
    }

    if (millis() - startTime >= RESPONSE_TIMEOUT) {
      Serial.println("[ERROR] Response timeout.");
      Serial.print("[RX] Received ");
      Serial.print(length);
      Serial.print("/");
      Serial.print(expectedLength);
      Serial.println(" bytes.");
      return false;
    }
  }

  Serial.print("[RX] ");
  printHex(response, length);
  Serial.println();

  // -----------------------------------------------------------
  // CRC
  // -----------------------------------------------------------

  if (length < 5) {
    Serial.println("[ERROR] Response too short.");
    return false;
  }

  uint16_t receivedCRC =
      response[length - 2] |
      ((uint16_t)response[length - 1] << 8);

  uint16_t calculatedCRC =
      modbusCRC(response, length - 2);

  if (receivedCRC != calculatedCRC) {
    Serial.println("[ERROR] CRC ERROR");

    Serial.print("       RX CRC   : 0x");
    Serial.println(receivedCRC, HEX);

    Serial.print("       CALC CRC : 0x");
    Serial.println(calculatedCRC, HEX);

    return false;
  }

  Serial.println("[OK] CRC valid.");

  // -----------------------------------------------------------
  // Slave ID
  // -----------------------------------------------------------

  if (response[0] != MODBUS_SLAVE_ID) {
    Serial.print("[ERROR] Wrong Slave ID: ");
    Serial.println(response[0]);
    return false;
  }

  // -----------------------------------------------------------
  // Modbus exception
  // -----------------------------------------------------------

  if (response[1] & 0x80) {

    uint8_t exceptionCode = response[2];

    Serial.print("[MODBUS EXCEPTION] FC=0x");
    Serial.print(response[1], HEX);
    Serial.print(" CODE=");
    Serial.println(exceptionCode);

    return false;
  }

  // -----------------------------------------------------------
  // Function code
  // -----------------------------------------------------------

  if (response[1] != 0x03) {
    Serial.print("[ERROR] Unexpected FC: 0x");
    Serial.println(response[1], HEX);
    return false;
  }

  // -----------------------------------------------------------
  // Byte count
  // -----------------------------------------------------------

  uint8_t byteCount = response[2];

  if (byteCount != quantity * 2) {
    Serial.print("[ERROR] Wrong byte count. Expected ");
    Serial.print(quantity * 2);
    Serial.print(", received ");
    Serial.println(byteCount);
    return false;
  }

  // -----------------------------------------------------------
  // Print registers
  // -----------------------------------------------------------

  Serial.println("[REGISTERS]");

  for (uint16_t i = 0; i < quantity; i++) {

    uint16_t value =
        ((uint16_t)response[3 + (i * 2)] << 8) |
        response[4 + (i * 2)];

    Serial.print("  ");
    Serial.print(address + i);
    Serial.print(" = ");
    Serial.print(value);
    Serial.print(" = 0x");

    if (value < 0x1000) Serial.print('0');
    if (value < 0x0100) Serial.print('0');
    if (value < 0x0010) Serial.print('0');

    Serial.println(value, HEX);
  }

  return true;
}

// -----------------------------------------------------------------------------
// Execute one query
// -----------------------------------------------------------------------------

void executeQuery(const QueryBlock& query) {

  Serial.println();
  Serial.println("--------------------------------------------------");

  Serial.print("[QUERY] ");
  Serial.println(query.name);

  Serial.print("[QUERY] Address = ");
  Serial.println(query.address);

  Serial.print("[QUERY] Quantity = ");
  Serial.println(query.quantity);

  Serial.print("[QUERY] Payload data = ");
  Serial.print(query.quantity * 2);
  Serial.println(" bytes");

  // Clear any stale bytes before sending.
  while (Serial1.available()) {
    Serial1.read();
  }

  sendReadRequest(query.address, query.quantity);

  readModbusResponse(query.address, query.quantity);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup() {

  pinMode(RS485_DE_RE_PIN, OUTPUT);

  // Start in receive mode.
  rs485ReceiveMode();

  Serial.begin(115200);

  Serial1.begin(MODBUS_BAUDRATE, SERIAL_8N1);

  delay(500);

  Serial.println();
  Serial.println("==================================================");
  Serial.println(" Arduino Mega 2560 - SLC MODBUS RTU MASTER");
  Serial.println("==================================================");

  Serial.println("[Serial1] RX = D19");
  Serial.println("[Serial1] TX = D18");
  Serial.println("[RS485] DE + /RE = D2");
  Serial.println("[Modbus] Slave ID = 1");
  Serial.println("[Modbus] Baudrate = 9600");
  Serial.println("[Modbus] Format = 8N1");
  Serial.println("[Modbus] Function = FC03");

  Serial.println();
  Serial.println("[QUERY] Selected parameters only.");
  Serial.println("[QUERY] Each response is below 200 bytes.");
}

// -----------------------------------------------------------------------------
// Loop
// -----------------------------------------------------------------------------

void loop() {

  for (uint8_t i = 0; i < QUERY_COUNT; i++) {

    executeQuery(queries[i]);

    // Short gap between Modbus requests.
    delay(300);
  }

  Serial.println();
  Serial.println("[WAIT] Next query cycle in 10 seconds...");

  delay(10000);
}
