/*
 * Scanner I2C + leitura continua de MPU9250 para Arduino Mega 2560.
 * SDA = pino 43, SCL = pino 44.
 *
 * O TWI por hardware do Mega fica nos pinos 20/21, entao 43/44 nao funcionam
 * com Wire.h. Este sketch faz bit-bang do protocolo, sem dependencia externa.
 *
 * Ligacao:
 *   MPU9250 VCC -> 3.3V  (NAO use 5V: a maioria dos modulos nao e 5V-tolerante)
 *   MPU9250 GND -> GND
 *   MPU9250 SDA -> pino 43
 *   MPU9250 SCL -> pino 44
 *
 * setup(): varre o barramento, identifica o chip e configura o sensor.
 * loop():  transmite accel e gyro no formato "rotulo:valor", que serve tanto
 *          para o Serial Monitor (texto legivel) quanto para o Serial Plotter
 *          (grafico com legenda). Os dois nao podem ficar abertos ao mesmo
 *          tempo na mesma porta.
 *
 * Serial Monitor / Serial Plotter em 115200 baud.
 */

const uint8_t PIN_SDA = 43;
const uint8_t PIN_SCL = 44;

/* Clock efetivo ~25-30 kHz: alem deste atraso, cada bit paga 3-4 us por
   chamada de pinMode/digitalWrite/digitalRead num AVR a 16 MHz. Lento, mas
   o I2C nao tem velocidade minima e isso ate ajuda com fios longos.
   Aumente o valor se o sinal estiver ruim. */
const uint8_t I2C_DELAY_US = 5;

const uint8_t MPU_ADDR_LOW  = 0x68;  /* AD0 em nivel baixo  */
const uint8_t MPU_ADDR_HIGH = 0x69;  /* AD0 em nivel alto   */
const uint8_t AK8963_ADDR   = 0x0C;

const uint8_t REG_WHO_AM_I     = 0x75;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;

/* Fundo de escala igual ao firmware da Pico, para os valores serem
   diretamente comparaveis: acelerometro +-4g, giroscopio +-500 dps. */
const float ACCEL_LSB_PER_G   = 8192.0f;
const float GYRO_LSB_PER_DPS  = 65.5f;
const float G_MS2             = 9.80665f;

/* Endereco onde o sensor foi efetivamente encontrado. 0 = nenhum. */
uint8_t mpuAddr = 0;

/* ── Linhas em dreno aberto ───────────────────────────────────────
   Nivel alto = pino solto como entrada com pull-up (quem puxa pra cima
   e o resistor). Nivel baixo = pino como saida em 0. A ordem importa:
   zerar o latch antes de virar saida evita um pulso alto espurio. */

static inline void sdaHigh() { pinMode(PIN_SDA, INPUT_PULLUP); }
static inline void sdaLow()  { digitalWrite(PIN_SDA, LOW); pinMode(PIN_SDA, OUTPUT); }
static inline void sclHigh() { pinMode(PIN_SCL, INPUT_PULLUP); }
static inline void sclLow()  { digitalWrite(PIN_SCL, LOW); pinMode(PIN_SCL, OUTPUT); }

static inline void i2cDelay() { delayMicroseconds(I2C_DELAY_US); }

/* Solta o clock e espera ele subir de fato (clock stretching do escravo).
   Retorna false se a linha ficar presa em baixo. */
static bool sclHighWait() {
  sclHigh();
  unsigned long t0 = micros();
  while (digitalRead(PIN_SCL) == LOW) {
    if (micros() - t0 > 2000UL) return false;
  }
  i2cDelay();
  return true;
}

static void i2cStart() {
  sdaHigh();
  sclHighWait();
  sdaLow();
  i2cDelay();
  sclLow();
  i2cDelay();
}

static void i2cStop() {
  sdaLow();
  i2cDelay();
  sclHighWait();
  sdaHigh();
  i2cDelay();
}

/* Envia um byte. Retorna true se o escravo deu ACK. */
static bool i2cWriteByte(uint8_t b) {
  for (uint8_t i = 0; i < 8; i++) {
    if (b & 0x80) sdaHigh(); else sdaLow();
    b <<= 1;
    i2cDelay();
    if (!sclHighWait()) return false;
    sclLow();
    i2cDelay();
  }

  /* Nono pulso: o escravo puxa SDA pra baixo se reconheceu. */
  sdaHigh();
  i2cDelay();
  if (!sclHighWait()) return false;
  bool ack = (digitalRead(PIN_SDA) == LOW);
  sclLow();
  i2cDelay();
  return ack;
}

static uint8_t i2cReadByte(bool sendAck) {
  uint8_t b = 0;
  sdaHigh();
  for (uint8_t i = 0; i < 8; i++) {
    b <<= 1;
    if (!sclHighWait()) return 0;
    if (digitalRead(PIN_SDA) == HIGH) b |= 1;
    sclLow();
    i2cDelay();
  }

  if (sendAck) sdaLow(); else sdaHigh();
  i2cDelay();
  sclHighWait();
  sclLow();
  i2cDelay();
  sdaHigh();
  return b;
}

/* Só endereça o dispositivo e vê se ele responde. */
static bool i2cProbe(uint8_t addr) {
  i2cStart();
  bool ack = i2cWriteByte((addr << 1) | 0);
  i2cStop();
  return ack;
}

static bool i2cWriteReg(uint8_t addr, uint8_t reg, uint8_t val) {
  i2cStart();
  if (!i2cWriteByte((addr << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(reg))             { i2cStop(); return false; }
  if (!i2cWriteByte(val))             { i2cStop(); return false; }
  i2cStop();
  return true;
}

/* Leitura em rajada: escreve o endereco do reg, repeated start, le len bytes.
   ACK em todos menos o ultimo, que leva NACK para encerrar. */
static bool i2cReadRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t len) {
  i2cStart();
  if (!i2cWriteByte((addr << 1) | 0)) { i2cStop(); return false; }
  if (!i2cWriteByte(reg))             { i2cStop(); return false; }

  i2cStart();  /* repeated start */
  if (!i2cWriteByte((addr << 1) | 1)) { i2cStop(); return false; }
  for (uint8_t i = 0; i < len; i++)
    buf[i] = i2cReadByte(i < (uint8_t)(len - 1));
  i2cStop();
  return true;
}

static void printHex(uint8_t v) {
  Serial.print("0x");
  if (v < 0x10) Serial.print('0');
  Serial.print(v, HEX);
}

static void describeWhoAmI(uint8_t who) {
  switch (who) {
    case 0x71: Serial.println(F("MPU9250  (correto)"));        break;
    case 0x73: Serial.println(F("MPU9255"));                    break;
    case 0x70: Serial.println(F("MPU6500 (sem magnetometro)")); break;
    case 0x68: Serial.println(F("MPU6050 (sem magnetometro)")); break;
    case 0x00:
    case 0xFF: Serial.println(F("linha sem resposta valida"));  break;
    default:   Serial.println(F("CI desconhecido"));            break;
  }
}

/* Tira o sensor do sleep e ajusta os fundos de escala. */
static bool mpuInit(uint8_t addr) {
  if (!i2cWriteReg(addr, REG_PWR_MGMT_1, 0x00)) return false;  /* acorda */
  delay(10);
  i2cWriteReg(addr, REG_SMPLRT_DIV,   0x04);  /* 200 Hz   */
  i2cWriteReg(addr, REG_CONFIG,       0x03);  /* DLPF 41 Hz */
  i2cWriteReg(addr, REG_GYRO_CONFIG,  0x08);  /* +-500 dps */
  i2cWriteReg(addr, REG_ACCEL_CONFIG, 0x08);  /* +-4g      */
  delay(10);
  return true;
}

static uint8_t scanBus() {
  Serial.println(F("--- varrendo 0x08 a 0x77 ---"));

  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2cProbe(addr)) {
      found++;
      Serial.print(F("  dispositivo em "));
      printHex(addr);
      if (addr == MPU_ADDR_LOW)  Serial.print(F("  (MPU9250, AD0=0)"));
      if (addr == MPU_ADDR_HIGH) Serial.print(F("  (MPU9250, AD0=1)"));
      if (addr == AK8963_ADDR)   Serial.print(F("  (AK8963, magnetometro)"));
      Serial.println();
    }
  }

  Serial.print(F("total encontrado: "));
  Serial.println(found);
  return found;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  sdaHigh();
  sclHigh();
  delay(100);

  Serial.println();
  Serial.println(F("=== MPU9250 via I2C bit-bang - Mega 2560 ==="));
  Serial.print(F("SDA = pino ")); Serial.print(PIN_SDA);
  Serial.print(F(" | SCL = pino ")); Serial.println(PIN_SCL);
  Serial.println();

  /* Linhas presas em baixo indicam curto ou dispositivo travado. */
  if (digitalRead(PIN_SDA) == LOW)
    Serial.println(F("ALERTA: SDA preso em nivel baixo (curto ou escravo travado)"));
  if (digitalRead(PIN_SCL) == LOW)
    Serial.println(F("ALERTA: SCL preso em nivel baixo (curto ou escravo travado)"));

  scanBus();
  Serial.println();

  if      (i2cProbe(MPU_ADDR_LOW))  mpuAddr = MPU_ADDR_LOW;
  else if (i2cProbe(MPU_ADDR_HIGH)) mpuAddr = MPU_ADDR_HIGH;

  if (mpuAddr == 0) {
    Serial.println(F("Nenhuma MPU encontrada. Verifique nesta ordem:"));
    Serial.println(F("  1. VCC e GND do modulo (use 3.3V, nao 5V)"));
    Serial.println(F("  2. continuidade do fio SDA ate o pino 43"));
    Serial.println(F("  3. continuidade do fio SCL ate o pino 44"));
    Serial.println(F("  4. pull-ups: a maioria dos modulos ja tem; se nao,"));
    Serial.println(F("     ponha 4.7k de SDA e SCL para o 3.3V"));
    return;
  }

  uint8_t who = 0;
  i2cReadRegs(mpuAddr, REG_WHO_AM_I, &who, 1);
  Serial.print(F("MPU em "));
  printHex(mpuAddr);
  Serial.print(F(" | WHO_AM_I = "));
  printHex(who);
  Serial.print(F(" -> "));
  describeWhoAmI(who);

  if (!mpuInit(mpuAddr)) {
    Serial.println(F("ERRO: o sensor respondeu mas nao saiu do modo sleep."));
    mpuAddr = 0;
    return;
  }

  Serial.println();
  Serial.println(F("Streaming. ax/ay/az em m/s2, gx/gy/gz em graus/s."));
  Serial.println(F("Em repouso, um dos eixos de accel deve marcar ~9.8."));
  Serial.println(F("Para o grafico: Tools -> Serial Plotter (feche o Monitor antes)."));
  Serial.println();
}

void loop() {
  if (mpuAddr == 0) {
    delay(2000);
    return;
  }

  uint8_t b[14];
  if (!i2cReadRegs(mpuAddr, REG_ACCEL_XOUT_H, b, 14)) {
    Serial.println(F("falha na leitura"));
    delay(500);
    return;
  }

  /* b[0..5] accel, b[6..7] temperatura, b[8..13] gyro */
  int16_t ax = (int16_t)((b[0]  << 8) | b[1]);
  int16_t ay = (int16_t)((b[2]  << 8) | b[3]);
  int16_t az = (int16_t)((b[4]  << 8) | b[5]);
  int16_t gx = (int16_t)((b[8]  << 8) | b[9]);
  int16_t gy = (int16_t)((b[10] << 8) | b[11]);
  int16_t gz = (int16_t)((b[12] << 8) | b[13]);

  /* "rotulo:valor" separado por virgula: legivel no Monitor, plotavel no
     Plotter. Nenhum texto extra pode entrar nesta linha. */
  Serial.print(F("ax:")); Serial.print(ax / ACCEL_LSB_PER_G * G_MS2, 2);
  Serial.print(F(",ay:")); Serial.print(ay / ACCEL_LSB_PER_G * G_MS2, 2);
  Serial.print(F(",az:")); Serial.print(az / ACCEL_LSB_PER_G * G_MS2, 2);
  Serial.print(F(",gx:")); Serial.print(gx / GYRO_LSB_PER_DPS, 2);
  Serial.print(F(",gy:")); Serial.print(gy / GYRO_LSB_PER_DPS, 2);
  Serial.print(F(",gz:")); Serial.println(gz / GYRO_LSB_PER_DPS, 2);

  delay(50);  /* 20 Hz */
}
