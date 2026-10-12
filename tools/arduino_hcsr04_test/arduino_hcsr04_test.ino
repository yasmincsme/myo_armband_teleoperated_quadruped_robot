/*
 * Teste de sonar HC-SR04 para Arduino Mega 2560.
 *
 * Ligacao:
 *   HC-SR04 VCC  -> 5V   (o modulo e 5V; nao funciona bem em 3.3V)
 *   HC-SR04 GND  -> GND
 *   HC-SR04 TRIG -> pino 9
 *   HC-SR04 ECHO -> pino 10
 *
 * ATENCAO ao usar o mesmo modulo na Raspberry Pi Pico: o ECHO entrega 5V e
 * os GPIOs do RP2350 nao sao tolerantes a 5V. Para a Pico e obrigatorio um
 * divisor resistivo no ECHO (ex.: 1k em serie + 2k para o GND, entregando
 * ~3.3V). No Mega isso nao e necessario.
 *
 * Serial Monitor em 115200 baud.
 */

const uint8_t PIN_TRIG = 9;
const uint8_t PIN_ECHO = 10;

/* Alcance do sensor: 2 cm a ~400 cm. Ida e volta a 343 m/s, 400 cm da
   ~23300 us; 30000 us de teto cobre o alcance todo com folga. */
const unsigned long ECHO_TIMEOUT_US = 30000UL;

/* O datasheet pede >=60 ms entre disparos para o eco anterior se dissipar. */
const unsigned long PING_INTERVAL_MS = 60;

/* Quantas medidas entram na mediana. Impar, para ter elemento central. */
const uint8_t SAMPLES = 5;

/* Uma medida crua. Retorna a largura do pulso em us, ou 0 se nao houve eco. */
static unsigned long readEchoUs() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  return pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
}

/* Mediana e melhor que media aqui: descarta leituras espurias isoladas sem
   deixar que elas puxem o resultado. */
static unsigned long medianEchoUs() {
  unsigned long v[SAMPLES];

  for (uint8_t i = 0; i < SAMPLES; i++) {
    v[i] = readEchoUs();
    delay(PING_INTERVAL_MS);
  }

  for (uint8_t i = 1; i < SAMPLES; i++) {
    unsigned long key = v[i];
    int8_t j = i - 1;
    while (j >= 0 && v[j] > key) {
      v[j + 1] = v[j];
      j--;
    }
    v[j + 1] = key;
  }

  return v[SAMPLES / 2];
}

static void checkStuckLines() {
  /* ECHO em alto permanente, sem disparo nenhum, indica fio trocado com o
     TRIG, modulo travado ou alimentacao errada. */
  if (digitalRead(PIN_ECHO) == HIGH) {
    Serial.println(F("ALERTA: ECHO esta em nivel alto em repouso."));
    Serial.println(F("        Verifique se TRIG e ECHO nao estao trocados"));
    Serial.println(F("        e se o VCC do modulo esta em 5V."));
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  delay(100);

  Serial.println();
  Serial.println(F("=== Teste HC-SR04 - Mega 2560 ==="));
  Serial.print(F("TRIG = pino ")); Serial.print(PIN_TRIG);
  Serial.print(F(" | ECHO = pino ")); Serial.println(PIN_ECHO);
  Serial.println();

  checkStuckLines();

  Serial.println(F("us\tcm\testado"));
  Serial.println(F("------------------------------"));
}

void loop() {
  unsigned long us = medianEchoUs();

  Serial.print(us);
  Serial.print('\t');

  if (us == 0) {
    Serial.print(F("--\t"));
    Serial.println(F("sem eco (fora de alcance, superficie absorvente ou fio solto)"));
  } else {
    /* 343 m/s a 20 C; o pulso faz ida e volta, dai a divisao por 2.
       0.0343 cm/us / 2 = 0.01715 cm/us  */
    float cm = us * 0.01715f;

    Serial.print(cm, 1);
    Serial.print('\t');

    if (cm < 2.0f)
      Serial.println(F("abaixo do minimo do sensor (2 cm) - leitura nao confiavel"));
    else if (cm > 400.0f)
      Serial.println(F("acima do alcance nominal (400 cm)"));
    else
      Serial.println(F("ok"));
  }

  delay(300);
}
