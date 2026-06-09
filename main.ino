/**
 * @brief Sistema de Monitoramento de Ilhas de Calor Urbanas — UrbanHeat
 *
 * @details
 * Nós de monitoramento instalados em postes e espaços públicos medem
 * temperatura (TMP36) e umidade relativa (potenciômetro) e respondem
 * proporcionalmente ao risco térmico detectado:
 *
 *   NORMAL  → LED verde,   válvula fechada  (0°),  buzzer off
 *   ALERTA  → LED amarelo, válvula parcial  (90°), buzzer off
 *   CRITICO → LED vermelho, válvula aberta  (180°), buzzer 2 kHz
 *   FALHA   → LED magenta, válvula fechada  (0°),  buzzer off
 *
 * O servo controla uma VÁLVULA DE ASPERSOR DE NÉVOA acoplada ao poste.
 * Aspersores de névoa são infraestrutura urbana real de mitigação de ilhas
 * de calor (utilizados em praças e calçadões em diversas cidades brasileiras).
 * Em FALHA a válvula fecha por segurança para evitar desperdício de água
 * com leitura de sensor inválida.
 *
 * Arquitetura de timers do ATmega328P:
 *   Timer0 → millis() — scheduler não-bloqueante, não conflita com nada
 *   Timer1 → biblioteca Servo — PWM contínuo da válvula
 *   Timer2 → tone() — buzzer de alerta
 *
 */

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <Servo.h>

/*---------------------------------------------------------------*/

/* PINOUT */
LiquidCrystal lcd(13, 12, 11, 10, 7, 6);

const uint8_t PIN_TEMP   = A0;
const uint8_t PIN_HUM    = A1;
const uint8_t PIN_RGB_R  = 3;
const uint8_t PIN_RGB_G  = 4;
const uint8_t PIN_RGB_B  = 5;
const uint8_t PIN_VALVULA = 9;
const uint8_t PIN_BUZZER = 8;

/*---------------------------------------------------------------*/

/* EEPROM */
const int EE_ADDR_TEMP_ERR = 0;
const int EE_ADDR_HUM_ERR  = sizeof(uint16_t);

/*---------------------------------------------------------------*/

/*
 * Limiares baseados em normas de conforto térmico urbano:
 *   35 °C → limite de conforto adaptativo em clima tropical (ABNT NBR 15575)
 *   40 °C → estresse térmico severo (OMS)
 *   40 %  → umidade mínima para conforto respiratório (ANVISA RDC 09/2003)
 *   20 %  → umidade crítica associada a episódios de ilha de calor seco
 *
 * A histerese de 1 °C / 1 % evita que atuadores oscilem quando o sensor
 * flutua em torno de um limiar.
 */
const float THR_TEMP_ALERTA  = 35.0f;
const float THR_TEMP_CRITICO = 40.0f;
const float THR_HUM_ALERTA   = 40.0f;
const float THR_HUM_CRITICO  = 20.0f;
const float HYSTERESIS       = 1.0f;

/* Faixa elétrica válida do TMP36 (~0,1 V a ~1,75 V → -40 °C a 125 °C) */
const float SENSOR_TEMP_MIN = -40.0f;
const float SENSOR_TEMP_MAX = 125.0f;

/*---------------------------------------------------------------*/

/* Cadências do scheduler */
const uint32_t INTERVAL_SENSORS = 200;   // ms — leitura e atuação
const uint32_t INTERVAL_LCD     = 2000;  // ms — alternância de página

uint32_t lastSensorTick = 0;
uint32_t lastLCDTick    = 0;

/*---------------------------------------------------------------*/

enum SystemState : uint8_t { NORMAL = 0, ALERTA, CRITICO, FALHA };

SystemState state        = NORMAL;
float       temperatura  = 0.0f;
float       umidade      = 0.0f;
bool        lcdPage      = false;

uint16_t    tempErrors   = 0;
uint16_t    humErrors    = 0;
bool        lastTempFail = false;
bool        lastHumFail  = false;

Servo valvula;

/*---------------------------------------------------------------*/

/**
 * @brief Lê os dois sensores, atualiza os valores globais e aciona a FSM.
 * Preserva o último valor válido (hold-last-value) em caso de falha pontual.
 */
void readSensors(void);
/*---------------------------------------------------------------*/
/**
 * @brief Converte a leitura do TMP36 (A0) para graus Celsius.
 * Fórmula: V = raw × 5,0/1023 → T = (V − 0,5) × 100.
 * Retorna NAN e registra erro se a tensão estiver fora da faixa do sensor.
 */
float readTemperature(void);
/*---------------------------------------------------------------*/
/**
 * @brief Converte a leitura do potenciômetro (A1) para % de umidade relativa.
 * Usa ponto flutuante ao invés de map() para preservar resolução sub-inteira.
 */
float readHumidity(void);
/*---------------------------------------------------------------*/
/**
 * @brief Avalia as leituras e transiciona o estado da FSM com histerese.
 * Subida de estado: imediata ao cruzar o limiar.
 * Descida de estado: exige cruzar (limiar − HYSTERESIS), um nível por vez.
 */
void updateState(bool tValid, bool hValid);
/*---------------------------------------------------------------*/
/**
 * @brief Aplica LED RGB, válvula de névoa e buzzer conforme o estado da FSM.
 *
 * Válvula de aspersor de névoa (servo):
 *   0°   → fechada  — temperatura nominal, sem ativação do sistema
 *   90°  → parcial  — risco emergente, névoa preventiva de baixa vazão
 *   180° → aberta   — ilha de calor ativa, névoa máxima de resfriamento
 *
 * Em FALHA a válvula fecha (0°) para evitar desperdício com sensor inválido.
 */
void applyActuators(void);
/*---------------------------------------------------------------*/
/**
 * @brief Atualiza o LCD 16×2 sem lcd.clear() para evitar flickering.
 * Página 0: leituras dos sensores. Página 1: estado da FSM e erros NVM.
 * Usa dtostrf() — única função AVR que converte float em string de forma
 * garantida em qualquer toolchain (snprintf %f falha no avr-libc padrão).
 */
void updateLCD(void);
/*---------------------------------------------------------------*/
/**
 * @brief Transmite payload JSON via UART, incluindo estado FALHA.
 * Emitir FALHA garante que o backend não interprete ausência de dado
 * como funcionamento normal.
 */
void sendJSON(void);
/*---------------------------------------------------------------*/
/**
 * @brief Define os níveis lógicos dos três canais do LED RGB.
 */
void setRGB(uint8_t r, uint8_t g, uint8_t b);
/*---------------------------------------------------------------*/
/**
 * @brief Incrementa e persiste o contador de falhas do TMP36 na EEPROM.
 * Cap em 0xFFFE para não colidir com o valor de chip virgem (0xFFFF).
 */
void registerTempError(void);
/*---------------------------------------------------------------*/
/**
 * @brief Incrementa e persiste o contador de falhas do potenciômetro na EEPROM.
 * Cap em 0xFFFE para não colidir com o valor de chip virgem (0xFFFF).
 */
void registerHumError(void);

/*---------------------------------------------------------------*/

void setup() {
  Serial.begin(9600);

  pinMode(PIN_RGB_R,  OUTPUT);
  pinMode(PIN_RGB_G,  OUTPUT);
  pinMode(PIN_RGB_B,  OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  valvula.attach(PIN_VALVULA);
  valvula.write(0);

  lcd.begin(16, 2);

  EEPROM.get(EE_ADDR_TEMP_ERR, tempErrors);
  EEPROM.get(EE_ADDR_HUM_ERR,  humErrors);
  // Chip virgem retorna 0xFFFF — sanitiza para zero
  if (tempErrors == 0xFFFF) { tempErrors = 0; EEPROM.put(EE_ADDR_TEMP_ERR, tempErrors); }
  if (humErrors  == 0xFFFF) { humErrors  = 0; EEPROM.put(EE_ADDR_HUM_ERR,  humErrors);  }

  Serial.println(F("===== URBANHEAT BOOT ====="));
  Serial.print(F("Erros TMP36 : ")); Serial.println(tempErrors);
  Serial.print(F("Erros POT   : ")); Serial.println(humErrors);
  Serial.println(F("=========================="));

  lcd.setCursor(0, 0); lcd.print(F("URBANHEAT"));
  lcd.setCursor(0, 1); lcd.print(F("Inicializando..."));
  delay(1500);
  lcd.clear();

  lastSensorTick = millis();
  lastLCDTick    = millis();
}

/*---------------------------------------------------------------*/

void loop() {
  uint32_t now = millis();

  if (now - lastSensorTick >= INTERVAL_SENSORS) {
    lastSensorTick = now;
    readSensors();
    applyActuators();
    sendJSON();
    if (!lcdPage) updateLCD(); // Atualiza dados em tempo real na página 0
  }

  if (now - lastLCDTick >= INTERVAL_LCD) {
    lastLCDTick = now;
    lcdPage = !lcdPage;
    updateLCD();
  }
}

/*---------------------------------------------------------------*/

void readSensors(void) {
  float t = readTemperature();
  float h = readHumidity();

  bool tValid = !isnan(t);
  bool hValid = !isnan(h);

  if (tValid) temperatura = t;
  if (hValid) umidade     = h;

  updateState(tValid, hValid);
}

/*---------------------------------------------------------------*/

float readTemperature(void) {
  int   raw = analogRead(PIN_TEMP);
  float v   = raw * (5.0f / 1023.0f);
  float t   = (v - 0.5f) * 100.0f;

  if (t < SENSOR_TEMP_MIN || t > SENSOR_TEMP_MAX) {
    if (!lastTempFail) registerTempError();
    lastTempFail = true;
    return NAN;
  }
  lastTempFail = false;
  return t;
}

/*---------------------------------------------------------------*/

float readHumidity(void) {
  int   raw = analogRead(PIN_HUM);
  float h   = raw * (100.0f / 1023.0f);

  if (h < 0.0f || h > 100.0f) {
    if (!lastHumFail) registerHumError();
    lastHumFail = true;
    return NAN;
  }
  lastHumFail = false;
  return h;
}

/*---------------------------------------------------------------*/

void updateState(bool tValid, bool hValid) {
  if (!tValid || !hValid) {
    state = FALHA;
    return;
  }

  bool tCrit      = temperatura >= THR_TEMP_CRITICO;
  bool tAlert     = temperatura >= THR_TEMP_ALERTA;
  bool hCrit      = umidade     <= THR_HUM_CRITICO;
  bool hAlert     = umidade     <= THR_HUM_ALERTA;
  bool tExitCrit  = temperatura <  (THR_TEMP_CRITICO - HYSTERESIS);
  bool tExitAlert = temperatura <  (THR_TEMP_ALERTA  - HYSTERESIS);
  bool hExitCrit  = umidade     >  (THR_HUM_CRITICO  + HYSTERESIS);
  bool hExitAlert = umidade     >  (THR_HUM_ALERTA   + HYSTERESIS);

  switch (state) {
    case NORMAL:
      if      (tCrit  || hCrit)  state = CRITICO;
      else if (tAlert || hAlert) state = ALERTA;
      break;

    case ALERTA:
      if      (tCrit || hCrit)           state = CRITICO;
      else if (tExitAlert && hExitAlert)  state = NORMAL;
      break;

    case CRITICO:
      if (tExitCrit && hExitCrit)
        state = (tAlert || hAlert) ? ALERTA : NORMAL;
      break;

    case FALHA:
      // Sensores voltaram a ser válidos — reavalia condição real
      state = NORMAL;
      updateState(tValid, hValid);
      break;
  }
}

/*---------------------------------------------------------------*/

void applyActuators(void) {
  switch (state) {
    case NORMAL:
      setRGB(LOW, HIGH, LOW);
      valvula.write(0);
      noTone(PIN_BUZZER);
      break;

    case ALERTA:
      setRGB(HIGH, HIGH, LOW);
      valvula.write(90);
      noTone(PIN_BUZZER);
      break;

    case CRITICO:
      setRGB(HIGH, LOW, LOW);
      valvula.write(180);
      tone(PIN_BUZZER, 2000);
      break;

    case FALHA:
      setRGB(HIGH, LOW, HIGH);
      valvula.write(0); // Fecha válvula — evita desperdício com sensor inválido
      noTone(PIN_BUZZER);
      break;
  }
}

/*---------------------------------------------------------------*/

void updateLCD(void) {
  char numBuf[8];
  char lineBuf[17];

  if (state == FALHA) {
    lcd.setCursor(0, 0); lcd.print(F("ERRO DE SENSOR  "));
    lcd.setCursor(0, 1); lcd.print(F("SISTEMA PAUSADO "));
    return;
  }

  if (!lcdPage) {
    dtostrf(temperatura, 5, 1, numBuf);
    snprintf(lineBuf, sizeof(lineBuf), "Temp:%sC  ", numBuf);
    lcd.setCursor(0, 0); lcd.print(lineBuf);

    dtostrf(umidade, 5, 1, numBuf);
    snprintf(lineBuf, sizeof(lineBuf), "Umid:%s%%  ", numBuf);
    lcd.setCursor(0, 1); lcd.print(lineBuf);
  } else {
    lcd.setCursor(0, 0);
    switch (state) {
      case NORMAL:  lcd.print(F("STATUS: NORMAL  ")); break;
      case ALERTA:  lcd.print(F("STATUS: ALERTA  ")); break;
      case CRITICO: lcd.print(F("STATUS: CRITICO ")); break;
      default: break;
    }
    snprintf(lineBuf, sizeof(lineBuf), "Erros NVM: %-4u ", (unsigned)(tempErrors + humErrors));
    lcd.setCursor(0, 1); lcd.print(lineBuf);
  }
}

/*---------------------------------------------------------------*/

void sendJSON(void) {
  Serial.print(F("{\"temp\":"));
  if (state == FALHA) {
    Serial.print(F("null,\"hum\":null"));
  } else {
    Serial.print(temperatura, 1);
    Serial.print(F(",\"hum\":"));
    Serial.print(umidade, 1);
  }
  Serial.print(F(",\"state\":\""));
  switch (state) {
    case NORMAL:  Serial.print(F("NORMAL"));  break;
    case ALERTA:  Serial.print(F("ALERTA"));  break;
    case CRITICO: Serial.print(F("CRITICO")); break;
    case FALHA:   Serial.print(F("FALHA"));   break;
  }
  Serial.print(F("\",\"errTemp\":"));
  Serial.print(tempErrors);
  Serial.print(F(",\"errHum\":"));
  Serial.print(humErrors);
  Serial.println(F("}"));
}

/*---------------------------------------------------------------*/

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  digitalWrite(PIN_RGB_R, r);
  digitalWrite(PIN_RGB_G, g);
  digitalWrite(PIN_RGB_B, b);
}

/*---------------------------------------------------------------*/

void registerTempError(void) {
  if (tempErrors < 0xFFFE) {
    tempErrors++;
    EEPROM.put(EE_ADDR_TEMP_ERR, tempErrors);
  }
  Serial.println(F("[FALHA] TMP36: fora da faixa eletrica — cabo partido ou desconectado."));
}

/*---------------------------------------------------------------*/

void registerHumError(void) {
  if (humErrors < 0xFFFE) {
    humErrors++;
    EEPROM.put(EE_ADDR_HUM_ERR, humErrors);
  }
  Serial.println(F("[FALHA] Potenciometro: fora da faixa de operacao."));
}
