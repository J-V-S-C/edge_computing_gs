# README: Sistema UrbanHeat

## Esquema de Ligação
Componentes conectados ao Arduino Uno:

| Componente | Pino Arduino | Descrição |
| :--- | :--- | :--- |
| **LCD (RS, E, D4-D7)** | 13, 12, 11, 10, 7, 6 | Interface visual (16x2) |
| **Servo Válvula** | 9 | Controle PWM (0-180°) |
| **Buzzer** | 8 | Alerta sonoro (2 kHz) |
| **LED RGB (R, G, B)** | 3, 4, 5 | Indicador de estado |
| **TMP36 (Temp)** | A0 | Sensor analógico |
| **Potenciômetro (Hum)** | A1 | Simulação de umidade |

## Instruções de Operação
1. **Monitoramento:** O sistema lê sensores a cada 200ms.
2. **Estados:**
    * **Normal:** LED Verde, Válvula 0°.
    * **Alerta:** LED Amarelo, Válvula 90°.
    * **Crítico:** LED Vermelho, Válvula 180°, Buzzer ligado.
    * **Falha:** LED Magenta, Válvula 0° (segurança).
3. **Interface:** O LCD alterna automaticamente entre dados dos sensores e status/contagem de falhas (persistidas na EEPROM).

## Código Fonte
* **Arquitetura:** FSM (Máquina de Estados Finitos) sem uso de `delay()`.
* **Temporização:** Baseada em `millis()` para execução não-bloqueante.
* **Confiabilidade:** Histerese implementada para evitar oscilação de atuadores; persistência de erros na EEPROM para diagnóstico.
