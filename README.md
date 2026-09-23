# Trabalho RTS - Drone ESP32

Trabalho da disciplina de Sistemas em Tempo Real.

- Universidade: UNIVALI
- Curso: Engenharia de Computação
- Aluno: Rafael Kraemer
- Professor: Felipe Viel
- Plataforma: ESP32
- Framework: ESP-IDF 6.1 / FreeRTOS

## Funcionalidades

- FUS_IMU periódica
- CTRL_ATT encadeada
- NAV_PLAN acionada por Touch B
- Telemetria acionada por Touch C
- Fail-safe acionado por interrupção no Touch D
- Comparação entre RM, DM e política Custom
- Testes preemptivos e cooperativos
- Frequências de 80, 160 e 240 MHz
- Medição de jitter, latência, WCET e deadline misses
