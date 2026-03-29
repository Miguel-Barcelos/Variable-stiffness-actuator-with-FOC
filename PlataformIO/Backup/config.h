// Este arquivo centraliza todas as configurações de hardware e parâmetros globais do sistema.

// Proteção contra inclusão múltipla
#ifndef CONFIG_H 
#define CONFIG_H

// Definições de Hardware
#define EN_GATE 14
#define IA_PIN 32
#define IB_PIN 33
#define PWM_U 21 // INH_A
#define PWM_V 22 // INH_B
#define PWM_W 23 // INH_C
#define SDA_PIN 25
#define SCL_PIN 26
#define OC_ADJ 13
#define DRIVER_FAULT_PIN 34 // Pino para leitura de falhas do driver

// Parâmetros do Sistema
#define F_CONTROL 10000
#define PARES_POLOS 7
#define VBUS 12.0f

#endif