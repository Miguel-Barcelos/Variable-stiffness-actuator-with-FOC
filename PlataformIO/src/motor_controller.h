#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "foc_math.h"
#include "pi_control.h"
#include "impedance.h"
#include "encoder_utils.h"
#include "svpwm.h"

// Estados do controlador
typedef enum {
    MC_STATE_INIT,
    MC_STATE_CALIBRATION,
    MC_STATE_RUNNING,
    MC_STATE_ERROR,
    MC_STATE_FAULT
} MotorControllerState;

// Configuração do controlador
typedef struct {
    float voltage_bus;      // Tensão de barramento (V)
    float current_limit;    // Limite de corrente (A)
    uint8_t pole_pairs;     // Pares de polos
    float max_speed;        // Velocidade máxima (rad/s)
    PIController pi_id;     // PI para corrente d
    PIController pi_iq;     // PI para corrente q
    PIController pi_omega;  // PI para velocidade (se usado)
} MotorControllerConfig;

// Classe principal do controlador de motor
class MotorController {
private:
    MotorVars motor_data;
    MotorControllerState state;
    SemaphoreHandle_t data_mutex;
    MotorControllerConfig config;
    VirtualImpedance impedance;
    float electrical_offset;
    uint32_t foc_counter;

    // Métodos privados
    void readSensors();
    void calculateElectricalAngle();
    void calculateSpeed();
    void updateImpedanceControl();
    void applyCurrentControl();
    void applyModulation();
    void checkFaults();

public:
    MotorController();
    ~MotorController();

    // Inicialização
    bool init(const MotorControllerConfig& cfg);
    
    // Controle de estado
    void setState(MotorControllerState new_state);
    MotorControllerState getState() const { return state; }
    
    // Controle de impedância
    void setImpedance(const VirtualImpedance& imp);
    void updateImpedance(float k, float b, float theta_set);
    
    // Execução do loop FOC
    void runFocLoop();
    
    // Calibração
    bool calibrateElectricalOffset();
    
    // Acesso a dados (thread-safe)
    MotorVars getMotorData();
    void setReference(float id_ref, float iq_ref);
    
    // Utilitários
    void enableDriver(bool enable);
    bool isDriverEnabled() const;
};

#endif // MOTOR_CONTROLLER_H