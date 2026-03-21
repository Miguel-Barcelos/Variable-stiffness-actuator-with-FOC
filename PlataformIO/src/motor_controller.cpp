#include "motor_controller.h"
#include "config.h"
#include <driver/mcpwm.h>

// Construtor
MotorController::MotorController() 
    : state(MC_STATE_INIT), data_mutex(NULL), electrical_offset(0.0f), foc_counter(0) {
    data_mutex = xSemaphoreCreateMutex();
    if (data_mutex == NULL) {
        // Erro crítico - não conseguiu criar mutex
        state = MC_STATE_ERROR;
    }
}

// Destrutor
MotorController::~MotorController() {
    if (data_mutex) {
        vSemaphoreDelete(data_mutex);
    }
}

// Inicialização
bool MotorController::init(const MotorControllerConfig& cfg) {
    if (state != MC_STATE_INIT) return false;
    
    config = cfg;
    
    // Inicializar MCPWM
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_U);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_V);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, PWM_W);
    
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 20000;
    pwm_config.cmpr_a = 0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);
    
    // Duty inicial zero
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 0);
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, 0);
    
    // Configurar ADC
    analogReadResolution(12);
    
    // Configurar pinos
    pinMode(EN_GATE, OUTPUT);
    digitalWrite(EN_GATE, LOW);
    pinMode(DRIVER_FAULT_PIN, INPUT_PULLUP);
    
    state = MC_STATE_CALIBRATION;
    return true;
}

// Controle de estado
void MotorController::setState(MotorControllerState new_state) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        state = new_state;
        xSemaphoreGive(data_mutex);
    }
}

// Controle de impedância
void MotorController::setImpedance(const VirtualImpedance& imp) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        impedance = imp;
        xSemaphoreGive(data_mutex);
    }
}

void MotorController::updateImpedance(float k, float b, float theta_set) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        impedance.K = k;
        impedance.B = b;
        impedance.theta_set = theta_set;
        xSemaphoreGive(data_mutex);
    }
}

// Leitura de sensores (thread-safe)
void MotorController::readSensors() {
    uint16_t raw_angle = readRawAngle();
    float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;
    
    float ia = (analogRead(IA_PIN) - 2048.0f) * 0.80488f / 1000.0f;
    float ib = (analogRead(IB_PIN) - 2048.0f) * 0.80488f / 1000.0f;
    float ic = -(ia + ib);
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        motor_data.theta_m = theta_m;
        motor_data.ia = ia;
        motor_data.ib = ib;
        motor_data.ic = ic;
        xSemaphoreGive(data_mutex);
    }
}

// Cálculo do ângulo elétrico
void MotorController::calculateElectricalAngle() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        float theta_e = (motor_data.theta_m * config.pole_pairs) - electrical_offset;
        theta_e = fmodf(theta_e, 2.0f * PI);
        if (theta_e < 0) theta_e += 2.0f * PI;
        motor_data.theta_e = theta_e;
        xSemaphoreGive(data_mutex);
    }
}

// Cálculo de velocidade com filtro simples
void MotorController::calculateSpeed() {
    static float theta_prev = 0.0f;
    static float omega_filtered = 0.0f;
    
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        float omega_raw = (motor_data.theta_e - theta_prev) / 0.001f;
        // Filtro passa-baixa simples (alpha = 0.1)
        omega_filtered = 0.9f * omega_filtered + 0.1f * omega_raw;
        
        motor_data.omega_measured = omega_filtered;
        theta_prev = motor_data.theta_e;
        xSemaphoreGive(data_mutex);
    }
}

// Controle de impedância
void MotorController::updateImpedanceControl() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        float torque = compute_impedance_torque(&impedance, motor_data.theta_m, motor_data.omega_measured);
        
        // Limitação de segurança
        if (torque > config.current_limit) torque = config.current_limit;
        if (torque < -config.current_limit) torque = -config.current_limit;
        
        motor_data.iq_ref = torque;
        xSemaphoreGive(data_mutex);
    }
}

// Controle de corrente
void MotorController::applyCurrentControl() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        // Transformadas
        clarke_transform(&motor_data);
        float sin_t = sin(motor_data.theta_e);
        float cos_t = cos(motor_data.theta_e);
        park_transform(&motor_data, sin_t, cos_t);
        
        // Controle PI
        motor_data.vd = compute_pi(&config.pi_id, (motor_data.id_ref - motor_data.id));
        motor_data.vq = compute_pi(&config.pi_iq, (motor_data.iq_ref - motor_data.iq));
        
        // Transformada inversa
        inverse_park_clarke(&motor_data, sin_t, cos_t);
        
        xSemaphoreGive(data_mutex);
    }
}

// Modulação
void MotorController::applyModulation() {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(1))) {
        apply_svpwm(&motor_data);
        xSemaphoreGive(data_mutex);
    }
}

// Verificação de falhas
void MotorController::checkFaults() {
    bool fault = (digitalRead(DRIVER_FAULT_PIN) == LOW);
    if (fault) {
        setState(MC_STATE_FAULT);
        enableDriver(false);
    }
}

// Loop FOC principal
void MotorController::runFocLoop() {
    if (state != MC_STATE_RUNNING) return;
    
    readSensors();
    calculateElectricalAngle();
    calculateSpeed();
    updateImpedanceControl();
    applyCurrentControl();
    applyModulation();
    checkFaults();
    
    foc_counter++;
}

// Calibração
bool MotorController::calibrateElectricalOffset() {
    if (state != MC_STATE_CALIBRATION) return false;
    
    // Aplicar tensão para alinhar rotor
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 50.0f);
    enableDriver(true);
    delay(2000);
    
    uint16_t raw_angle = readRawAngle();
    float theta_m = (raw_angle / 4096.0f) * 2.0f * PI;
    electrical_offset = theta_m * config.pole_pairs;
    
    // Desligar
    mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 0.0f);
    enableDriver(false);
    
    state = MC_STATE_RUNNING;
    return true;
}

// Acesso a dados
MotorVars MotorController::getMotorData() {
    MotorVars data;
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        data = motor_data;
        xSemaphoreGive(data_mutex);
    }
    return data;
}

void MotorController::setReference(float id_ref, float iq_ref) {
    if (xSemaphoreTake(data_mutex, pdMS_TO_TICKS(10))) {
        motor_data.id_ref = id_ref;
        motor_data.iq_ref = iq_ref;
        xSemaphoreGive(data_mutex);
    }
}

// Utilitários
void MotorController::enableDriver(bool enable) {
    digitalWrite(EN_GATE, enable ? HIGH : LOW);
}

bool MotorController::isDriverEnabled() const {
    return digitalRead(EN_GATE) == HIGH;
}