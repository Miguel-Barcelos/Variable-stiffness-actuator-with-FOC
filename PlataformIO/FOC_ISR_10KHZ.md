# Migração FOC para ISR 10 kHz - Guia Completo

## 🎯 Objetivo
Aumentar frequência de controle FOC de **1 kHz → 10 kHz** usando ISR (Interrupt Service Routine) em vez de task.

## ⚡ Por Que ISR em Vez de Task?

### Task (1 kHz - ANTES)
```
FreeRTOS Scheduler
    ↓
Verifica tarefas prontas
    ↓
Executa FOC a cada 1ms
    ↓
Problema: Período mínimo cerca de 1ms, jitter de scheduling
```

### ISR (10 kHz - DEPOIS)
```
Hardware Timer
    ↓
Interrupção exata a cada 100 µs
    ↓
Executa FOC imediatamente
    ↓
Vantagem: Frequência precisa, sem jitter, prioridade máxima
```

---

## ⚠️ Cuidados Críticos com ISR

### 1. **ISR Deve Ser RÁPIDA (10 kHz = 100 µs por ciclo)**
```
Alocação máxima: 100 µs / ciclo
Recomendado: < 50 µs (deixa margem)

Se ISR demorar > 100 µs:
❌ Perde ciclos
❌ Instabilidade total
```

### 2. **NÃO Usar FreeRTOS Primitives** (mutex, delay, etc)
```cpp
// ❌ ERRADO em ISR:
xSemaphoreTake(motorMutex, 0);          // Pode bloquear
vTaskDelay(1);                           // CRASH
xQueueSend(...);                         // Comportamento indefinido

// ✅ CORRETO em ISR:
volatile float x = value;                // Variáveis volátiles
portDISABLE_INTERRUPTS();                // Desabilita interrupções
// ... código crítico crítico ...
portENABLE_INTERRUPTS();                 // Reabilita
```

### 3. **Dados Compartilhados Devem Ser Volátiles**
```cpp
// ✅ Declaração correta para ISR
volatile MotorVars motor;           // ISR escreve, task lê
volatile float omega_filtered;      // Volátile garante reload
volatile uint32_t foc_tick;         // Contador de ciclos

// ❌ ERRADO (pode ser otimizado fora):
float motor_state;                  // Compilador pode cachear
```

---

## 🏗️ Arquitetura Nova

```
setup()
    ↓
┌─────────────────────────────────────────┐
│ TIMER0 Hardware (ESP32)                 │
│ Período: 100 µs (10 kHz)               │
└─────────────────────────────────────────┘
    ↓ (Interrupção)
┌─────────────────────────────────────────┐
│ ISR: foc_isr()                         │
│ - Lê encoder                           │
│ - Transformadas Clarke/Park            │
│ - PI Controllers                       │
│ - SVPWM                                │
│ - Atualiza estruturas volátiles        │
│ Tempo: ~30-50 µs                       │
└─────────────────────────────────────────┘
    ↓ (Sincronização via volátiles)
┌─────────────────────────────────────────┐
│ Task: debugTask (100 Hz - 10 ms)       │
│ - Lê dados volátiles (atomic)          │
│ - Print Serial (lento)                 │
│ - Monitora FAULT                       │
└─────────────────────────────────────────┘

┌─────────────────────────────────────────┐
│ Task: calibrationTask (executa uma vez)│
│ - Alinha rotor                         │
│ - Calcula offset                       │
└─────────────────────────────────────────┘
```

---

## 🔄 Fluxo de Sincronização

### Dados Compartilhados Entre ISR e Tasks
```cpp
// Globais volátiles
volatile MotorVars motor;               // ISR escreve, tasks leem
volatile float omega_filtered;          // Atualizado a cada ciclo ISR
volatile uint32_t foc_tick;             // Contador de ciclos

// Não volátil (somente leitura de controle)
float id_ref = 0;                       // Setado por comunicação
float iq_ref = 0;                       // Setado por comunicação
VirtualImpedance molaVirtual;           // Setado por comunicação
```

### Sincronização ISR ↔ Task
```
ISR (10 kHz - 100 µs)
│
├─ Lê encoder
├─ Lê ADCs (correntes)
├─ Calcula transformadas
├─ Calcula PI
├─ Atualiza motor (volatile)
├─ foc_tick++
└─ Retorna (~30-50 µs)

       ↓↓↓ Dados volátiles são lidos por tasks ↓↓↓

debugTask (100 Hz - 10 ms)
│
├─ Cria cópia local de motor
├─ Printa Serial (lento)
└─ Controla FAULT
```

---

## 📋 Implementação Passo a Passo

### Passo 1: Instância para Copiar Dados (Atomicamente)
```cpp
// Helper para ler motor atomicamente (sem ISR atrapalhar)
MotorVars motor_read_snapshot()
{
    MotorVars temp;
    portDISABLE_INTERRUPTS();        // Desabilita ISR temporariamente
    temp = motor;                    // Cópia completa
    portENABLE_INTERRUPTS();         // Reabilita ISR
    return temp;                     // Retorna cópia segura
}
```

### Passo 2: Configuração do Timer
```cpp
// No setup()
void setup_timer_foc()
{
    // TIMER_GROUP_0, TIMER_0, 10 kHz (100 µs)
    timer_config_t config;
    config.divider = 8;              // 80MHz / 8 = 10MHz
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = true;
    config.alarm_value = 1000;       // 1000 * 0.1µs = 100µs = 10kHz
    
    timer_init(TIMER_GROUP_0, TIMER_0, &config);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, 1000);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, &foc_isr, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(TIMER_GROUP_0, TIMER_0);
}
```

### Passo 3: ISR FOC
```cpp
void IRAM_ATTR foc_isr(void* arg)
{
    TIMERG0.int_clr_timers.t0 = 1;   // Limpar flag de interrupção
    
    if (!driver_enabled) return;      // Se desabilitado, pula
    
    // Leitura do encoder
    uint16_t rawAngle = readRawAngle();
    float thetaMec = (rawAngle / 4096.0f) * 2.0f * PI;
    
    // ... resto do controle FOC ...
    
    // Atualizar dados volátiles
    motor.thetaM = thetaMec;
    motor.iq = resultado_iq;
    foc_tick++;
}
```

---

## 🧪 Testes Recomendados

### 1. Verificar Frequência Real
```cpp
// Monitorar foc_tick
Serial.println(foc_tick);
delay(1000);
Serial.println(foc_tick);
// Deve mostrar incremento de ~10000 por segundo
```

### 2. Monitorar Tempo de ISR
```cpp
// Usar pino GPIO para medir duração da ISR
// digitalWrite(GPIO, HIGH);  // Início ISR
// ... código ...
// digitalWrite(GPIO, LOW);   // Fim ISR
// Medir com osciloscópio: deve ser < 50µs
```

### 3. Verificar Estabilidade
- Temperatura do motor (não deve aquecer)
- Resposta de impedância (deve ser mais suave)
- Serial output (não deve ter glitches)

---

## 📊 Performance Esperada

| Métrica | Antes (1 kHz) | Depois (10 kHz) | Ganho |
|---------|---------------|-----------------|--------|
| **Frequência** | 1000 Hz | 10000 Hz | 10x |
| **Período** | 1 ms | 100 µs | 10x |
| **Jitter** | ~0.1 ms | ~0.1 µs | 1000x |
| **Resposta** | Mais lenta | Mais rápida | Suavidade |
| **Resolução** | Baixa | Alta | Mais preciso |

---

## ⚠️ Possíveis Problemas

### Problema: "ISR muito lenta, não consegue 10 kHz"
**Sintoma:** `foc_tick` não chega a 10k/s
**Causa:** Operações lentas na ISR (especialmente I2C do encoder)
**Solução:** Usar buffer de leitura anterior ao invés de ler a cada ciclo

### Problema: "Sistema fica instável"
**Sintoma:** Motor oscila, perde controle
**Causa:** ISR interferindo com outras operações
**Solução:** Reduzir frequência para 5 kHz (200 µs), usar `IRAM_ATTR`

### Problema: "Dados incorretos no Serial"
**Sintoma:** Valores aleatórios, valores muito grandes
**Causa:** Race condition entre ISR e task
**Solução:** Usar `portDISABLE_INTERRUPTS()` ao ler

---

## ✅ Checklist de Implementação

- [ ] Timer configurado para 10 kHz
- [ ] ISR implementada com `IRAM_ATTR`
- [ ] Todas as variáveis compartilhadas são `volatile`
- [ ] Sem chamadas FreeRTOS em ISR
- [ ] Helper `motor_read_snapshot()` implementado
- [ ] debugTask usa snapshot para ler dados
- [ ] foc_tick incrementa corretamente
- [ ] Tempo de ISR < 50 µs (medido com GPIO)
- [ ] Motor responde corretamente
- [ ] Serial output estável
- [ ] Testado em bancada com perturbações

---

## 🎓 Notas Importantes

1. **IRAM_ATTR**: Coloca ISR na RAM interna (mais rápida que flash)
2. **volatile**: Força o compilador a ler/escrever sempre (não cacheia)
3. **portDISABLE/ENABLE_INTERRUPTS**: Cria seção crítica segura
4. **Frequência máxima**: Depende do tempo de ISR + overhead

Recomendação: **Coce começar em 5 kHz (200 µs)** e aumentar conforme necessário.

