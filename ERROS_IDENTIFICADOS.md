# Análise de Erros - Atuador de Rigidez Variável com FOC

## ⚠️ ERROS CRÍTICOS ENCONTRADOS

### 1. **ERRO CRÍTICO: Fórmula de Impedância Invertida** ❌
**Arquivo**: `src/impedance.cpp` - Linhas 7-8  
**Severidade**: CRÍTICA  
**Impacto**: O controle de impedância não funciona corretamente

```cpp
// ERRADO:
float torque = (imp-> K * error_theta) - (imp-> B * current_omega);
```

**Problema**: 
- A força de amortecimento está subtraindo a velocidade medida (dissipativo), mas esthe sendo feito com sinal oposto
- O termo deveria adicionar uma força oposta ao movimento para criar amortecimento

**Solução Correta**:
```cpp
// CORRETO:
float torque = (imp->K * error_theta) + (imp->B * (-current_omega));
// Ou equivalente:
float torque = (imp->K * error_theta) - (imp->B * current_omega); // MAS C não deveria estar negativa
```

**Explicação**: Em um amortecedor ideal: `τ_amort = -B * ω`
- Se ω > 0 (rotação positiva), a força deve ser negativa
- Se ω < 0 (rotação negativa), a força deve ser positiva

---

### 2. **ERRO: Race Condition - Acesso a `id_ref` e `iq_ref` Sem Proteção**
**Arquivo**: `src/mainx.cpp` - Linhas 145-152  
**Severidade**: ALTA  
**Impacto**: Leitura de valores inconsistentes

```cpp
motor.vd = compute_pi(&pi_id, (id_ref - motor.id));  // ← id_ref não tem mutex!
motor.vq = compute_pi(&pi_iq, (iq_ref - motor.iq));  // ← iq_ref não tem mutex!
```

**Problema**:
- `id_ref` e `iq_ref` são globais modificáveis
- Podem ser modificadas por outra tarefa enquanto são lidas
- Não há sincronização (mutex) protegendo esses acessos

**Solução**:
```cpp
// Adicionar proteção com mutex ou usar volatile + atomic
extern volatile float id_ref;
extern volatile float iq_ref;

// OU proteger com mutex antes de usar
if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(1)))
{
    float id_ref_copy = id_ref;
    float iq_ref_copy = iq_ref;
    xSemaphoreGive(motorMutex);
    
    motor.vd = compute_pi(&pi_id, (id_ref_copy - motor.id));
    motor.vq = compute_pi(&pi_iq, (iq_ref_copy - motor.iq));
}
```

---

### 3. **ERRO: Calculo de Velocidade com Período Assumido Fixo**
**Arquivo**: `src/mainx.cpp` - Linha 97  
**Severidade**: MÉDIA  
**Impacto**: Velocidade angular medida imprecisa

```cpp
float omega_measured = delta / 0.001f;  // ← Assume sempre 1ms!
```

**Problema**:
- O código assume que a tarefa FOC SEMPRE executa a cada exatamente 1ms
- Se houver jitter de timing ou preempção, a velocidade será incorreta
- Divisão por tempo escrito em hardcoded perde a relação com a configuração

**Solução**:
```cpp
// Opção 1: Usar variável de tempo medido
static uint32_t last_time_us = 0;
uint32_t now_us = micros();
float dt_s = (now_us - last_time_us) / 1000000.0f;
float omega_measured = delta / dt_s;
last_time_us = now_us;

// Opção 2: Padrão com período conhecido
const float DT_CONTROL = 0.001f;  // 1ms = 1000Hz
float omega_measured = delta / DT_CONTROL;
```

---

### 4. **ERRO: Buffer de Velocidade Não Inicializado (Garbage Data)**
**Arquivo**: `src/mainx.cpp` - Linhas 103-108  
**Severidade**: ALTA  
**Impacto**: Picos de velocidade aleatórios no início

```cpp
static float omega_buffer[16];  // ← Contém lixo na memória!
static int idx = 0;
omega_buffer[idx] = omega_measured;
```

**Problema**:
- Array `static` não garante inicialização zero em C++
- Os 16 primeiros ciclos terão valores pseudo-aleatórios
- Média será incorreta até encher o buffer

**Solução**:
```cpp
// Inicializar explicitamente
static float omega_buffer[16] = {0};  // ← Força zero inicial
static int idx = 0;
static bool buffer_full = false;

omega_buffer[idx] = omega_measured;
idx = (idx + 1) % 16;

// Após 16 iterações, usar a média
if (!buffer_full && idx == 0)
    buffer_full = true;

float omega_avg = 0;
for (int i = 0; i < 16; i++)
    omega_avg += omega_buffer[i];
omega_filtered = omega_avg / 16.0f;
```

---

### 5. **ERRO: Controladores PI com Ganhos Muito Pequenos**
**Arquivo**: `src/control_instances.cpp` - Linhas 8-9  
**Severidade**: ALTA  
**Impacto**: Controle lento e impreciso, pode não atingir setpoint

```cpp
PIController pi_id = {0.005f, 0.05f, 0.0f, 12.0f};   // Kp absurdamente pequeno
PIController pi_iq = {0.005f, 0.05f, 0.0f, 12.0f};
```

**Problema**:
- Kp = 0.005 é extremamente baixo para saída em volts (até 12V)
- Com erro de 1A, saída será apenas 0.005V (praticamente nada)
- O controle demorará muito para responder

**Análise**:
- Limite de saída: 12V
- Erro típico de corrente: 1A
- Com Kp=0.005 → Saída = 0.005V (ineficaz!)

**Solução**:
```cpp
// Ganhos más realistas (ajustar conforme bancada):
PIController pi_id = {5.0f, 0.5f, 0.0f, 12.0f};     // Kp = 5
PIController pi_iq = {5.0f, 0.5f, 0.0f, 12.0f};     // Kp = 5

// Com Kp=5: para erro de 1A → Saída = 5V (muito melhor)
// Ki mais baixo para facilitar sintonia
```

---

### 6. **ERRO: Falta de Sincronização na Criação do Mutex**
**Arquivo**: `src/mainx.cpp` - Linha 236  
**Severidade**: MÉDIA  
**Impacto**: Possível crash se mutex não for criado

```cpp
motorMutex = xSemaphoreCreateMutex();
// Não há verificação se retornou NULL!

// Depois...
if (xSemaphoreTake(motorMutex, 0))  // ← Pode crashear aqui!
```

**Solução**:
```cpp
motorMutex = xSemaphoreCreateMutex();
if (motorMutex == NULL)
{
    Serial.println("ERRO: Falha ao criar mutex!");
    while(1);  // Halt
}
```

---

### 7. **ERRO: Normalização de Duty Cycle Incorreta**
**Arquivo**: `src/svpwm.cpp` - Linhas 14-15  
**Severidade**: ALTA  
**Impacto**: Tensão de fase não escala corretamente com VBUS

```cpp
// ERRADO - não está normalizando corretamente
float da_norm = (t_a - v_offset) / VBUS + 0.5f;
```

**Problema**:
- `t_a` está em volts (saída de Park Inversa)
- Divisão por VBUS deveria resultar em valor -0.5 a +0.5
- Soma 0.5 resulta em 0 a 1.0 (correto), MAS...
- Depois converte para 0-100% e limita entre 5-95%

**O verdadeiro problema**: A transformação linha 14-15 não está considerando que `t_a` é uma tensão de referência que já deveria estar normalizada!

**Solução Melhor**:
```cpp
// Normalizar PRIMEIRO por VBUS/2 (amplitude do vetor)
float v_norm = 2.0f / VBUS;  // Ganho de normalização
float t_a_norm = t_a * v_norm;
float t_b_norm = t_b * v_norm;
float t_c_norm = t_c * v_norm;

// Depois centralizar e limitar
float v_max_norm = std::max({t_a_norm, t_b_norm, t_c_norm});
float v_min_norm = std::min({t_a_norm, t_b_norm, t_c_norm});
float v_offset_norm = (v_max_norm + v_min_norm) * 0.5f;

// Converter para 0-1
float da_norm = (t_a_norm - v_offset_norm) * 0.5f + 0.5f;
float db_norm = (t_b_norm - v_offset_norm) * 0.5f + 0.5f;
float dc_norm = (t_c_norm - v_offset_norm) * 0.5f + 0.5f;
```

---

### 8. **ERRO: Espaços Inconsistentes em Acesso de Struct**
**Arquivo**: `src/impedance.cpp` - Linha 7  
**Severidade**: BAIXA (cosmético, mas sintoma de falta de qualidade)  
**Impacto**: Nenhum funcional, mas reduz legibilidade

```cpp
float torque = (imp-> K * error_theta) - (imp-> B * current_omega);
              //     ↓ espaço desnecessário
```

**Solução**:
```cpp
float torque = (imp->K * error_theta) - (imp->B * current_omega);
```

---

### 9. **ERRO: Função Chamadas Sem Existir (Linhas do FOC)**
**Arquivo**: `src/mainx.cpp` - Linha 122  
**Severidade**: MÉDIA  
**Impacto**: Compilação falha

```cpp
inverse_park_clarke(&motor, sin_t, cos_t);
apply_svpwm(&motor);
```

**Problema**:
- `apply_svpwm` está definida em `svpwm.cpp`
- `inverse_park_clarke` está definida em `foc_math.cpp`
- Faltam headers correspondentes ou declarações forward

**Solução**:
- Garantir que `foc_math.h` declara `inverse_park_clarke`
- Garantir que existe `svpwm.h` com declaração de `apply_svpwm`

---

### 10. **ERRO: Impedância Não É Atualizada Durante Execução**
**Arquivo**: `src/mainx.cpp` - Linha 120  
**Severidade**: MÉDIA  
**Impacto**: Rigidez sempre fixa, sem controle variável real

```cpp
float iq_imp = compute_impedance_torque(&molaVirtual, theta_m, omega_filtered);
```

**Problema**:
- `molaVirtual` é criada em `control_instances.cpp` com valores fixos
- Não há função para modificar K, B, theta_set durante execução
- O "atuador de rigidez variável" não pode variar rigidez!

**Solução**:
```cpp
// Adicionar função para atualizar impedância
void update_impedance(float new_K, float new_B, float new_theta_set)
{
    if (xSemaphoreTake(motorMutex, pdMS_TO_TICKS(10)))
    {
        molaVirtual.K = new_K;
        molaVirtual.B = new_B;
        molaVirtual.theta_set = new_theta_set;
        xSemaphoreGive(motorMutex);
    }
}

// Permitir comunicação (ex: Serial, CAN, etc) para receber novos valores
```

---

## 📋 SUMÁRIO DOS ERROS

| # | Erro | Arquivo | Severidade | Impacto |
|---|------|---------|-----------|---------|
| 1 | Fórmula impedância | impedance.cpp | 🔴 CRÍTICA | Amortecimento invertido |
| 2 | Race condition id/iq_ref | mainx.cpp | 🟠 ALTA | Valores inconsistentes |
| 3 | Velocidade período fixo | mainx.cpp | 🟡 MÉDIA | Medição imprecisa |
| 4 | Buffer não inicializado | mainx.cpp | 🟠 ALTA | Garbage data |
| 5 | Ganhos PI muito baixos | control_instances.cpp | 🟠 ALTA | Controle lento |
| 6 | Mutex sem verificação | mainx.cpp | 🟡 MÉDIA | Possível crash |
| 7 | Normalização PWM | svpwm.cpp | 🟠 ALTA | Voltagem incorreta |
| 8 | Espaços extra (cosmético) | impedance.cpp | 🟢 BAIXA | Só legibilidade |
| 9 | Includes faltando | mainx.cpp | 🟡 MÉDIA | Compilação falha |
| 10 | Impedância fixa | mainx.cpp | 🟡 MÉDIA | Sem variação real |

---

## 🔧 RECOMENDAÇÕES DE PRIORIDADE

1. **Primeiro**: Corrigir fórmula de impedância (#1) - Sem isso, tudo falha
2. **Segundo**: Ajustar ganhos PI (#5) - Motor não vai responder
3. **Terceiro**: Sincronização id/iq_ref (#2) - Race conditions
4. **Quarto**: Normalização SVPWM (#7) - Voltagens erradas
5. **Quinto**: Inicialização buffer (#4) - Estabilidade
6. **Sexto**: Adicionar capacidade de variação de impedância (#10) - Funcionalidade

---

## ✅ TESTES RECOMENDADOS

Após corrigir os erros:

1. **Teste de Calibração**: Verificar se offset elétrico é correto
2. **Teste de PI**: Validar resposta do controlador com degrau de corrente
3. **Teste de Impedância**: Aplicar força externa e verificar rigidez
4. **Teste de Amortecimento**: Verificar se força opõe movimento
5. **Teste de Taxa de Controle**: Confirmar que FOC roda a 1kHz
