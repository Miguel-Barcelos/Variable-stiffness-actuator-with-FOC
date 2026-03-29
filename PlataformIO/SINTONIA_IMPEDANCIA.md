# Estratégia de Sintonia para Controle de Impedância - Mola Virtual

## 🎯 Diagnóstico do Problema Atual

**Sintomas observados:**
- Motor retorna agressivamente quando perturbado
- Overshoot excessivo (ultrapassa a posição de equilíbrio)
- Possível oscilação/zumbido

**Causa raiz:**
```
Razão B/K = 0.2 / 2.0 = 0.1 (MUITO BAIXO!)
```

Para uma resposta sem overshoot (amortecimento crítico), você precisa:
```
ζ = B / (2*√(K*I)) ≥ 1  (amortecimento crítico ou super-amortecimento)
```

Com K=2.0, I≈0.01 (inércia típica do motor):
```
B_mínimo = 2 * √(2.0 * 0.01) = 2 * √0.02 = 2 * 0.1414 ≈ 0.283
```

**Valor atual B=0.2 < 0.283 → SUBAMORTECIDO (oscila!)** 🔴

---

## 📊 Tabela de Configurações Recomendadas

### Configuração 1: Conservadora (Recomendada para COMEÇAR)
**Para resposta lenta mas muito estável**

```cpp
// IMPEDÂNCIA
VirtualImpedance molaVirtual = {2.0f, 1.0f, 0.0f};
// K = 2.0 N.m/rad  (rigidez moderada)
// B = 1.0 N.m.s/rad (amortecimento CRÍTICO)

// CONTROLADOR PI (corrente)
PIController pi_id = {2.0f, 0.2f, 0.0f, 12.0f};
PIController pi_iq = {2.0f, 0.2f, 0.0f, 12.0f};
```

**Características:**
- ✅ Sem overshoot
- ✅ Resposta amortecida (não oscila)
- ⚠️ Resposta mais lenta (~500ms para acomodar)
- ✅ SEGURO para comprovar o comportamento base

---

### Configuração 2: Moderada (Recomendada para OPERAÇÃO)
**Para resposta rápida com estabilidade**

```cpp
// IMPEDÂNCIA
VirtualImpedance molaVirtual = {3.0f, 1.2f, 0.0f};
// K = 3.0 N.m/rad  (rigidez um pouco maior)
// B = 1.2 N.m.s/rad (amortecimento bem calculado)

// CONTROLADOR PI (corrente)
PIController pi_id = {3.0f, 0.3f, 0.0f, 12.0f};
PIController pi_iq = {3.0f, 0.3f, 0.0f, 12.0f};
```

**Características:**
- ✅ Overshoot pequeno (~5-10%)
- ✅ Resposta rápida (~200-300ms)
- ✅ Estável e previsível
- ✅ BOM para aplicações reais

---

### Configuração 3: Agressiva (Para Máxima Resposta)
**Se precisar de resposta MUITO rápida**

```cpp
// IMPEDÂNCIA
VirtualImpedance molaVirtual = {5.0f, 2.0f, 0.0f};
// K = 5.0 N.m/rad  (rigidez alta)
// B = 2.0 N.m.s/rad (amortecimento proporcional)

// CONTROLADOR PI (corrente)
PIController pi_id = {5.0f, 0.5f, 0.0f, 12.0f};
PIController pi_iq = {5.0f, 0.5f, 0.0f, 12.0f};
```

**Características:**
- ⚠️ Overshoot moderado (~15-20%)
- ✅ Resposta muito rápida (~100ms)
- ⚠️ Mais sensível a perturbações
- ⚖️ Trade-off entre velocidade e estabilidade

---

## 🔧 Fórmula de Sintonia Prática

Para qualquer configuração, mantenha a **relação de amortecimento**:

```
B = K * damping_ratio

Onde damping_ratio:
  0.7 - 0.8  → Subamortecido (pequeno overshoot, rápido)
  1.0 - 1.2  → Amortecimento crítico (melhor compromisso)
  1.5 - 2.0  → Super-amortecido (sem overshoot, mais lento)
```

**Exemplo prático:**
```cpp
float K = 3.0f;
float damping_ratio = 1.0f;  // Ajuste este valor
float B = K * damping_ratio;  // B = 3.0

VirtualImpedance molaVirtual = {K, B, 0.0f};
```

---

## 🎮 Método de Sintonia Empírica (Recomendado)

### Passo 1: Testar Configuração Base
```cpp
// Comece com Configuração 1 (Conservadora)
VirtualImpedance molaVirtual = {2.0f, 1.0f, 0.0f};
PIController pi_iq = {2.0f, 0.2f, 0.0f, 12.0f};
```

### Passo 2: Teste no Hardware
Abra o Serial Monitor (115200 baud) e observe:
- Tick (iteração do FOC)
- Theta_m (posição mecânica)
- Omega (velocidade)
- Iq (corrente de torque)

**Teste**: Aplique um deslocamento manual de ~0.5 rad (±90°) e solte

**O que você deve ver**:
```
Tempo:  0ms    → Theta salta para 0.5 rad (perturbação)
Tempo: 50ms    → Theta começa a retornar (força de mola)
Tempo: 200ms   → Theta volta para 0 (ou PERTO)
Tempo: 250ms   → Oscilações residuais MÍNIMAS ou nenhuma
```

### Passo 3: Avaliar e Ajustar

**Se ainda oscila muito:**
- ❌ B é baixo demais
- ✅ **Ação**: Aumentar B em +0.2 a cada teste
```cpp
B = 1.0 → 1.2 → 1.4 → 1.6 (até estabilizar)
```

**Se responde muito lentamente:**
- ❌ B é alto demais (super-amortecido)
- ✅ **Ação**: Diminuir B ou aumentar K
```cpp
// Opção A: Reduzir B
B = 1.0 → 0.9 → 0.8

// Opção B: Aumentar K (mola mais rígida)
K = 2.0 → 2.5 → 3.0
```

**Se ainda acelera agressivamente:**
- ❌ Controlador PI de corrente está agressivo
- ✅ **Ação**: Reduzir Kp no PI
```cpp
Kp = 2.0 → 1.5 → 1.0 (reduzir até estabilizar)
```

---

## 📈 Interpretazione dos Gráficos Serial

### Resposta Ideal (sem overshoot):
```
Θ(t)
  1.0 |          ___________
      |        /
  0.5 |   ___/
      |  /
  0.0 |___________________ t
      0   50  100 150 200 250
         Acomodar para 0 sem oscilação
```

### Resposta com Overshoot (problema atual):
```
Θ(t)
  1.0 | ╱╲     ╱╲
      |╱  ╲___╱  ╲___
  0.5 |            ╱
      |
  0.0 |_____________ t
      0   50  100 150 200
         ← OSCILAÇÕES (ruim!)
```

### Resposta Super-amortecida (muito lento):
```
Θ(t)
  1.0 |
      |      ___________
  0.5 |    /
      |___/
  0.0 |_________________ t
      0    100  200  300 400
         ← Muito lento!
```

---

## ⚙️ Ajustes do Controlador PI

Se a resposta estiver corrigida na mola mas ainda houver problemas, ajuste também o **PI de corrente**:

### Ganho Proporcional (Kp)
- **Alto (Kp=5)**: Resposta rápida, mas pode oscilar
- **Baixo (Kp=1)**: Resposta lenta, mas estável
- **Recomendado**: Kp = K_impedancia / 2

### Ganho Integrativo (Ki)
- **Alto (Ki=0.5)**: Elimina erro DC, mas lento
- **Baixo (Ki=0.1)**: Rápido, mas pode não eliminar offset
- **Recomendado**: Ki = Kp / 10

### Anti-Windup
O código já tem anti-windup! Mas a lógica é:
```cpp
if (saturado)
    erro_integrado -= erro;  // Desfaz integração
```

Isso está correto. ✅

---

## 📋 Resumo: Próximas Ações

### 1. **IMEDIATAMENTE**: Testar Configuração Conservadora
```cpp
// src/control_instances.cpp
VirtualImpedance molaVirtual = {2.0f, 1.0f, 0.0f};  // B aumentado!
PIController pi_iq = {2.0f, 0.2f, 0.0f, 12.0f};
```

### 2. **DEPOIS**: Avaliar Resposta com Serial Monitor
- Aplicar perturbação manual
- Observar overshoot e oscilações
- Registrar tempo de acomodação

### 3. **ITERAR**: Ajustar conforme tabela acima
- Se oscila → aumentar B
- Se lento → aumentar K ou diminuir B
- Se agressivo → diminuir Kp

### 4. **VALIDAR**: Confirmar estabilidade
- Testar com múltiplas perturbações
- Verificar se comportamento é consistente
- Validar que corrente não satura

---

## 🧮 Cálculo da Relação B/K para Amortecimento Crítico

Se você conhecer a inércia do seu motor (I):

```
B_critica = 2 * √(K * I)

Exemplo:
  K = 2.0 N.m/rad
  I = 0.01 kg.m² (típico para motor pequeno)
  
  B_critica = 2 * √(2.0 * 0.01)
            = 2 * √0.02
            = 2 * 0.1414
            ≈ 0.283

  Usar B = 0.3 a 0.4 para estar seguro
```

---

## ⚡ DEBUG: Verificar o Que Está Acontecendo

Adicione este código temporário em `mainx.cpp` para debugar:

```cpp
static uint32_t print_counter = 0;
if (++print_counter % 100 == 0)  // Print a cada 100ms
{
    Serial.printf("K=%.2f B=%.2f | error_θ=%.3f | ω=%.3f | τ_imp=%.3f\n",
                  molaVirtual.K,
                  molaVirtual.B,
                  (molaVirtual.theta_set - theta_m),
                  omega_filtered,
                  iq_imp);  // Torque de impedância
}
```

Assim você pode ver em tempo real como a impedância está calculando o torque.

---

## ✅ Checklist de Implementação

- [ ] Copiar Configuração 1 para `control_instances.cpp`
- [ ] Compilar e fazer upload
- [ ] Abrir Serial Monitor a 115200 baud
- [ ] Aplicar perturbação manual e observar
- [ ] Ajustar B conforme gráfico de resposta
- [ ] Testar múltiplas perturbações
- [ ] Ir para Configuração 2 quando estável
- [ ] Documentar os valores finais que funcionaram
