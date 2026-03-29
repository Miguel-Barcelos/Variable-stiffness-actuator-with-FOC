# 🧪 Guia Prático de Teste e Ajuste

## ✅ Etapa 1: Compilar e Fazer Upload

```bash
# No terminal do PlatformIO:
platformio run --target upload
```

Confirme que:
- ✅ Compilação bem-sucedida
- ✅ Upload completo
- ✅ Sem erros de conexão

---

## 🔍 Etapa 2: Abrir Serial Monitor

1. **Abra o Serial Monitor** (Ctrl+Shift+A no PlatformIO)
2. **Configure a velocidade**: 115200 baud
3. **Aguarde a calibração** (~3 segundos)

Você deve ver mensagens como:
```
Tick:1234 | Raw:2048 | Theta_m:0.123 | Theta_e:0.861 | Omega:0.000 ...
```

---

## 🔄 Etapa 3: Teste de Perturbação Manual

### Antes do Teste - Certifique-se:
- [ ] Motor escuta bem (não travado)
- [ ] Encoder funcionando (valores Raw mudam quando você gira)
- [ ] Sem sons estranhos

### Procedimento:
1. **Segure o motor com a mão**
2. **Gire lentamente ~45-90° (~0.5-1 rad)**
3. **Solte rapidamente**
4. **Observe o Serial Monitor com atenção nos primeiros 500ms**

### O Que Observar:

**GOOD ✅** - Amortecimento crítico:
```
Theta antes:  0.0
Theta pico:   0.8 (sua mão aplicou força)
Theta 100ms:  0.4 (retornando...)
Theta 200ms:  0.1 (quase lá)
Theta 300ms:  ~0.0 (estável!)
```

**BAD ❌** - Subamortecido (PROBLEMA ATUAL):
```
Theta antes:  0.0
Theta pico:   1.0 (ultrapassa!)
Theta 100ms:  -0.3 (volta demais!)
Theta 200ms:  0.5 (oscila!)
Theta 300ms:  -0.2 (ainda oscilando!)
```

**BAD ❌** - Super-amortecido (muito lento):
```
Theta antes:  0.0
Theta pico:   0.8 (sua mão aplicou)
Theta 100ms:  0.6 (muito lento!)
Theta 200ms:  0.4 (ainda lento!)
Theta 300ms:  0.2 (só agora está retornando)
Theta 500ms:  ~0.0 (demorou muito!)
```

---

## 📊 Etapa 4: Analisar os Dados

### Métricas para Avaliar:

**1. Tempo de Acomodação (Settling Time)**
- Tempo para Theta retornar a ±5% do setpoint
- Ideal: 100-300ms
- Muito rápido: <50ms (pode instabilizar)
- Muito lento: >500ms (ruim para controle)

**2. Overshoot**
- Porcentagem máxima além do setpoint
- Ideal: 0-10%
- Razoável: 0-20%
- Ruim: >30%

**3. Oscilações Residuais**
- Após retornar, há barulho/vibração?
- Ideal: Nenhuma
- Razoável: Pequenas ondulações que decaem

### Como Calcular Overshoot:
```
Overshoot % = ((Theta_max - Theta_final) / Theta_inicial) × 100

Exemplo:
  Você aplicou 1.0 rad
  Theta máximo atingido = 1.2 rad
  Overshoot = ((1.2 - 0.0) / 1.0) × 100 = 20%
```

---

## 🔧 Etapa 5: Ajustar os Parâmetros

### Fluxograma de Decisão:

```
┌─ Teste de Perturbação ─┐
│                        │
│  Monitor Omega         ├─ Oscila muito? ─── Aumentar B (+0.2)
│  Monitor Theta         ├─ Muito lento?  ─── Aumentar K (+0.5) OU Reduzir B (-0.1)
│  Monitor Iq            ├─ Agressivo?    ─── Reduzir Kp (-0.5)
│                        ├─ Perfeito? ✓   ─── DOCUMENTAR!
└────────────────────────┘
```

### Sequência de Ajustes Recomendada:

**Teste 1 - Sua Configuração ATUAL:**
```cpp
// Valores colocados no sistema:
K = 2.0,  B = 1.0,  Kp = 2.0
```
👉 Faça o teste acima e veja como responde

**Teste 2 - Se ainda oscila (ondula muito):**
```cpp
K = 2.0,  B = 1.2,  Kp = 2.0  // Aumentar B em +0.2
```

**Teste 3 - Se ainda oscila:**
```cpp
K = 2.0,  B = 1.4,  Kp = 2.0  // Aumentar B novamente
```

**Teste 4 - Se mudar e ficar lento demais:**
```cpp
K = 2.5,  B = 1.2,  Kp = 2.5  // Aumentar K e dar reset ao B
```

**Teste 5 - Se ainda acelera agressivamente:**
```cpp
K = 2.0,  B = 1.0,  Kp = 1.5  // Reduzir Kp para frear a resposta
```

---

## 📝 Tabela de Teste - Preencha Enquanto Testa

| # | K | B | Kp | Comportamento | Overshoot | Oscila? | Tempo Acomod. | Próximo Teste |
|---|---|---|---|---|---|---|---|---|
| 1 | 2.0 | 1.0 | 2.0 | ATUAL | ? | ? | ? | Ver abaixo ↓ |
| 2 | 2.0 | 1.2 | 2.0 | | | | | |
| 3 | 2.0 | 1.4 | 2.0 | | | | | |
| 4 | 2.5 | 1.2 | 2.5 | | | | | |
| 5 | 2.0 | 1.0 | 1.5 | | | | | |

---

## 🎯 Código para Testar em Tempo Real

Se quiser adicionar um código de debug para MONITORAR os parâmetros em tempo real, edite `mainx.cpp` no final do `focTask` (após `foc_tick++`):

```cpp
// ===== DEBUG: Descomente para monitorar impedância em tempo real =====
static uint32_t debug_counter = 0;
if (++debug_counter % 100 == 0)  // Print a cada 100 iterações (100ms)
{
    float error_theta = molaVirtual.theta_set - motor.theta_m;
    float torque_imp = compute_impedance_torque(&molaVirtual, motor.theta_m, motor.omega_measured);
    
    Serial.printf("K=%.1f B=%.1f | error_θ=%.3f rad | ω=%.3f rad/s | τ_imp=%.3f A | Iq=%.3f A\n",
                  molaVirtual.K,
                  molaVirtual.B,
                  error_theta,
                  motor.omega_measured,
                  torque_imp,
                  motor.iq);
}
```

Isso mostrará:
- Ganhos atuais (K, B)
- Erro de posição
- Velocidade medida
- Torque que a impedância está comandando
- Corrente Iq real

---

## ✅ Checklist de Teste

- [ ] Compilado e enviado ao ESP32
- [ ] Serial Monitor aberto a 115200 baud
- [ ] Calibração concluída (~3 seg)
- [ ] Primeira perturbação testada
- [ ] Dados anotados na tabela
- [ ] Comportamento avaliado (Good/Bad)
- [ ] Próxima configuração decidida
- [ ] **CRÍTICO**: Arquivo `control_instances.cpp` modificado com novos valores
- [ ] Recompilado e enviado
- [ ] Teste 2 executado
- [ ] Resultado documentado

---

## 🚨 Se Algo Der Muito Errado

Se o motor começar a fazer barulhos estranhos, vibrar muito ou ficar quente:

1. **IMEDIATAMENTE**: Desconecte a alimentação
2. **Restaure valores conservadores**:
```cpp
VirtualImpedance molaVirtual = {2.0f, 1.5f, 0.0f};  // B muito alto
PIController pi_iq = {1.0f, 0.1f, 0.0f, 12.0f};   // Ganhos muito baixos
```
3. **Recompile e teste novamente**
4. **Procure por problemas no código** (race conditions, etc)

---

## 📈 Meta Final

Após todos os testes, você quer alcançar:

```
┌─────────────────────────────────────────────────┐
│ ✅ RESPOSTA IDEAL ESPERADA                      │
│                                                  │
│ • Overshoot:        < 10%                       │
│ • Tempo Acomod:     150-300ms                   │
│ • Oscilações:       Nenhuma visível              │
│ • Estabilidade:     Consistente em < 5% erro   │
│ • Som:              Suave, sem zumbidos         │
│ • Aquecimento:      Mínimo                      │
│                                                  │
│ Se atingir isto, documentar os valores K, B! ✓  │
└─────────────────────────────────────────────────┘
```

---

## 📞 Troubleshooting

### Problema: "Não vejo mudanças no comportamento"
**Possível causa**: Motor não está respondendo ao comando
- [ ] Verificar se `driver_enabled = true`
- [ ] Verificar se `iq_ref` está sendo recebido (não é zero sempre)
- [ ] Testar com Serial Monitor vendo `Iq: 0.000` (está zerando depois)

### Problema: "Motor está muito quente"
**Possível causa**: Corrente alta demais
- [ ] Reduzir K ou Kp
- [ ] Aumentar B (amortecimento, reduz corrente)
- [ ] Verificar se há curto-circuito

### Problema: "Encoder lê valores estranhos"
**Possível causa**: Comunicação I2C falha
- [ ] Verificar conexão AS5600
- [ ] Verificar pinos SDA/SCL (25/26 no código)
- [ ] Tentar aumentar velocidade I2C ou reduzir

---

## 🎓 Conceito: Por Que B Afeta Overshoot?

Pense em um sistema massa-mola-amortecedor:

```
    ┌─────────────┐
    │  MOTOR      │
    │ (I = inércia)
    └──────┬──────┘
           │ ╱ K (mola - restaura posição)
           │╱
    ●─────●────────────► Posição
             ╲
              ╲ B (amortecedor - dissipa energia)

τ_total = K * (θ_set - θ) - B * ω

Se B é baixo:
  - A mola (K) puxa forte
  - O amortecedor (B) freia pouco
  - Resultado: motor "overshooteia" (passa do alvo)

Se B é alto:
  - A mola (K) puxa forte
  - O amortecedor (B) freia muito
  - Resultado: motor retorna lentamente (sem overshoot)
```

**Balanceamento perfeito**: B alto o suficiente para evitar overshoot, mas não tão alto que fique lento.

---

Boa sorte com os testes! 🚀 Documente tudo que funciona para poder replicar depois.
