# 🔍 DIAGNÓSTICO: Comportamento "Resistência Contínua" do Motor

## Problema Identificado

O motor apresenta **"resistência contínua"** (baixa resposta dinâmica) e não exibe o comportamento esperado de **rigidez virtual com impedância**.

---

## Root Cause (Raiz do Problema) 🎯

### **Issue Crítica: `thetaDesired` Congelado**

- **O quê**: `molaVirtual.thetaDesired` é inicializado **UMA VEZ** durante calibração e **NUNCA é atualizado** durante operação normal.
- **Consequência**: O controlador de impedância tenta sempre manter o motor na **posição inicial** (de calibração).
- **Efeito observado**: Motor comporta-se como uma "mola congelada" tentando voltar a um ponto fixo → parece estar gerando uma "resistência contínua".

### Equação de Impedância (do `foc_isr_handler()`):
```cpp
float error_theta = molaVirtual.thetaDesired - theta_m;  // ← thetaDesired FIXO!
float iq_imp = (molaVirtual.K * error_theta) - (molaVirtual.B * omega);
```

Se `thetaDesired` nunca muda:
- ✓ Quando `theta_m` se afasta de `thetaDesired` → erro cresce → torque aumenta (mola puxa de volta)
- ✗ Quando `theta_m` se aproxima de `thetaDesired` → erro diminui → isto é controlado apenas quando o motor está em movimento ativo

**Resultado**: Comportamento similar a uma **mola muito mole ou travada**.

---

## 🔧 Soluções Implementadas

### 1. **Nova Tarefa: `positionDesiredTask()` (100 Hz)**

Adicionada em `mainx.cpp`:

```cpp
void positionDesiredTask(void *pvParameters)
{
    // Gera uma trajetória senoidal para teste
    // Simula comandos de posição desejada
    // amplitude = ±45° (π/4)
    // frequência = 0.5 Hz
    
    molaVirtual.thetaDesired = position_command + amplitude * sin(2π*f*t);
}
```

**Benefício**: Agora o `thetaDesired` é **atualizado dinamicamente**, permitindo:
- Teste de resposta dinâmica
- Visualização do comportamento de "mola"
- Validação de que impedância está funcionando

### 2. **Aumento de Ganhos (Calibração de Parâmetros)**

Alterados em `control_system.h`:

| Parâmetro | Anterior | Novo | Justificativa |
|-----------|----------|------|---------------|
| `K` (rigidez) | 15.0 | **50.0** | Mola 3.3x mais rígida → resposta mais forte |
| `B` (amortecimento) | 0.5 | **2.0** | 4x mais amortecimento → menos oscilação |
| `Kp` (PI proporcional) | 0.5 | **2.0** | Resposta 4x mais rápida do controlador |
| `Ki` (PI integral) | 0.1 | **0.5** | Integração 5x mais agressiva → elimina erro de regime |
| `limite_saida` (PI saturação) | 12.0 | **20.0** | Permite tensões maiores se necessário |
| `CURRENT_LIMIT_DEFAULT` | 2.0 A | **3.0 A** | Mais corrente disponível |

**Racional**: Com ganhos maiores, o sistema é mais responsivo e a mola virtual mais evidente.

---

### 3. **Melhorias de Debug (Output da `debugTask`)**

Antes:
```
Tick:... | Raw:... | Theta_m:... | Theta_e:... | Omega:... | Ia:... | Ib:... | Ic:... | Iq:... | Vq:... | Fault:...
```

Depois:
```
Tick:... | Tm:... | Te:... | Om:... | Iq:... | Td:... | Vq:... | Ia:... | Fault:...
```

**Novo campo**: `Td:...` = `thetaDesired` (permite verificar que está mudando!)

---

## 🧪 Como Testar

### Pré-teste
1. **Compilar e carregar** novo código
2. **Iniciar serial** @ 115200 baud
3. **Observar output** de debug

### Teste Dinâmico

**Esperado com as mudanças**:
- Motor inicia em posição de calibração
- Tarefa `positionDesiredTask()` começa a **gerar referência senoidal** (±45°, 0.5 Hz)
- Motor deve **acompanhar** a trajetória com uma **mola virtual visível**
- Serial mostra:
  - `Iq:...` variando (corrente sendo aplicada)
  - `Td:...` mudando sinusoidalmente (trajetória desejada)
  - `Tm:...` seguindo `Td` com lag (comportamento de mola amortecida)

### Se Ainda Não Funcionar

1. **Aumentar ainda mais K e B** em `control_system.h`
2. **Aumentar Kp do PI** (ex: 5.0f)
3. **Verificar I2C** da `positionTask()` não está travando ISR
4. **Adicionar GPIO debug** para confirmar ISR está rodando

---

## 📋 Resumo das Mudanças

| Arquivo | Mudança |
|---------|---------|
| `mainx.cpp` | + `positionDesiredTask()` <br/> + criação em `setup()` <br/> + debug output melhorado |
| `control_system.h` | ↑ K: 15→50 <br/> ↑ B: 0.5→2.0 <br/> ↑ Kp: 0.5→2.0 <br/> ↑ Ki: 0.1→0.5 <br/> ↑ limite_saida: 12→20 <br/> ↑ CURRENT_LIMIT: 2→3 |

---

## 🎯 Próximas Ações

1. ✅ Compilar novo código
2. ⏳ Testar motor em hardware
3. ⏳ Observar trajetória motor vs. `thetaDesired`
4. ⏳ **Como o motor deve se comportar**:
   - **Sem comando** (antes): Tenta voltar a posição inicial = "mola congelada"
   - **Com senoide** (agora): Acompanha referência com lag amortecido = "mola real"
5. ⏳ Se OK: Remover `positionDesiredTask()` e implementar **entrada de comando real** (CAN, Serial, Joystick, etc)

---

## ⚠️ Notas de Segurança

- Ganhos maiores **aumentam risco** de oscilação/instabilidade
- Se motor começar a oscilar ou comportar-se agressivamente:
  - **REDUZIR** `K` e `Kp` imediatamente
  - Aumentar `B` para amortecimento
  - Reduzir `CURRENT_LIMIT`
- Sempre usarcompilação/teste incremental

---

**Status**: Diagnóstico completado. Aguardando teste em hardware.
