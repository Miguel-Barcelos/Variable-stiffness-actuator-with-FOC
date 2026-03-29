# Sistema de Controle Unificado - Documentação

## 📁 Estrutura de Arquivos Após Refatoração

### ✅ Arquivos Mantidos (Não Modificados)
- `mainx.cpp` - Loop principal e tarefas FOC
- `config.h` - Configurações de hardware
- `foc_math.h/.cpp` - Funções matemáticas FOC
- `foc_utils.cpp` - Utilitários FOC
- `encoder_utils.h/.cpp` - Interface encoder AS5600
- `svpwm.cpp` - Modulação SVPWM
- `motor_controller.h` - Classe controlador (não usada)

### 🗑️ Arquivos Removidos (Unificados)
- `control_instances.h/.cpp` → Integrado em `control_system.h/.cpp`
- `pi_control.h/.cpp` → Integrado em `control_system.h/.cpp`
- `impedance.h/.cpp` → Integrado em `control_system.h/.cpp`

### 🆕 Arquivos Criados
- `control_system.h` - Declarações unificadas
- `control_system.cpp` - Implementações unificadas

---

## 🏗️ Arquitetura do Sistema Unificado

```
control_system.h/.cpp
├── PI Controllers
│   ├── PIController (struct)
│   ├── compute_pi() function
│   └── Instâncias: pi_id, pi_iq
├── Virtual Impedance
│   ├── VirtualImpedance (struct)
│   ├── compute_impedance_torque() function
│   └── Instância: molaVirtual
└── Global References
    ├── id_ref, iq_ref (current refs)
    └── iq_limit (safety limit)
```

---

## 📋 Como Usar o Sistema Unificado

### 1. Include Único
```cpp
// ANTES (múltiplos includes):
#include "pi_control.h"
#include "impedance.h"
#include "control_instances.h"

// DEPOIS (apenas um):
#include "control_system.h"
```

### 2. Acesso às Instâncias
```cpp
// Controladores PI (já inicializados)
extern PIController pi_id;      // Corrente de fluxo
extern PIController pi_iq;      // Corrente de torque

// Impedância virtual (já inicializada)
extern VirtualImpedance molaVirtual;

// Referências globais
extern float id_ref, iq_ref, iq_limit;
```

### 3. Uso das Funções
```cpp
// Controlador PI
float output = compute_pi(&pi_iq, error);

// Controle de impedância
float torque_ref = compute_impedance_torque(&molaVirtual, theta, omega);
iq_ref = torque_ref;  // Define referência para FOC
```

---

## ⚙️ Configuração dos Parâmetros

### Valores Atuais (Configurados)
```cpp
// Controladores PI
PIController pi_id  = {1.5f, 0.09f, 0.0f, 12.0f};
PIController pi_iq  = {1.5f, 0.09f, 0.0f, 12.0f};

// Impedância Virtual
VirtualImpedance molaVirtual = {1.0f, 1.9f, 0.0f};
```

### Como Modificar em Runtime
```cpp
// Alterar rigidez da mola
molaVirtual.K = 2.0f;

// Alterar amortecimento
molaVirtual.B = 1.5f;

// Alterar posição de equilíbrio
molaVirtual.theta_set = 1.57f;  // π/2 rad

// Modificar ganhos PI
pi_iq.Kp = 2.0f;
pi_iq.Ki = 0.2f;
```

---

## 🔧 Funções Disponíveis

### compute_pi()
```cpp
float compute_pi(PIController *pi, float erro);
```
- **Entrada**: Ponteiro para controlador PI + erro atual
- **Saída**: Comando de controle (limitado por anti-windup)
- **Uso**: Controle de corrente FOC

### compute_impedance_torque()
```cpp
float compute_impedance_torque(VirtualImpedance *imp, float theta, float omega);
```
- **Entrada**: Config impedância + posição atual + velocidade atual
- **Saída**: Torque de referência (A)
- **Equação**: τ = K*(θ_set - θ) - B*ω

---

## 📊 Constantes de Configuração

```cpp
// Valores padrão seguros
#define PI_CONTROLLER_DEFAULT     {1.5f, 0.09f, 0.0f, 12.0f}
#define VIRTUAL_IMPEDANCE_DEFAULT {1.0f, 1.9f, 0.0f}
#define CURRENT_LIMIT_DEFAULT     2.0f
#define VOLTAGE_LIMIT_DEFAULT     12.0f
```

---

## 🧪 Teste do Sistema Unificado

### Compilação
```bash
platformio run --target upload
```

### Verificação
1. ✅ Compila sem erros
2. ✅ Serial mostra dados FOC
3. ✅ Motor responde a perturbações
4. ✅ Comportamento similar ao anterior

### Debug
```cpp
// Adicionar no mainx.cpp para verificar
Serial.printf("K=%.1f B=%.1f | iq_ref=%.3f | Iq=%.3f\n",
              molaVirtual.K, molaVirtual.B, iq_ref, motor.iq);
```

---

## 📈 Benefícios da Unificação

### ✅ Vantagens
- **Menos arquivos**: 6 arquivos → 2 arquivos
- **Manutenção**: Tudo em um lugar
- **Visibilidade**: Parâmetros juntos
- **Performance**: Mesmo código, mesma eficiência
- **Modularidade**: Estruturas bem organizadas

### ⚠️ Considerações
- **Compatibilidade**: Interface mantida igual
- **Documentação**: Comentários detalhados
- **Segurança**: Mesmo anti-windup e limites
- **Flexibilidade**: Fácil modificar parâmetros

---

## 🔄 Migração Completa

### Passos para Migração
1. ✅ Criar `control_system.h/.cpp`
2. ✅ Atualizar includes em `mainx.cpp`
3. ✅ Testar compilação
4. ✅ Verificar funcionamento
5. 🟡 **REMOVER** arquivos antigos:
   - `control_instances.h`
   - `control_instances.cpp`
   - `pi_control.h`
   - `pi_control.cpp`
   - `impedance.h`
   - `impedance.cpp`

### Comando para Remover Arquivos Antigos
```bash
# No terminal do VS Code / PowerShell:
Remove-Item "src\control_instances.h"
Remove-Item "src\control_instances.cpp"
Remove-Item "src\pi_control.h"
Remove-Item "src\pi_control.cpp"
Remove-Item "src\impedance.h"
Remove-Item "src\impedance.cpp"
```

---

## 🎯 Resumo

O sistema de controle agora está **unificado em 2 arquivos** com todas as funcionalidades preservadas:

- **`control_system.h`**: Declarações, estruturas, protótipos
- **`control_system.cpp`**: Implementações e instâncias globais

**Resultado**: Pasta mais limpa, código mais organizado, mesma performance! 🚀
