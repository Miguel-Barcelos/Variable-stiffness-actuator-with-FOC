
# Controle de impedância ativa aplicado a atuadores elétricos com Controle Orientado a Campo (FOC)

Este repositório contém a base de dados experimentais, os scripts e arquivos de desenvolvimento de um controle de impedância ativa aplicado a atuadores elétricos com Controle Orientado a Campo. 

Este trabalho foi submetido e aceito como Trabalho de Conclusão de Curso para o Curso de Engenharia de Controle e Automação no Instituto Federal Fluminense. Disponível em: DOI: 10.13140/RG.2.2.10687.47526 https://www.researchgate.net/publication/404936282_Controle_de_impedancia_ativa_aplicado_a_atuadores_eletricos_com_Controle_Orientado_a_Campo_FOC?channel=doi&linkId=6a089f97e93d461915964204&showFulltext=true



## Visão Geral do Projeto

O objetivo principal desta pesquisa é o desenvolvimento de um **sensor virtual** baseado em redes neurais artificiais. Utilizando apenas grandezas de fácil monitoramento, como corrente elétrica e sinal PWM, associadas ao tempo decorrido. O modelo é capaz de inferir dinamicamente o empuxo gerado pelo motor, eliminando a necessidade de células de carga físicas na aeronave, reduzindo peso estrutural e custos. Além de possibilitar análises futuras de relação de empuxo vs potência em motopropulsores de drones.

A inclusão da variável temporal atua como um estimador indireto para contornar o efeito de histerese térmica sofrido pelo conjunto propulsor durante a operação em bancada estática.

## 📂 Estrutura do Repositório

*   `/data`: Matriz de dados experimentais contendo as coletas realizadas na bancada estática.
*   `/src`: Scripts utilizados para o tratamento dos dados, arquitetura, treinamento da rede neural e execução de testes de bancada.
*   `/project`: Figuras do desenvolvimento do projeto e arquivo CAD da bancada de teste.
*   `/figures`: Gráficos de desempenho do modelo (regressão, análise de erro e épocas de treinamento).

## 📊 Variáveis do Modelo

*   **Entradas (Inputs):**
    1. Corrente Elétrica ($I$)
    2. Sinal de Controle ($PWM$)
    3. Tempo Decorrido ($t$)
*   **Saída (Output):**
    1. Empuxo Estimado ($T_{est}$)

 ## 📈 Métricas de Avaliação de Modelos
 * MAE (Erro Médio Absoluto).
 * RMSE (Raiz do Erro Quadrático Médio).
 * MAPE (Erro Médio Percentual Absoluto) com tratamento para divisão por zero.
 * $R^2$ (Coeficiente de Determinação / Ajuste).

## 💻 Requisitos e Tecnologias

Os scripts foram desenvolvidos utilizando:
*   [MATLAB]
*   Principais bibliotecas: [Deep Learning Toolbox / Statistics and Machine Learning Toolbox]
