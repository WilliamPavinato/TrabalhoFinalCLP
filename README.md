# ICBM Ballistic Simulation 🚀
Rust Physics Engine + Python/Pygame GUI
Este projeto consiste em um simulador de trajetórias de mísseis balísticos intercontinentais. A aplicação utiliza uma arquitetura híbrida para maximizar a performance e a flexibilidade.

## Arquitetura do Sistema
O projeto é dividido em dois domínios principais que se comunicam via FFI (Foreign Function Interface):

Backend (Rust): Um motor de física de alta performance responsável por calcular a trajetória usando o método de integração numérica RK4 (Runge-Kutta 4). Ele lida com variáveis complexas como gravidade e arrasto aerodinâmico.

Frontend (Python): Uma interface gráfica interativa construída com Pygame. Responsável por renderizar a trajetória, gerenciar a telemetria em tempo real e capturar as entradas do usuário.

## O Makefile: Orquestração de Build
O Makefile é o coração da automação deste projeto. Como trabalhamos com duas linguagens que possuem ciclos de vida diferentes (Rust é compilado, Python é interpretado), o Makefile garante que o fluxo de execução seja respeitado:

make build: Compila o código Rust em modo --release (otimizado) e move a biblioteca gerada (.so ou .dll) para a raiz do projeto, permitindo que o Python a localize instantaneamente.

make gui: Garante que o motor de física esteja atualizado (executa o build) e inicia a interface visual.

make check: Uma ferramenta de diagnóstico que verifica se o compilador Rust, o interpretador Python e a biblioteca Pygame estão instalados e configurados corretamente.

make clean: Remove artefatos de compilação e logs temporários para resetar o ambiente de desenvolvimento.

## Estrutura do Código
Motor de Física (lib.rs)
Escrito em Rust para garantir segurança de memória e velocidade. Expõe uma interface C-compatible (#[repr(C)]) para que o Python possa enviar parâmetros de simulação e receber um array de pontos de trajetória.

Interface Gráfica (MissileSimGui.py)
Utiliza a biblioteca ctypes para carregar a biblioteca dinâmica compilada pelo Rust. Converte as coordenadas físicas (metros) em coordenadas de tela (pixels) de forma dinâmica através de uma função de auto-escala.

## Como Executar
Pré-requisitos
Rust (via rustup.rs)

Python 3.x e Pygame (pip install pygame)

Build Tools do Windows (C++ Desktop Development) — Necessário para o linker do Rust no Windows.

Comandos de Automação
Se você estiver no Linux/macOS, use o Makefile:

Bash

make gui
Se estiver no Windows, utilize o script de lote fornecido para simular o comportamento do Makefile:

DOS

.\run.bat
## Funcionalidades da Simulação
Ajuste Dinâmico: Altere velocidade inicial, ângulo e resistência do ar via sliders.

Telemetria: Visualize altitude máxima, tempo de voo e ponto de impacto exato.

Comparação Teórica: O sistema desenha uma linha azul representando o alcance parabólico ideal (vácuo) para comparação com o modelo real com arrasto.

Este README serve como documentação técnica para o seu trabalho final de Conceitos de Linguagens de Programação, destacando a viabilidade de sistemas multi-linguagem.

Deseja que eu adicione uma seção de "Troubleshooting" (Solução de Problemas) com os erros que resolvemos hoje?