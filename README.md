# ✨ Trabalho SE 05 - Sistema de Monitoramento de Cheias com Raspberry Pi Pico W
<p align="center"> Repositório dedicado ao sistema de monitoramento de cheias utilizando a placa Raspberry Pi Pico W, que monitora o nível de água e o volume de chuva, exibindo estados de alerta através de diferentes atuadores (LED RGB, Buzzer, etc).</p>

## :clipboard: Apresentação da tarefa

Para este trabalho foi necessário implementar um sistema de monitoramento de cheias utilizando o Raspberry Pi Pico W. O sistema monitora o nível de água e o volume de chuva através de um joystick analógico (simulando sensores reais), processa esses dados e exibe alertas visuais e sonoros quando os níveis ultrapassam limites pré-definidos. O sistema utiliza um display OLED SSD1306, uma matriz de LEDs WS2812B, um LED RGB e um buzzer para comunicar o estado atual.

## :dart: Objetivos

- Implementar um sistema de monitoramento de nível de água e volume de chuva
- Exibir dados em tempo real no display OLED SSD1306
- Gerenciar diferentes estados (normal, alerta e crítico) baseados nos níveis monitorados
- Exibir padrões visuais na matriz de LEDs conforme o estado atual
- Controlar o LED RGB indicando o modo de operação (verde para normal, vermelho piscante para alerta)
- Acionar um buzzer quando o sistema entra em modo de alerta
- Implementar um sistema de tempo real usando FreeRTOS para gerenciar todas as tarefas
- Utilizar da API de Filas para o controle da transmissão de dados do sistema

## :books: Descrição do Projeto
Utilizou-se a placa Raspberry Pi Pico (com o microcontrolador RP2040) para criar um sistema completo de monitoramento de cheias. O sistema simula leituras de sensores através de um joystick analógico conectado às entradas ADC. O eixo X controla o nível de água (0-100%) e o eixo Y controla o volume de chuva (0-9).
Quando qualquer valor ultrapassa os limites definidos (70% para nível de água e 8 para volume de chuva), o sistema entra no modo de alerta, ativando sinais visuais e sonoros. Se os valores excederem 90%, o sistema entra em estado crítico, intensificando os alertas.
O sistema é gerenciado pelo sistema operacional de tempo real FreeRTOS, que divide o processamento em tarefas concorrentes, permitindo operação multitarefa eficiente e determinística. Foi utilizado Filas (Queues) para o processamento e comunicação do sistema, aquisição e atuação.

## :walking: Integrantes do Projeto
Matheus Pereira Alves

## :bookmark_tabs: Funcionamento do Projeto

O sistema é dividido em seis tarefas principais do FreeRTOS:

- vLeituraJoystick: Lê os valores analógicos do joystick (simulando sensores reais)
- vProcessamentoDados: Avalia os níveis e determina o modo de operação e estado do sistema
- vLedRGBTask: Controla o LED RGB (verde para normal, vermelho piscante para alerta)
- vBuzzerTask: Gerencia o buzzer, produzindo alertas sonoros em modo de alerta
- vMatrixLEDTask: Controla a matriz de LEDs 5x5, exibindo padrões visuais conforme o estado
- vDisplayOLEDTask: Atualiza o display OLED com informações do sistema

tudo isso sendo comandado por meio de Filas.

## :eyes: Observações

- A comunicação entre tarefas é realizada através de filas FreeRTOS de tamanho 1 para minimizar latência;
- As tarefas operam em diferentes prioridades para garantir que as mais críticas sejam executadas primeiro;
- Todas as operações de fila são não-bloqueantes para melhorar a resposta do sistema em tempo real;
- De preferência, utilize o PICO SDK versão 2.1.0;
- Lembre-se de anexar o seu diretório do FreeRTOS;
- Fora utilizado xQueueOverwrite, xQueuePeek e vTaskDelayUntil.

:camera: GIF mostrando o funcionamento do programa na placa Raspberry Pi Pico
<p align="center">
  <img src="images/trabalhose05.gif" alt="GIF" width="526px" />
</p>
:arrow_forward: Vídeo no youtube mostrando o funcionamento do programa na placa Raspberry Pi Pico
<p align="center">
    <a href="https://www.youtube.com/watch?v=Lx5jLoK_OvM">Clique aqui para acessar o vídeo</a>
</p>
