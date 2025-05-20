// Trabalho SE 05 - Sistema de Monitoramento de Cheias
// Bibliotecas utilizadas
#include <stdio.h>              
#include <ctype.h>              
#include <string.h>             
#include "pico/stdlib.h"        
#include "pico/binary_info.h"   
#include "hardware/pwm.h"       
#include "hardware/adc.h"       
#include "hardware/clocks.h"    
#include "hardware/pio.h"       
#include "ws2818b.pio.h"        
#include "ssd1306.h"            
#include "FreeRTOS.h"           // Bibliotecas do freeRTOS
#include "FreeRTOSConfig.h"     
#include "task.h"               
#include "queue.h"              

// Definições e constantes
#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define ENDERECO 0x3C
#define WIDTH 128
#define HEIGHT 64
#define RED_PIN 13
#define BLUE_PIN 12
#define GREEN_PIN 11
#define BOTAO_A 5
#define BUZZER 21
#define LED_COUNT 25
#define LED_PIN 7

// ADC para joystick
#define ADC_X 26  // Pino do joystick X no ADC0
#define ADC_Y 27  // Pino do joystick Y no ADC1

// Definindo a frequência desejada
#define PWM_FREQ_LED 1000  // 1 kHz
#define PWM_FREQ_BUZZER 1000  // 1 kHz
#define PWM_WRAP 255   // 8 bits de wrap (256 valores)

// Definição dos modos de operação
#define MODO_NORMAL 0
#define MODO_ALERTA 1

// Estados para a matriz de LEDs
#define ESTADO_NORMAL 0
#define ESTADO_ALERTA 1
#define ESTADO_CRITICO 2
#define ESTADO_DESLIGADO 3

// Valores limite para o alerta
#define LIMITE_NIVEL_AGUA 70  
#define LIMITE_VOLUME_CHUVA 80  

// Flags e Variáveis globais
ssd1306_t ssd;           
volatile uint8_t g_nivel_agua = 0;
volatile uint8_t g_volume_chuva = 0;
volatile int g_modo_atual = MODO_NORMAL;
volatile int g_estado_matrix = ESTADO_NORMAL;

// Definição das filas
QueueHandle_t xQueueDados;         // Fila para enviar dados entre tarefas
QueueHandle_t xQueueModo;          // Fila para comunicar mudanças de modo

// Estrutura para dados do sensor
typedef struct {
    uint8_t nivel_agua;        // 0-100%
    uint8_t volume_chuva;      // 0-100%
} DadosSensor_t;

// Estrutura para modo de operação
typedef struct {
    int modo;                  // Modo de operação
    int estado_matrix;         // Modo da matriz
} ModoDados_t;

// Definição da estrutura do pixel
struct pixel_t {
    uint8_t G, R, B;
};
typedef struct pixel_t pixel_t;
typedef pixel_t npLED_t;

// Configurações das máquinas de estados do PIO
npLED_t leds[LED_COUNT];
PIO np_pio;
uint sm;

// Protótipos de funções
void initSettings();
void initADC();
void initssd1306();
void configurarBuzzer(uint32_t volume);
void configurarLEDRGB(uint8_t r, uint8_t g, uint8_t b);
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b);
void npWrite();
void npDisplayDigit(int digit);
void npClear();
void npInit(uint pin);
int getIndex(int x, int y);
void vLeituraJoystick(void *pvParameters);
void vProcessamentoDados(void *pvParameters);
void vLedRGBTask(void *pvParameters);
void vBuzzerTask(void *pvParameters);
void vMatrixLEDTask(void *pvParameters);
void vDisplayOLEDTask(void *pvParameters);

// Matrizes para cada estado na matriz de LEDs 
const uint8_t states[4][5][5][3] = {
    // Verde - Normal
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}, 
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}},    
        {{0, 0, 0}, {0, 0, 0}, {0, 100, 0}, {0, 0, 0}, {0, 0, 0}}  
    },
    // Amarelo - Alerta
    {
        {{0, 0, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}},
        {{120, 40, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {120, 40, 0}},
        {{0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {120, 40, 0}, {0, 0, 0}, {0, 0, 0}}
    },
    // Vermelho - Crítico
    {
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {100, 0, 0}, {0, 0, 0}, {100, 0, 0}, {0, 0, 0}},
        {{100, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {100, 0, 0}} 
    },
    // Desligado
    {
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},   
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}},
        {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}} 
    },
};

int main() {
    // Inicializa stdio
    stdio_init_all();
    printf("Iniciando sistema de monitoramento de cheias\n");
    
    // Inicializa as configurações
    initSettings();
    initADC();
    initssd1306();
    npInit(LED_PIN);
    
    // Configura o ADC corretamente
    adc_init();
    adc_gpio_init(ADC_X);
    adc_gpio_init(ADC_Y);
    adc_set_temp_sensor_enabled(false);
    adc_set_clkdiv(96);
    
    // Cria as filas
    xQueueDados = xQueueCreate(1, sizeof(DadosSensor_t)); // Tamanho 1 para menos latência
    xQueueModo = xQueueCreate(1, sizeof(ModoDados_t)); // Tamanho 1 para menos latência
    
    if (xQueueDados == NULL || xQueueModo == NULL) {
        printf("Erro ao criar as filas!\n");
        while(1);
    }
    
    // Cria as tarefas e define as prioridades (a leitura e o processamento devem ser mais rápidas)
    xTaskCreate(vLeituraJoystick, "Leitura Joystick", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 4, NULL);
               
    xTaskCreate(vProcessamentoDados, "Processamento Dados", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 3, NULL);
               
    xTaskCreate(vMatrixLEDTask, "Matrix LED", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 2, NULL);
               
    xTaskCreate(vLedRGBTask, "LED RGB", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 2, NULL);
               
    xTaskCreate(vBuzzerTask, "Buzzer", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
               
    xTaskCreate(vDisplayOLEDTask, "Display OLED", configMINIMAL_STACK_SIZE*2,
               NULL, tskIDLE_PRIORITY + 1, NULL);
    
    // Inicia o escalonador do FreeRTOS
    vTaskStartScheduler();
    
    // O código nunca deve chegar aqui
    panic_unsupported();
}

// Função para inicializar o ADC para o joystick
void initADC() {
    adc_init();
    adc_gpio_init(ADC_X);
    adc_gpio_init(ADC_Y); 
}

// Função para configurar o volume do buzzer
void configurarBuzzer(uint32_t volume) {
    pwm_set_gpio_level(BUZZER, volume);
}

// Função para configurar a cor do LED RGB
void configurarLEDRGB(uint8_t r, uint8_t g, uint8_t b) {
    pwm_set_gpio_level(RED_PIN, r);
    pwm_set_gpio_level(GREEN_PIN, g);
    pwm_set_gpio_level(BLUE_PIN, b);
}

void initSettings(){
    // Inicializa o pino do botão
    gpio_init(BOTAO_A);
    gpio_set_dir(BOTAO_A, GPIO_IN);
    gpio_pull_up(BOTAO_A);
    
    // Inicializa os pinos PWM para os LEDs e buzzer
    gpio_set_function(RED_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BLUE_PIN, GPIO_FUNC_PWM);
    gpio_set_function(GREEN_PIN, GPIO_FUNC_PWM);
    gpio_set_function(BUZZER, GPIO_FUNC_PWM);
    
    // Obtém os números dos canais PWM para os pinos
    uint slice_num_red = pwm_gpio_to_slice_num(RED_PIN);
    uint slice_num_blue = pwm_gpio_to_slice_num(BLUE_PIN);
    uint slice_num_green = pwm_gpio_to_slice_num(GREEN_PIN);
    uint slice_num_buzzer = pwm_gpio_to_slice_num(BUZZER);
    
    // Configuração da frequência PWM
    pwm_set_clkdiv(slice_num_red, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_blue, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_green, (float)clock_get_hz(clk_sys) / PWM_FREQ_LED / (PWM_WRAP + 1));
    pwm_set_clkdiv(slice_num_buzzer, (float)clock_get_hz(clk_sys) / PWM_FREQ_BUZZER / (PWM_WRAP + 1));
    
    // Configura o wrap do contador PWM para 8 bits (256)
    pwm_set_wrap(slice_num_red, PWM_WRAP);
    pwm_set_wrap(slice_num_blue, PWM_WRAP);
    pwm_set_wrap(slice_num_green, PWM_WRAP);
    pwm_set_wrap(slice_num_buzzer, PWM_WRAP);
    
    // Habilita o PWM
    pwm_set_enabled(slice_num_red, true);
    pwm_set_enabled(slice_num_blue, true);
    pwm_set_enabled(slice_num_green, true);
    pwm_set_enabled(slice_num_buzzer, true);
    
    // Desliga o buzzer e configura o LED RGB para verde (modo normal)
    configurarBuzzer(0);
    configurarLEDRGB(0, 255, 0);
}

void initssd1306() {
    // I2C Initialisation. Using it at 600Khz.
    i2c_init(I2C_PORT, 600 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, ENDERECO, I2C_PORT);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);
}

// Função para configurar o LED na matriz de LEDs
void npSetLED(uint index, uint8_t r, uint8_t g, uint8_t b) {
    leds[index].R = r;
    leds[index].G = g;
    leds[index].B = b;
}

// Função para limpar a matriz de LEDs
void npClear() {
    for (int i = 0; i < LED_COUNT; i++) {
        npSetLED(i, 0, 0, 0);
    }
    npWrite();
}

// Inicialização da matriz de LEDs
void npInit(uint pin) {
    uint offset = pio_add_program(pio0, &ws2818b_program);
    np_pio = pio0;
    sm = pio_claim_unused_sm(np_pio, true);
    ws2818b_program_init(np_pio, sm, offset, pin, 800000.f);
    npClear();
}

// Função para escrever os dados na matriz de LEDs
void npWrite() {
    for (uint i = 0; i < LED_COUNT; i++) {
        pio_sm_put_blocking(np_pio, sm, leds[i].G);
        pio_sm_put_blocking(np_pio, sm, leds[i].R);
        pio_sm_put_blocking(np_pio, sm, leds[i].B);
    }
    sleep_us(100);
}

// Função para obter o índice do LED na matriz de LEDs
int getIndex(int x, int y) {
    if (y % 2 == 0) {
        return 24 - (y * 5 + x);
    } else {
        return 24 - (y * 5 + (4 - x));
    }
}

// Função responsável por exibir o dígito na matriz de LEDs
void npDisplayDigit(int digit) {
    if (digit < 0) digit = 0;
    if (digit > 3) digit = 3;
    
    for (int coluna = 0; coluna < 5; coluna++) {
        for (int linha = 0; linha < 5; linha++) {
            int posicao = getIndex(linha, coluna);
            npSetLED(
                posicao,
                states[digit][coluna][linha][0],  // R
                states[digit][coluna][linha][1],  // G
                states[digit][coluna][linha][2]   // B
            );
        }
    }
    npWrite();
}

// Tarefa de leitura do joystick
void vLeituraJoystick(void *pvParameters) {
    const TickType_t xDelayTicks = pdMS_TO_TICKS(20); // 20 ms fica bom
    DadosSensor_t dados;
    uint16_t raw_x, raw_y;
    
    while (1) {
        // Selecionar e ler o canal ADC 0 (nível da água)
        adc_select_input(0);
        sleep_us(100);
        raw_x = adc_read();
        
        // Selecionar e ler o canal ADC 1 (volume de chuva)
        adc_select_input(1);
        sleep_us(100);
        raw_y = adc_read();
        
        // Converter leituras para porcentagem (0-100%)
        dados.nivel_agua = (uint8_t)((raw_x * 100) / 4095);
        dados.volume_chuva = (uint8_t)((raw_y * 100) / 4095);
        
        // Atualizar variáveis globais para garantir que os valores estejam disponíveis
        g_nivel_agua = dados.nivel_agua;
        g_volume_chuva = dados.volume_chuva;
        
        // Debug via serial
        printf("ADC Raw - X: %d, Y: %d | Porcentagem - Nivel: %d%%, Volume: %d%%\n", 
               raw_x, raw_y, dados.nivel_agua, dados.volume_chuva);
        
        // Enviar dados para a fila (substituir dados antigos se necessário)
        xQueueOverwrite(xQueueDados, &dados);
        
        // Aguardar antes da próxima leitura
        vTaskDelay(xDelayTicks);
    }
}

// Tarefa para processar os dados - versão otimizada
void vProcessamentoDados(void *pvParameters) {
    const TickType_t xDelayTicks = pdMS_TO_TICKS(20); // Reduzido para 20ms para maior responsividade
    DadosSensor_t dados_recebidos;
    ModoDados_t modo_dados;
    int modo_anterior = MODO_NORMAL;
    int estado_anterior = ESTADO_NORMAL;
    
    while (1) {
        // Receber dados da fila
        if (xQueueReceive(xQueueDados, &dados_recebidos, 0) == pdPASS) {
            // Processar os dados
            if (dados_recebidos.nivel_agua >= LIMITE_NIVEL_AGUA || 
                dados_recebidos.volume_chuva >= LIMITE_VOLUME_CHUVA) {
                // Condição de alerta
                modo_dados.modo = MODO_ALERTA;
                
                if (dados_recebidos.nivel_agua >= 90 || dados_recebidos.volume_chuva >= 90) {
                    modo_dados.estado_matrix = ESTADO_CRITICO;
                } else {
                    modo_dados.estado_matrix = ESTADO_ALERTA;
                }
            } else {
                // Condição normal
                modo_dados.modo = MODO_NORMAL;
                modo_dados.estado_matrix = ESTADO_NORMAL;
            }
            
            // Atualizar variáveis globais
            g_modo_atual = modo_dados.modo;
            g_estado_matrix = modo_dados.estado_matrix;
            
            // Notificar outras tarefas se o modo OU o estado mudou
            if (modo_dados.modo != modo_anterior || modo_dados.estado_matrix != estado_anterior) {
                xQueueOverwrite(xQueueModo, &modo_dados);
                modo_anterior = modo_dados.modo;
                estado_anterior = modo_dados.estado_matrix;
                
                // Debug via serial
                printf("Mudanca de estado: Modo=%d, Estado=%d, Nivel=%d%%, Volume=%d%%\n", 
                       modo_dados.modo, modo_dados.estado_matrix, 
                       dados_recebidos.nivel_agua, dados_recebidos.volume_chuva);
            }
        }
        
        vTaskDelay(xDelayTicks);
    }
}

// Tarefa para controlar o LED RGB com base no modo
void vLedRGBTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(100); // 100ms 
    ModoDados_t modo_dados;
    static bool led_estado = false;
    
    // Inicializa o tempo para uso com vTaskDelayUntil
    xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Verifica se há dados na fila de modo - não-bloqueante pelo jeito
        if (xQueuePeek(xQueueModo, &modo_dados, 0) == pdPASS) {
            if (modo_dados.modo == MODO_ALERTA) {
                // No modo de alerta, pisca o LED vermelho
                led_estado = !led_estado;
                if (led_estado) {
                    configurarLEDRGB(255, 0, 0); // Vermelho
                } else {
                    configurarLEDRGB(0, 0, 0);   // Desligado
                }
            } else {
                // No modo normal, LED verde
                configurarLEDRGB(0, 255, 0);
            }
        }
        
        // Aguarda 100ms
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// Tarefa para controlar o buzzer com base no modo
void vBuzzerTask(void *pvParameters) {
    TickType_t xLastWakeTime;
    const TickType_t xPeriod = pdMS_TO_TICKS(100); // 100ms
    ModoDados_t modo_dados;
    
    // Inicializa o tempo para uso com vTaskDelayUntil
    xLastWakeTime = xTaskGetTickCount();
    
    while (1) {
        // Verifica se há dados na fila de modo
        if (xQueuePeek(xQueueModo, &modo_dados, 0) == pdPASS) {
            if (modo_dados.modo == MODO_ALERTA) {
                // No modo de alerta, ativa o buzzer com padrão sonoro
                static uint8_t buzzer_estado = 0;
                
                // Padrão sonoro: ativa por 100ms, desativa por 100ms, ciclo de 400ms
                buzzer_estado = (buzzer_estado + 1) % 4;
                if (buzzer_estado < 2) {
                    configurarBuzzer(5); // Volume baixo
                } else {
                    configurarBuzzer(0); // Desligado
                }
            } else {
                // No modo normal, buzzer desligado
                configurarBuzzer(0);
            }
        } else {
            // Se não houver dados na fila, mantenha o buzzer desligado
            configurarBuzzer(0);
        }
        
        // Aguarda precisamente 100ms antes da próxima atualização
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// Tarefa para atualizar a matriz de LEDs
void vMatrixLEDTask(void *pvParameters) {
    const TickType_t xDelayTicks = pdMS_TO_TICKS(100); // 100ms
    ModoDados_t modo_dados;
    static bool piscar = false;
    static int estado_exibido = ESTADO_NORMAL;
    
    while (1) {
        // Verificar se há dados na fila de modo
        if (xQueuePeek(xQueueModo, &modo_dados, 0) == pdPASS) {
            // Sempre atualizar o estado exibido para refletir a mudança
            estado_exibido = modo_dados.estado_matrix;
        } else {
            // Usar variável global como backup
            estado_exibido = g_estado_matrix;
        }
        
        // Se estiver no modo de alerta, piscar
        if (g_modo_atual == MODO_ALERTA) {
            piscar = !piscar;
            if (piscar) {
                npDisplayDigit(estado_exibido); // Exibe o estado atual (ALERTA ou CRÍTICO)
            } else {
                npDisplayDigit(ESTADO_DESLIGADO);
            }
        } else {
            // No modo normal, exibe o estado normal constantemente
            npDisplayDigit(estado_exibido);
        }
        
        vTaskDelay(xDelayTicks);
    }
}

// Tarefa para atualizar o display OLED - versão otimizada
void vDisplayOLEDTask(void *pvParameters) {
    const TickType_t xDelayTicks = pdMS_TO_TICKS(100); // 100ms
    DadosSensor_t dados_sensor;
    ModoDados_t modo_dados;
    char buffer[64];
    
    // Variáveis para armazenar o estado atual
    uint8_t nivel_agua = 0;
    uint8_t volume_chuva = 0;
    int modo = MODO_NORMAL;
    int estado = ESTADO_NORMAL;
    
    while (1) {
        // Limpar o display
        ssd1306_fill(&ssd, false);
        
        // Título do sistema
        ssd1306_draw_string(&ssd, "MONITORAMENTO", 7, 0);
        ssd1306_draw_string(&ssd, "DE CHEIAS", 22, 16);
        
        // Atualizar dados do sensor
        if (xQueuePeek(xQueueDados, &dados_sensor, 0) == pdPASS) {
            nivel_agua = dados_sensor.nivel_agua;
            volume_chuva = dados_sensor.volume_chuva;
        } else {
            // Usar backup global
            nivel_agua = g_nivel_agua;
            volume_chuva = g_volume_chuva;
        }
        
        // Atualizar dados de modo/estado
        if (xQueuePeek(xQueueModo, &modo_dados, 0) == pdPASS) {
            modo = modo_dados.modo;
            estado = modo_dados.estado_matrix;
        } else {
            // Usar backup global
            modo = g_modo_atual;
            estado = g_estado_matrix;
        }
        
        // Exibir valores dos sensores
        sprintf(buffer, "Nivel agua: %d%%", nivel_agua);
        ssd1306_draw_string(&ssd, buffer, 0, 34);
        
        sprintf(buffer, "Volume chuva: %d%%", volume_chuva);
        ssd1306_draw_string(&ssd, buffer, 0, 44);
        
        // Exibir status do sistema - destacar estado crítico
        if (modo == MODO_ALERTA) {
            ssd1306_draw_string(&ssd, "STATUS: ", 0, 54);
            if (estado == ESTADO_CRITICO) {
                // Desenhar "CRITICO" de maneira mais destacada
                ssd1306_draw_string(&ssd, " CRITICO!", 50, 54);
            } else {
                ssd1306_draw_string(&ssd, " ALERTA", 50, 54);
            }
        } else {
            ssd1306_draw_string(&ssd, "STATUS: Normal", 0, 54);
        }
        
        // Enviar dados para o display
        ssd1306_send_data(&ssd);
        
        vTaskDelay(xDelayTicks);
    }
}
