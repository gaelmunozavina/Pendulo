#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_timer.h"

// =========================================================
// 1. CONFIGURACIÓN DEL USUARIO (CAMBIAR Y FLASHEAR)
// =========================================================

// Elige el modo de trabajo: 0 = Curvas de Velocidad, 1 = Control de Péndulo con Potenciómetro
#define MODO_DE_TRABAJO 1

// --- Si usas Modo 0 (Curvas) ---
#define TRAYECTORIA_ELEGIDA 0 

// =========================================================
// 2. CONFIGURACIÓN DE PINES (HARDWARE)
// =========================================================
#define PIN_POTENCIA_PWM     GPIO_NUM_4   
#define PIN_PUENTE_DIR_1     GPIO_NUM_16  
#define PIN_PUENTE_DIR_2     GPIO_NUM_17  
#define PIN_LECTURA_F1       GPIO_NUM_33  
#define PIN_LECTURA_F2       GPIO_NUM_25  
#define PIN_SENSOR_VOLTAJE   32   // Pin analógico para el sensor de voltaje
#define PIN_POTENCIOMETRO    34   // Pin analógico para controlar el ángulo con el potenciómetro

// Parámetros físicos
#define RESOLUCION_ENCODER 937.0f
#define CICLO_SEGUNDOS     10.0f
#define ENERGIA_INERCIAL   120.0f  // Fuerza inicial para vencer fricción

// =========================================================
// 3. VARIABLES DE CONTROL
// =========================================================
pcnt_unit_handle_t moduloContador = NULL;
volatile long conteoAbsolutoPasos = 0;
bool calibracionCompletada = true; 

// PID Velocidad
float gananciaP_Vel = 1.5f, gananciaI_Vel = 4.0f, gananciaD_Vel = 0.02f;
float acumuladorErrorVel = 0.0f, velocidadAnterior = 0.0f, velocidadSuavizada = 0.0f;

// PID Posición (Ganancia Proporcional alta para vencer fricción)
float gananciaP_Pos = 15.0f, gananciaD_Pos = 0.5f;
long errorPosicionAnterior = 0;

// =========================================================
// 4. FUNCIONES MATEMÁTICAS Y HARDWARE
// =========================================================
float calcularCurvaMatematica(int tipo, float tiempoActual) {
    float velMax = 280.0f;
    switch (tipo) {
        case 0: return 200.0f; 
        case 1: return (velMax / CICLO_SEGUNDOS) * tiempoActual; 
        case 2: { 
            float t1 = CICLO_SEGUNDOS * 0.3f, t2 = CICLO_SEGUNDOS * 0.7f;
            float pendiente = velMax / t1;
            if (tiempoActual < t1) return pendiente * tiempoActual;
            if (tiempoActual < t2) return velMax;
            return velMax - pendiente * (tiempoActual - t2);
        }
        case 3: return 150.0f + 100.0f * sinf(2.0f * M_PI * 0.5f * tiempoActual); 
        case 4: return (fmodf(tiempoActual, 2.0f) < 1.0f) ? 50.0f : 250.0f;       
        case 5: return velMax * powf(tiempoActual / CICLO_SEGUNDOS, 2.0f);        
        default: return 0.0f;
    }
}

void configurarPerifericos() {
    gpio_config_t configPines = {};
    configPines.intr_type = GPIO_INTR_DISABLE;
    configPines.mode = GPIO_MODE_OUTPUT;
    configPines.pin_bit_mask = (1ULL << PIN_PUENTE_DIR_1) | (1ULL << PIN_PUENTE_DIR_2);
    gpio_config(&configPines);
    
    // Configuración de pines analógicos
    pinMode(PIN_SENSOR_VOLTAJE, INPUT);
    pinMode(PIN_POTENCIOMETRO, INPUT);

    ledc_timer_config_t timerPWM = {};
    timerPWM.speed_mode = LEDC_LOW_SPEED_MODE;
    timerPWM.duty_resolution = LEDC_TIMER_10_BIT;
    timerPWM.timer_num = LEDC_TIMER_0;
    timerPWM.freq_hz = 20000;
    timerPWM.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timerPWM);

    ledc_channel_config_t canalPWM = {};
    canalPWM.gpio_num = PIN_POTENCIA_PWM;
    canalPWM.speed_mode = LEDC_LOW_SPEED_MODE;
    canalPWM.channel = LEDC_CHANNEL_0;
    canalPWM.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&canalPWM);

    pcnt_unit_config_t pcntConfig = {.low_limit = -32000, .high_limit = 32000};
    ESP_ERROR_CHECK(pcnt_new_unit(&pcntConfig, &moduloContador));
    pcnt_glitch_filter_config_t filtroPcnt = {.max_glitch_ns = 1000};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(moduloContador, &filtroPcnt));

    pcnt_chan_config_t confFase1 = {.edge_gpio_num = PIN_LECTURA_F1, .level_gpio_num = PIN_LECTURA_F2};
    pcnt_channel_handle_t canal1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(moduloContador, &confFase1, &canal1));
    
    pcnt_chan_config_t confFase2 = {.edge_gpio_num = PIN_LECTURA_F2, .level_gpio_num = PIN_LECTURA_F1};
    pcnt_channel_handle_t canal2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(moduloContador, &confFase2, &canal2));

    pcnt_channel_set_edge_action(canal1, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(canal1, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
    pcnt_channel_set_edge_action(canal2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(canal2, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    ESP_ERROR_CHECK(pcnt_unit_enable(moduloContador));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(moduloContador));
    ESP_ERROR_CHECK(pcnt_unit_start(moduloContador));
}

void enviarSenalPuenteH(float energiaInyectada) {
    if (energiaInyectada > 0) {
        gpio_set_level(PIN_PUENTE_DIR_1, 1); gpio_set_level(PIN_PUENTE_DIR_2, 0);
    } else if (energiaInyectada < 0) {
        gpio_set_level(PIN_PUENTE_DIR_1, 0); gpio_set_level(PIN_PUENTE_DIR_2, 1);
    } else {
        gpio_set_level(PIN_PUENTE_DIR_1, 0); gpio_set_level(PIN_PUENTE_DIR_2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs((int)energiaInyectada));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// =========================================================
// 5. LAZO CERRADO PRINCIPAL
// =========================================================
void tareaLazoCerrado(void *parametros) {
    int64_t tiempoArranque = esp_timer_get_time();
    int64_t marcaTiempoAnterior = esp_timer_get_time();

    printf("\n>Sistema_Iniciado: Modo 1 con Potenciometro Ajustado\n");

    while (1) {
        int64_t ahora = esp_timer_get_time();
        float deltaT = (ahora - marcaTiempoAnterior) / 1000000.0f;
        
        if (deltaT >= 0.02f) { // Lazo a 50Hz (20ms)
            marcaTiempoAnterior = ahora;
            
            // --- LECTURA DEL SENSOR DE VOLTAJE (Pin 32) ---
            int valorVoltajeADC = analogRead(PIN_SENSOR_VOLTAJE);
            float voltajeReal = ((float)valorVoltajeADC / 4095.0f) * 3.3f * 5.0f;

            // --- LECTURA DEL POTENCIÓMETRO PARA EL ÁNGULO (Pin 34) ---
            int valorPotADC = analogRead(PIN_POTENCIOMETRO);
            // Mapeo ajustado considerando el giro físico parcial (~270°) del potenciómetro
            float anguloDeseado = (float)valorPotADC * (360.0f / 3000.0f);
            if (anguloDeseado > 360.0f) anguloDeseado = 360.0f;

            int lecturaBloque = 0;
            pcnt_unit_get_count(moduloContador, &lecturaBloque);
            pcnt_unit_clear_count(moduloContador);
            conteoAbsolutoPasos += lecturaBloque;
            
            float energiaCalculada = 0.0f;

            // ---------------------------------------------------
            // RAMA A: MODO CURVAS DE VELOCIDAD
            // ---------------------------------------------------
            if (MODO_DE_TRABAJO == 0) {
                float segTranscurridos = (ahora - tiempoArranque) / 1000000.0f;
                float metaRPM = 0.0f;
                
                if (segTranscurridos <= CICLO_SEGUNDOS) {
                    metaRPM = calcularCurvaMatematica(TRAYECTORIA_ELEGIDA, segTranscurridos);
                } else if (segTranscurridos > (CICLO_SEGUNDOS + 2.0f)) {
                    tiempoArranque = esp_timer_get_time(); 
                    acumuladorErrorVel = 0.0f; 
                    velocidadSuavizada = 0.0f; 
                }

                float rpmInstantanea = ((float)lecturaBloque / RESOLUCION_ENCODER) * (60.0f / deltaT);
                velocidadSuavizada = (0.3f * rpmInstantanea) + (0.7f * velocidadSuavizada);
                
                float divergencia = metaRPM - fabsf(velocidadSuavizada);
                float prop = gananciaP_Vel * divergencia;
                float deriv = -gananciaD_Vel * ((fabsf(velocidadSuavizada) - fabsf(velocidadAnterior)) / deltaT);
                velocidadAnterior = velocidadSuavizada;
                
                energiaCalculada = prop + (gananciaI_Vel * acumuladorErrorVel) + deriv;

                if (metaRPM > 0 && fabsf(velocidadSuavizada) < 5.0f && energiaCalculada > 0) energiaCalculada += ENERGIA_INERCIAL;
                
                if (energiaCalculada > 1023.0f) { energiaCalculada = 1023.0f; if (divergencia < 0) acumuladorErrorVel += divergencia * deltaT; }
                else if (energiaCalculada < -1023.0f) { energiaCalculada = -1023.0f; if (divergencia > 0) acumuladorErrorVel += divergencia * deltaT; }
                else { acumuladorErrorVel += divergencia * deltaT; }

                static int contadorImpresionVel = 0;
                contadorImpresionVel++;
                if (contadorImpresionVel >= 15) {
                    contadorImpresionVel = 0;
                    printf(">Voltaje_Entrada:%.2f\n", voltajeReal);
                    printf(">Objetivo_RPM:%.2f\n", metaRPM);
                    printf(">Medicion_RPM:%.2f\n", fabsf(velocidadSuavizada));
                    printf(">Potencia_Motor:%.2f\n", energiaCalculada);
                }
            } 
            
            // ---------------------------------------------------
            // RAMA B: MODO PÉNDULO CON CONTROL DE POTENCIÓMETRO
            // ---------------------------------------------------
            else if (MODO_DE_TRABAJO == 1) {
                long pasosDestino = (long)((anguloDeseado / 360.0f) * RESOLUCION_ENCODER);
                long divergenciaPos = pasosDestino - conteoAbsolutoPasos;
                
                float prop = gananciaP_Pos * (float)divergenciaPos;
                float deriv = gananciaD_Pos * ((float)(divergenciaPos - errorPosicionAnterior) / deltaT);
                errorPosicionAnterior = divergenciaPos;
                
                energiaCalculada = prop + deriv;
                if (energiaCalculada > 1023.0f) energiaCalculada = 1023.0f;
                if (energiaCalculada < -1023.0f) energiaCalculada = -1023.0f;

                float gradosActuales = ((float)conteoAbsolutoPasos / RESOLUCION_ENCODER) * 360.0f;

                // --- CONTROL DE VELOCIDAD DEL MONITOR SERIAL ---
                static int contadorImpresion = 0;
                contadorImpresion++;
                
                if (contadorImpresion >= 15) { // Imprime cada 15 ciclos (aprox. cada 300 ms)
                    contadorImpresion = 0;
                    
                    printf(">Voltaje_Entrada:%.2f\n", voltajeReal);
                    printf(">Meta_Grados:%.2f\n", anguloDeseado);
                    printf(">Grados_Fisicos:%.2f\n", gradosActuales);
                    printf(">Potencia_Motor:%.2f\n", energiaCalculada);
                }
            }

            enviarSenalPuenteH(energiaCalculada);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    configurarPerifericos();
    xTaskCreatePinnedToCore(tareaLazoCerrado, "tarea_motriz", 4096, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
