#include <Arduino.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// =========================================================
// 1. CONFIGURACIÓN DE PINES Y CONSTANTES
// =========================================================
#define PIN_POTENCIA_PWM     GPIO_NUM_4   // ENA del Puente H
#define PIN_PUENTE_DIR_1     GPIO_NUM_16  // IN1 del Puente H
#define PIN_PUENTE_DIR_2     GPIO_NUM_17  // IN2 del Puente H
#define PIN_LECTURA_F1       GPIO_NUM_33  // Encoder Fase A
#define PIN_LECTURA_F2       GPIO_NUM_25  // Encoder Fase B
#define PIN_SENSOR_VOLTAJE   32   // Sensor de Voltaje (Analógico)
#define PIN_POTENCIOMETRO    34   // Potenciómetro B10k (Analógico)

// Resolución ajustada para los 360° exactos
#define RESOLUCION_ENCODER   1250.0f

// =========================================================
// 2. VARIABLES GLOBALES Y DE CONTROL
// =========================================================
volatile long conteoAbsolutoPasos = 0;
volatile int32_t desplazamientoEncoder = 0;

// Sistemas de Control de Modos
char fuenteControl = 'A';    // 'A' = Potenciómetro físico, 'B' = Comandos seriales
int subModoOperacion = 0;    // 0 = Neutro/Velocidad, 1 = Posición/Espejo, 2 = Aceleración/Ondas

// Variables para el Modo 1 de Potenciómetro (Espejo)
float potFiltrado = 2048.0;
float potFiltradoAnterior = 2048.0;
float potenciaMotorAnterior = 0.0;

// Variables de Control para el Modo B0 (RPM y Dirección)
float potenciaB0 = 0.0;
float targetRPM = 0.0;      
int direccionB0 = 0;         

// Variables de Control para el Modo B1 (Posición en Grados 0° - 360°)
float targetGrados = 0.0;   
bool b1EnParo = false;       

// Variables para el Generador de Ondas (Modo B2)
int tipoOnda = 0;            // 0 = Seno, 1 = Cuadrada, 2 = Rampa, 3 = Gaussiana
float faseOnda = 0.0;       

// Variable global para almacenar las RPM calculadas actuales
float rpmActuales = 0.0;

// =========================================================
// 3. CONFIGURACIÓN DE PERIFÉRICOS
// =========================================================
void configurarPerifericos() {
    gpio_config_t configPines = {};
    configPines.intr_type = GPIO_INTR_DISABLE;
    configPines.mode = GPIO_MODE_OUTPUT;
    configPines.pin_bit_mask = ((uint64_t)1 << PIN_PUENTE_DIR_1) | ((uint64_t)1 << PIN_PUENTE_DIR_2);
    gpio_config(&configPines);

    pinMode(PIN_SENSOR_VOLTAJE, INPUT);
    pinMode(PIN_POTENCIOMETRO, INPUT);
    pinMode(PIN_LECTURA_F1, INPUT_PULLUP);
    pinMode(PIN_LECTURA_F2, INPUT_PULLUP);

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

    desplazamientoEncoder = 0;
    conteoAbsolutoPasos = 0;
}

void enviarSenalPuenteH(float energiaInyectada) {
    if (energiaInyectada > 0.0f) {
        gpio_set_level(PIN_PUENTE_DIR_1, 1); 
        gpio_set_level(PIN_PUENTE_DIR_2, 0);
    } else if (energiaInyectada < 0.0f) {
        gpio_set_level(PIN_PUENTE_DIR_1, 0); 
        gpio_set_level(PIN_PUENTE_DIR_2, 1);
    } else {
        gpio_set_level(PIN_PUENTE_DIR_1, 0); 
        gpio_set_level(PIN_PUENTE_DIR_2, 0);
    }
    
    int pwmFinal = abs((int)energiaInyectada);
    if (pwmFinal > 1023) pwmFinal = 1023;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwmFinal);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// =========================================================
// 4. TAREA PRINCIPAL (BUCLE DE CONTROL A 50Hz / 20ms)
// =========================================================
void tareaPrincipal(void *parametros) {
    int64_t marcaTiempoAnterior = esp_timer_get_time();
    printf("\n> SISTEMA LISTO:\n");
    printf("  Modos Físicos: A0, A1, A2\n");
    printf("  Modos Comandos:\n");
    printf("    - B0: 'D', 'I', 'C', 'RPM [num]'\n");
    printf("    - B1: 'G [grados]' (0°-360°), 'C' (Paro)\n");
    printf("    - B2: Generador de Ondas ('W SENO', 'W CUAD', 'W RAMPA', 'W GAUSS', 'C')\n\n");

    while (1) {
        // --- LECTURA DE COMANDOS DESDE EL MONITOR SERIAL ---
        if (Serial.available() > 0) {
            String comandoStr = Serial.readStringUntil('\n');
            comandoStr.trim();
            comandoStr.toUpperCase(); 

            if (comandoStr.length() > 0) {
                char letra = comandoStr.charAt(0);

                // Detección de cambios de modo principal (A o B)
                if ((letra == 'A' || letra == 'B') && comandoStr.length() >= 2) {
                    fuenteControl = letra;
                    char numero = comandoStr.charAt(1);
                    
                    if (numero == '0') {
                        subModoOperacion = 0;
                        printf("\n>>> MODO ACTIVADO: %c0 (Velocidad) <<<\n", fuenteControl);
                    } else if (numero == '1') {
                        subModoOperacion = 1;
                        if (fuenteControl == 'B') {
                            desplazamientoEncoder = 0;
                            conteoAbsolutoPasos = 0;
                            targetGrados = 0.0f;
                            b1EnParo = false;
                            printf("\n>>> MODO B1 ACTIVADO: Posición (Calibrado en 0°) <<<\n");
                        } else {
                            printf("\n>>> MODO ACTIVADO: A1 (Espejo) <<<\n");
                        }
                    } else if (numero == '2') {
                        subModoOperacion = 2;
                        if (fuenteControl == 'B') {
                            desplazamientoEncoder = 0;
                            conteoAbsolutoPasos = 0;
                            faseOnda = 0.0f;
                            b1EnParo = false;
                            printf("\n>>> MODO B2 ACTIVADO: Generador de Ondas <<<\n");
                        } else {
                            printf("\n>>> MODO ACTIVADO: A2 (Aceleración) <<<\n");
                        }
                    }

                    if (fuenteControl == 'B' && subModoOperacion == 0) {
                        potenciaB0 = 0.0f; 
                        targetRPM = 0.0f;  
                        direccionB0 = 0;
                    }
                }
                // Control para Modo B0
                else if (fuenteControl == 'B' && subModoOperacion == 0) {
                    if (comandoStr == "D") {
                        direccionB0 = 1; targetRPM = 30.0f;
                        printf("-> [Modo B0] DERECHA | RPM Por Defecto: %.1f\n", targetRPM);
                    } else if (comandoStr == "I") {
                        direccionB0 = -1; targetRPM = 30.0f;
                        printf("-> [Modo B0] IZQUIERDA | RPM Por Defecto: %.1f\n", targetRPM);
                    } else if (comandoStr == "C") {
                        direccionB0 = 0; targetRPM = 0.0f; potenciaB0 = 0.0f;
                        printf("-> [Modo B0] CENTRO / PARO\n");
                    } else if (comandoStr.startsWith("RPM")) {
                        String valStr = comandoStr.substring(3);
                        valStr.trim();
                        targetRPM = valStr.toFloat();
                        printf("-> [Modo B0] RPM Objetivo Actualizado: %.1f\n", targetRPM);
                    }
                }
                // Control para Modo B1
                else if (fuenteControl == 'B' && subModoOperacion == 1) {
                    if (comandoStr == "C") {
                        b1EnParo = true;
                        printf("-> [Modo B1] PARO DE EMERGENCIA ACTIVADO\n");
                    } else if (comandoStr.startsWith("G")) {
                        String valStr = comandoStr.substring(1);
                        valStr.trim();
                        float g = valStr.toFloat();
                        targetGrados = (g < 0.0f) ? 0.0f : ((g > 360.0f) ? 360.0f : g);
                        b1EnParo = false;
                        printf("-> [Modo B1] Posición Objetivo: %.1f°\n", targetGrados);
                    }
                }
                // Control para Modo B2 (Generador de Ondas)
                else if (fuenteControl == 'B' && subModoOperacion == 2) {
                    if (comandoStr == "C") {
                        b1EnParo = true;
                        printf("-> [Modo B2] Generador detenido.\n");
                    } else if (comandoStr.startsWith("W ")) {
                        String tipo = comandoStr.substring(2);
                        tipo.trim();
                        if (tipo == "SENO") { tipoOnda = 0; printf("-> Onda: SENOIDAL\n"); }
                        else if (tipo == "CUAD") { tipoOnda = 1; printf("-> Onda: CUADRADA\n"); }
                        else if (tipo == "RAMPA") { tipoOnda = 2; printf("-> Onda: RAMPA\n"); }
                        else if (tipo == "GAUSS") { tipoOnda = 3; printf("-> Onda: GAUSSIANA\n"); }
                        b1EnParo = false;
                        faseOnda = 0.0f;
                    }
                }
            }
        }

        int64_t ah_tiempo = esp_timer_get_time();
        float deltaT = (ah_tiempo - marcaTiempoAnterior) / 1000000.0f;
        
        if (deltaT >= 0.02f) { // Bucle a 50Hz (20ms)
            marcaTiempoAnterior = ah_tiempo;
            
            int valorVoltajeADC = analogRead(PIN_SENSOR_VOLTAJE);
            int valorPotADC = (fuenteControl == 'A') ? analogRead(PIN_POTENCIOMETRO) : 0;

            int lecturaBloque = desplazamientoEncoder;
            desplazamientoEncoder = 0;

            conteoAbsolutoPasos += lecturaBloque;
            rpmActuales = (((float)lecturaBloque / deltaT) / RESOLUCION_ENCODER) * 60.0f;
            float gradosActuales = ((float)conteoAbsolutoPasos / RESOLUCION_ENCODER) * 360.0f;

            // Auto-reinicio de vuelta completa en B1
            if (subModoOperacion == 1 && fuenteControl == 'B' && targetGrados >= 360.0f) {
                if (fabs(gradosActuales - 360.0f) < 1.5f) {
                    desplazamientoEncoder = 0;
                    targetGrados = 0.0f;
                    gradosActuales = 0.0f;
                    printf("\n>>> [Modo B1] Vuelta completa de 360° alcanzada. Posición reconfigurada a 0° <<<\n");
                }
            }

            float potenciaMotor = 0.0f;

            // =========================================================
            // SUBMODO 0: Control de Velocidad (B0 / A0)
            // =========================================================
            if (subModoOperacion == 0) {
                if (fuenteControl == 'A') {
                    int centroADC = 2048, zonaMuerta = 100;
                    if (abs(valorPotADC - centroADC) < zonaMuerta) potenciaMotor = 0.0f;
                    else if (valorPotADC > centroADC) potenciaMotor = ((float)(valorPotADC - centroADC) / (4095.0f - centroADC)) * 1023.0f;
                    else potenciaMotor = -((float)(centroADC - valorPotADC) / centroADC) * 1023.0f;
                } else {
                    if (direccionB0 == 0 || targetRPM == 0.0f) {
                        potenciaMotor = 0.0f;
                    } else {
                        float errorRPM = targetRPM - fabs(rpmActuales);
                        float ajuste = errorRPM * 2.0f; 
                        if (direccionB0 == 1) { 
                            if (potenciaB0 >= 0.0f) potenciaB0 = -300.0f; 
                            potenciaB0 -= ajuste;
                            if (potenciaB0 < -1023.0f) potenciaB0 = -1023.0f;
                            if (potenciaB0 > -80.0f) potenciaB0 = -80.0f;
                        } else { 
                            if (potenciaB0 <= 0.0f) potenciaB0 = 300.0f; 
                            potenciaB0 += ajuste;
                            if (potenciaB0 > 1023.0f) potenciaB0 = 1023.0f;
                            if (potenciaB0 < 80.0f) potenciaB0 = 80.0f;
                        }
                        potenciaMotor = potenciaB0; 
                    }
                }
            }
            // =========================================================
            // SUBMODO 1: Control de Posición Fija (B1 / A1)
            // =========================================================
            else if (subModoOperacion == 1) {
                if (fuenteControl == 'A') {
                    potFiltrado = (0.4f * (float)valorPotADC) + (0.6f * potFiltrado);
                    float potenciaDeseada = 0.0f;
                    if (valorPotADC > 40 && valorPotADC < 4055) {
                        float deltaMovimiento = potFiltrado - potFiltradoAnterior;
                        if (fabs(deltaMovimiento) > 0.5f) {
                            potenciaDeseada = deltaMovimiento * 22.0f;
                            if (fabs(potenciaDeseada) > 0.0f && fabs(potenciaDeseada) < 130.0f)
                                potenciaDeseada = (potenciaDeseada > 0.0f) ? 130.0f : -130.0f;
                        }
                    }
                    potFiltradoAnterior = potFiltrado;
                    potenciaMotor = (0.3f * potenciaDeseada) + (0.7f * potenciaMotorAnterior);
                    potenciaMotorAnterior = potenciaMotor;
                } 
                else if (fuenteControl == 'B') {
                    if (b1EnParo) {
                        potenciaMotor = 0.0f; 
                    } else {
                        float errorGrados = targetGrados - gradosActuales;
                        if (fabs(errorGrados) < 1.5f) {
                            potenciaMotor = 0.0f; 
                        } else {
                            potenciaMotor = -errorGrados * 12.0f; 
                            if (potenciaMotor > 0.0f && potenciaMotor < 200.0f) potenciaMotor = 200.0f;
                            if (potenciaMotor < 0.0f && potenciaMotor > -200.0f) potenciaMotor = -200.0f;
                            if (potenciaMotor > 1023.0f) potenciaMotor = 1023.0f;
                            if (potenciaMotor < -1023.0f) potenciaMotor = -1023.0f;
                        }
                    }
                }
            }
            // =========================================================
            // SUBMODO 2: Aceleración (A2) / Generador de Ondas (B2)
            // =========================================================
            else if (subModoOperacion == 2) {
                if (fuenteControl == 'A') {
                    int potInvertido = 4095 - valorPotADC;
                    if (potInvertido > 30) {
                        potenciaMotor = -((float)potInvertido / 4095.0f) * 1023.0f;
                    } else {
                        potenciaMotor = 0.0f;
                    }
                } 
                else if (fuenteControl == 'B') {
                    if (b1EnParo) {
                        potenciaMotor = 0.0f;
                    } else {
                        faseOnda += 0.03f; 
                        if (faseOnda > 6.283185f) faseOnda = 0.0f;

                        if (tipoOnda == 0) {
                            targetGrados = 180.0f + 180.0f * sin(faseOnda);
                        } 
                        else if (tipoOnda == 1) {
                            targetGrados = (sin(faseOnda) >= 0.0f) ? 360.0f : 0.0f;
                        } 
                        else if (tipoOnda == 2) {
                            targetGrados = (faseOnda < 3.14159f) ? (faseOnda * (360.0f / 3.14159f)) : (728.318f - faseOnda * (360.0f / 3.14159f));
                        } 
                        else if (tipoOnda == 3) {
                            float x = (faseOnda - 3.14159f) * 1.2f;
                            targetGrados = 360.0f * exp(-0.5f * (x * x));
                        }

                        float errorGrados = targetGrados - gradosActuales;
                        if (fabs(errorGrados) < 1.0f) {
                            potenciaMotor = 0.0f;
                        } else {
                            potenciaMotor = -errorGrados * 14.0f; 
                            if (potenciaMotor > 0.0f && potenciaMotor < 200.0f) potenciaMotor = 200.0f;
                            if (potenciaMotor < 0.0f && potenciaMotor > -200.0f) potenciaMotor = -200.0f;
                            if (potenciaMotor > 1023.0f) potenciaMotor = 1023.0f;
                            if (potenciaMotor < -1023.0f) potenciaMotor = -1023.0f;
                        }
                    }
                }
            }

            enviarSenalPuenteH(potenciaMotor);

            // --- IMPRESIÓN DETALLADA RECUPERADA PARA MONITOR Y PLOTTER ---
            static int contadorImpresion = 0;
            contadorImpresion++;
            if (contadorImpresion >= 15) { // Cada ~300ms
                contadorImpresion = 0;
                
                if (fuenteControl == 'A') {
                    printf(">Modo:%c | SubModo:%d | ValorPot:%d | Potencia:%.2f | RPM:%.1f\n", fuenteControl, subModoOperacion, valorPotADC, potenciaMotor, rpmActuales);
                } else {
                    if (subModoOperacion == 0) {
                        printf(">Modo:B0 | TargetRPM:%.1f | PotCmd:%.2f | RPM_Real:%.1f\n", targetRPM, potenciaMotor, rpmActuales);
                    } else if (subModoOperacion == 1) {
                        printf(">Modo:B1 | TargetDeg:%.1f° | RealDeg:%.1f° | Pot:%.2f | Paro:%d\n", targetGrados, gradosActuales, potenciaMotor, b1EnParo);
                    } else if (subModoOperacion == 2) {
                        printf(">Modo:B2 (Ondas) | TargetDeg:%.1f° | RealDeg:%.1f° | Pot:%.2f\n", targetGrados, gradosActuales, potenciaMotor);
                    }
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =========================================================
// 5. SETUP Y LOOP DE ARDUINO
// =========================================================
void setup() {
    Serial.begin(115200);
    configurarPerifericos();
    xTaskCreatePinnedToCore(tareaPrincipal, "tarea_principal", 4096, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelete(NULL);
}
