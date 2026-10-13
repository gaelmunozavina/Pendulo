// Definición de pines (utilizando la hilera izquierda)
const int RPWM_PIN = 25;       // Pin PWM conectado a RPWM del IBT-2
const int LPWM_PIN = 26;       // Pin PWM conectado a LPWM del IBT-2
const int POT_PIN = 34;        // Pin analógico D34 conectado al potenciómetro

void setup() {
  Serial.begin(115200);

  // Configurar pines del motor como salida
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  
  Serial.println("Sistema listo. Control por potenciómetro en D34 iniciado.");
}

void loop() {
  // Leer el valor del potenciómetro en el ESP32 (rango de 0 a 4095)
  int valorPot = analogRead(POT_PIN);
  
  // Imprimir el valor para monitoreo en el Monitor Serial
  Serial.print("Valor Potenciómetro (D34): ");
  Serial.println(valorPot);

  // Zona muerta en el centro (aprox. 2048) para evitar vibraciones
  if (valorPot > 2150) {
    // Giro hacia un sentido (velocidad proporcional)
    int velocidad = map(valorPot, 2150, 4095, 0, 255);
    analogWrite(RPWM_PIN, velocidad);
    analogWrite(LPWM_PIN, 0);
    Serial.print("Girando Adelante - Velocidad: ");
    Serial.println(velocidad);
    
  } else if (valorPot < 1950) {
    // Giro hacia el sentido contrario (velocidad proporcional)
    int velocidad = map(valorPot, 1950, 0, 0, 255);
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, velocidad);
    Serial.print("Girando Atrás - Velocidad: ");
    Serial.println(velocidad);
    
  } else {
    // Motor detenido en el centro
    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
    Serial.println("Motor Detenido (Centro)");
  }

  delay(50);
}
