# Actividades: Control de Motores DC con ESP32 y Puente H

## Práctica 1: Control de dirección de un motor DC con ESP32 y puente H
**Equipo:** Gerardo Esquivel de Luna, José Ismael Guerrero Román.


**Fecha:** 03/10/25


**Descripción:** Se usó un ESP32 y un puente H (L298N) para controlar un motor DC, haciendo que girara hacia adelante y hacia atrás, con pequeñas pausas entre cada cambio de dirección.


## Objetivos:
- **General:** Controlar el giro de un motor DC usando un puente H conectado al ESP32.
- Configurar los pines del ESP32 como salidas digitales.
- Programar el control de sentido (adelante/atrás).
- Implementar pausas de parada entre los cambios de dirección.


## Requisitos:
**Software:** 
- Arduino IDE 2.x o superior.
- Librería ESP32.


**Hardware:**
- ESP32 DevKit.
- Puente H L298N.
- Motor DC de 6 V.
- Fuente externa de 6 V.
- Cables Dupont / protoboard.


## Código:
```
#define in1 27
#define in2 14

void setup() {
  /* Declarar pines como salida */
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

void loop() {
  /* ADELANTE */
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  delay(1000);

  /* ALTO */
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);

  /* ATRÁS */
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  delay(1000);

  /* ALTO */
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  delay(1000);
}
```


## Video
<video width="400" controls>
  <source src="../imgs/motor1.mp4" type="video/mp4">
</video>

## Práctica 2: Control de potencia de un motor DC con ESP32 y PWM
**Equipo:** Gerardo Esquivel de Luna, José Ismael Guerrero Román.

**Fecha:** 10/10/25


**Descripción:** Se usó un ESP32 junto con un puente H (L298N) con una fuente de 6 V para controlar la velocidad de un motor DC usando PWM, haciendo que su potencia bajara poco a poco del máximo al mínimo.


## Objetivos:
- **General:** Ajustar la velocidad de un motor DC usando señales PWM que salen del ESP32.
-  Configurar un canal PWM en el ESP32 con la frecuencia y resolución adecuadas.
-  Implementar un programa que incremente y luego disminuya la velocidad del motor.
-  Observar y analizar la respuesta del motor ante los diferentes niveles de potencia.


## Requisitos:
**Software:** 
- Arduino IDE 2.x o superior.
- Librería del ESP32 instalada desde el Gestor de Placas.


**Hardware:**
- ESP32 DevKit.
- Puente H L298N.
- Motor DC de 6 V.
- Fuente externa de 6 V.
- Protoboard y cables Dupont.


## Código:
```
#define IN1 27
#define IN2 14
#define PWM_PIN 12

// Configuración del canal PWM
#define PWM_CHANNEL 0
#define PWM_FREQ 1000   // Frecuencia de 1 kHz
#define PWM_RES 8       // Resolución de 8 bits (valores 0 a 255)

void setup() {
  /* Configurar pines de salida */
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  /* Configurar canal PWM */
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RES);

  /* Asociar el pin físico al canal PWM */
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);
}

void loop() {
  /* GIRO HACIA ADELANTE */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  /* Aumentar gradualmente la velocidad */
  for (int i = 0; i <= 255; i++) {
    ledcWrite(PWM_CHANNEL, i);
    delay(10);  // Aumento progresivo
  }

  /* Mantener velocidad máxima */
  delay(1000);

  /* Disminuir gradualmente la velocidad */
  for (int i = 255; i >= 0; i--) {
    ledcWrite(PWM_CHANNEL, i);
    delay(10);  // Disminución progresiva
  }

  /* Detener el motor */
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  delay(1000);
}
```

## Video:
<video width="400" controls>
  <source src="../imgs/motor2.mp4" type="video/mp4">
</video>



