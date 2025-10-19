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

