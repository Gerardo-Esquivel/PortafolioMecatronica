# Control de LED con ESP32 (botón, Bluetooth e intervalos)

**Equipo:** Gerardo Esquivel de Luna, José Ismael Guerrero Román.


**Fecha:** 19/09/25


**Descripción:** Se programó un ESP32 para controlar un LED de tres formas: con un botón, por comandos vía Bluetooth y haciendo que parpadeara solo a intervalos de tiempo definidos.


## Objetivos:
- **General:** Probar distintas formas de controlar un LED con el ESP32 usando un botón, comunicación inalámbrica y programación de parpadeo.
- Programar el ESP32 para que un botón controle el encendido y apagado inmediato del LED.
- Configurar la comunicación Bluetooth del ESP32 para que el LED responda a comandos enviados desde la terminal ("on" y "off").
- Implementar un programa que haga parpadear el LED en intervalos de tiempo definidos.
- Documentar con fotografías y videos el código y el funcionamiento de cada caso.


## Requisitos:
**Software:**
- Arduino IDE con soporte para ESP32.
- Librerías necesarias para comunicación Bluetooth.
- Monitor serie o aplicación de terminal Bluetooth.


**Hardware:**
- ESP 32.
- LED +  resistencia limitadora (330 Ω a 820 Ω).
- Pulsador (botón).
- Protoboard y cables de conexión.
- Fuente de alimentación USB (5V).


# Programas ESP32: Control de LED
## Control por Botón:
Encender y apagar con un botón físico.
```
const int led = 33;
const int btn = 27;

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);
  pinMode(btn, INPUT);
}

void loop() {
  int estado = digitalRead(btn);
  if (estado == 1) {
    digitalWrite(led, 1);  // LED encendido mientras el botón esté presionado
  } else {
    digitalWrite(led, 0);  // LED apagado cuando no se presiona
  }
}
```
<video width="400" controls>
  <source src="../imgs/ESP32Btn.mp4" type="video/mp4">
</video>


## Control solo por ESP 32 (intervalos de parpadeo):
Encender y apagar mediante código, en intervalos.
```
const int led = 33;

void setup() {
  pinMode(led, OUTPUT);
}

void loop() {
  digitalWrite(led, HIGH);  // LED encendido
  delay(2000);              // Espera 2 segundos
  digitalWrite(led, LOW);   // LED apagado
  delay(2000);              // Espera 2 segundos
}
```
<video width="400" controls>
  <source src="../imgs/ESP32.mp4" type="video/mp4">
</video>


## Control por Bluetooth:
Control del led escribiendo en una consola conectada por Bluetooth
```
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

const int led = 33;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-de-los-PAPITOS"); // Nombre del dispositivo Bluetooth
  pinMode(led, OUTPUT);
}

void loop() {
  if (SerialBT.available()) {
    String mensaje = SerialBT.readString();
    Serial.println("Recibido: " + mensaje);

    if (mensaje == "on") {
      digitalWrite(led, 1);   // LED encendido por comando
    } 
    else if (mensaje == "off") {
      digitalWrite(led, 0);   // LED apagado por comando
    }
  }
  delay(1000);
}
```
<video width="400" controls>
  <source src="../imgs/ESP32Bth.mp4" type="video/mp4">
</video>


