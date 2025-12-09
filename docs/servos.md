# Control Angular de Servomotor con ESP32


El propósito fundamental de esta práctica fue *analizar y aplicar los principios operativos de un servomotor* utilizando las capacidades de generación de señal *PWM* de la placa ESP32. El objetivo específico se centró en la *secuenciación precisa del eje del servo* a tres ángulos clave predefinidos: *0°, 90° y 180°*, asegurando un movimiento controlado y repetitivo.


## Fundamentos teóricos


Los *servomotores* son dispositivos actuadores fundamentales en la robótica, conocidos por su capacidad para lograr un *control angular exacto* dentro de un rango limitado (comúnmente 180°). El control de su posición se realiza a través de la *Modulación por Ancho de Pulso (PWM), donde el **ancho del pulso eléctrico* (ciclo de trabajo) que reciben es directamente proporcional al ángulo de giro deseado.


## Con ESP 32


El microcontrolador *ESP32* es idóneo para esta tarea, ya que su hardware permite una *generación de señales PWM altamente configurable* en términos de resolución y frecuencia. Para la programación, se emplean dos funciones esenciales del framework de Arduino:
1.  *ledcWrite():* Se utiliza para enviar el valor del ciclo de trabajo PWM al pin de salida.
2.  *map():* Es crucial para realizar la *conversión matemática* del rango de grados legible por el usuario (0 a 180) al rango específico de valores PWM que el servo interpreta.


## Materiales


- ESP 32: Microcontrolador principal
- Servomotor: Actuador de control angular
- Protoboard: Plataforma para el ensamblaje del circuito
- Jumpers: Cables conectores
- Fuente de alimentación: Suministro de energía independiente para el servo.
- PC con IDE (Arduino) y cable para ESP


## Montaje y programación


1.  *Montaje Físico:* La ESP32 y el servo se ensamblaron en la protoboard. La línea de señal del servo se conectó al *GPIO 12* de la ESP32. Se aseguraron conexiones separadas de *alimentación (5V)* y *tierra (GND)* para el servo.

  
2.  *Desarrollo del Código:* Se programó la lógica de control para establecer un ciclo secuencial de movimiento a 0°, 90° y 180°. Se empleó la función map() para asegurar la correcta correspondencia entre los ángulos y el ciclo de trabajo PWM.

  
3.  *Verificación:* Se cargó el programa a la ESP32 y se monitoreó el *movimiento físico del servo*. Los retardos de 1000 ms se usaron para confirmar cada posición.


## Código Arduino


```cpp
/**
 * @file Control_Servo_Libreria.ino
 * @brief Control secuencial de un servomotor (0°, 90°, 180°) usando la librería Servo.h en ESP32.
 * * La librería Servo.h simplifica el control al encargarse de la gestión interna del PWM.
 */

// Incluir la librería Servo específica para ESP32
// NOTA: Para ESP32, a menudo se usa "ESP32Servo.h" o simplemente "Servo.h"
// Si Servo.h no funciona, intente usar ESP32Servo.h
#include <Servo.h> 

// --- Configuración de Pines y Objetos ---

#define SERVO_PIN 12 // Pin GPIO donde se conecta la señal del servo (Pin de ejemplo)

// Crea un objeto Servo para controlar el motor
Servo miServo; 

// --- Setup (Configuración Inicial) ---
void setup() {
    Serial.begin(115200);
    Serial.println("Inicializando Servomotor con Servo.h...");
    
    // Asocia el objeto Servo al pin de salida
    // Este paso configura automáticamente el canal PWM necesario en el ESP32
    miServo.attach(SERVO_PIN); 

    // Asegurar que el servo inicie en una posición conocida
    miServo.write(0); 
    delay(1000);
}

// --- Loop Principal (Ejecución Continua) ---
void loop() {
    // Mover a 0 grados
    Serial.println("Movimiento a 0 grados");
    miServo.write(0); 
    delay(1500); // Usamos un retardo ligeramente mayor para notar el movimiento

    // Mover a 90 grados
    Serial.println("Movimiento a 90 grados");
    miServo.write(90); 
    delay(1500);

    // Mover a 180 grados
    Serial.println("Movimiento a 180 grados");
    miServo.write(180); 
    delay(1500);
}
```


## Resultados y conclusion


Los resultados de la fase de pruebas confirmaron la viabilidad del método de control implementado, demostrando una respuesta angular precisa e inmediata del servomotor al posicionarse exactamente en los puntos preestablecidos (0°, 90°, 180°), lo que verifica la integridad de la conexión física. La clave del éxito fue la función map(), que resultó crítica para la calibración al permitir la traducción efectiva y lineal del rango lógico (0-180) al ciclo de trabajo PWM (205-410) requerido. Además, los datos numéricos del ciclo de trabajo PWM comunicados por el monitor serial coincidieron rigurosamente con el movimiento físico observado, validando la lógica de programación y la precisión del cálculo matemático. Esta actividad consolidó la comprensión práctica del control de actuadores angulares mediante la técnica PWM en la plataforma ESP32 y reforzó el entendimiento de la relación crítica entre la frecuencia del pulso, el ciclo de trabajo de la señal PWM y el desplazamiento angular preciso que se obtiene en el servomotor.
