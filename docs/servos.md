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


