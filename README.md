# Debri net

Debri net es un innovador sistema de alerta temprana diseñado para prevenir colisiones orbitales que aprovecha redes satelitales preexistentes (como GNSS, Starlink e Iridium) como radares biestáticos gigantes. Empleando la técnica de *Forward Scattering*, nuestra plataforma analiza las perturbaciones en las señales de radio entre los nodos orbitales para detectar, medir y predecir las trayectorias de escombros espaciales invisibles, lo que permite enviar comandos automáticos de evasión sin la necesidad de lanzar nuevo hardware ni misiones de barrido al espacio.

## El Problema: El Peligro Invisible

Actualmente hay más de 100 millones de fragmentos de basura espacial orbitando la Tierra a velocidades cercanas a los 28.000 km/h. Un fragmento de apenas 10 gramos viajando a velocidad orbital equivale, en energía cinética, a un refrigerador cayendo desde el Costanera Center. Los satélites activos no tienen escudos y conviven en un entorno hostil donde no hay margen de error.

Solo en el año 2025, constelaciones como Starlink tuvieron que realizar 300.000 maniobras evasivas (una colisión esquivada cada 2 minutos). Si ocurre una colisión importante, podríamos desatar el temido **Síndrome de Kessler**: una reacción en cadena de colisiones que vuelva impenetrable la órbita baja por décadas o siglos, arruinando la infraestructura orbital valorada en cientos de billones de dólares de la que dependen 3.000+ millones de personas para sus comunicaciones, GPS, meteorología y defensa.

## Por Qué las Soluciones Actuales Fallan

Las propuestas de capturar la basura espacial activamente sufren de problemas insalvables:
- **Económicamente inviables**: Muchos satélites capturadores deben quemarse en la atmósfera junto con el escombro capturado.
- **El Efecto Piñata**: Intentar capturar un satélite viejo y degradado puede destruirlo en el proceso, generando 10.000 fragmentos inrastreables nuevos.
- **Armas de Doble Uso**: Cualquier tecnología capaz de atrapar un satélite es, por definición, un arma espacial.

Nuestra filosofía cambia el paradigma: **"No lancemos barredoras. Encendamos las luces."**
La solución no es tratar de remover la basura — es detectarla con precisión para poder esquivarla.

## La Solución: Forward Scattering

Debri net convierte la vasta red de satélites existentes en un sistema masivo de vigilancia. Utilizamos tres canales de detección de señales de radio que ya cruzan el espacio constantemente:
1. **GNSS a LEO**: GPS, Galileo, GLONASS, BeiDou (Banda L).
2. **Starlink a Starlink**: Una malla de 6.000+ nodos con ISL óptico y banda Ku/Ka.
3. **Iridium a Iridium**: Cobertura constante polo a polo 24/7.

Al interceptar y analizar estas señales, detectamos las perturbaciones (*Forward Scattering*) causadas por escombros cruzando la línea de visión. El flujo de funcionamiento del sistema cuenta con 4 pasos clave:
1. **Detectar**: Identificamos fragmentos invisibles a los radares terrestres.
2. **Observar**: Medimos la velocidad y dirección del objeto basándonos en la alteración de la señal.
3. **Predecir**: Calculamos su trayectoria futura con precisión milimétrica mediante modelos matemáticos avanzados.
4. **Desviar**: Emitimos comandos automatizados al satélite en riesgo para evadir la colisión inminente.

## Validación Empírica

Para probar este principio físico en la Tierra, creamos un prototipo empírico con microcontroladores ESP32 analizando datos de *Channel State Information* (CSI) de redes WiFi. Logramos observar la posición, velocidad y trayectoria de objetos físicos que cruzan de forma iterativa entre nuestros nodos transmisores y receptores.
El sistema diferencia las perturbaciones en la señal con extrema precisión basándose en métricas como la coherencia de subportadoras y desviaciones relativas de amplitud. Si el mismo principio físico funciona en tierra, con hardware 10 veces menos preciso que un radar orbital y con altísima interferencia electromagnética, en el vacío del espacio la sensibilidad y alcance serán órdenes de magnitud superiores.

## Estructura del Proyecto

Nuestro repositorio está dividido en módulos fundamentales correspondientes a nuestra Prueba de Concepto (PoC) terrestre:

- `/firmware`: Código C/C++ para los transmisores y receptores ESP32 encargados de emitir y recolectar el flujo constante de datos brutos de las ondas de radio (WiFi CSI).
- `/fusion`: Backend en Python que procesa los streams UDP en vivo. Aplica filtros avanzados, detectores de caídas de señal (*SignalDropDetector*) y algoritmos de filtrado para diferenciar el movimiento humano del paso de objetos de prueba rápidos a través de la red sensora.
- `/dashboard`: Interfaz web interactiva construida con Vite y herramientas de modelado 3D, que permite visualizar de manera inmersiva los nodos satelitales (como los módulos HB100 de nuestra simulación), las órbitas y el mapeo en tiempo real de los escombros rastreados.

## Equipo

Debri net ha sido ideado y desarrollado para la Hackathon LATAM por:
- Agustín Arévalo
- Nicolás Quintana
- Benjamín Varas
- Harold Camas

> *"No podemos permitirnos perder la nueva frontera de la humanidad cuando recién estamos empezando a conquistarla."*
