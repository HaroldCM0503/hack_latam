Resumen de la Presentación
1. El Problema — Peligro Invisible
Un fragmento de 10 gramos viajando a velocidad orbital equivale a un refrigerador cayendo desde el Costanera Center. Más de 100 millones de fragmentos orbitan la Tierra hoy, sin forma de verlos llegar.

2. Entorno Hostil — Coexistencia forzada
Cada satélite activo convive con ~76 escombros cercanos a 28.000 km/h. Sin escudos. Sin defensas activas. Sin margen de error.

3. Amenaza Inminente
Solo Starlink realizó 300.000 maniobras evasivas en 2025 — una colisión esquivada cada 2 minutos. Cada maniobra fue posible únicamente porque alguien más rastreó el objeto desde tierra.

4. Síndrome de Kessler
Una sola colisión puede desencadenar una reacción en cadena que haga la órbita baja impenetrable por décadas o siglos. El modelo matemático (Liou & Johnson, 2006) indica que ya cruzamos el punto crítico: aunque dejáramos de lanzar hoy, la basura seguiría creciendo.

5. Infraestructura en Riesgo
Más de 15.000 satélites activos sostienen GPS, sincronización financiera, meteorología, telecomunicaciones, defensa y conectividad para 3.000+ millones de personas. Valor estimado: $700B+ en infraestructura orbital.

6. Por qué las soluciones actuales no sirven
Solo existen prototipos (Astroscale, ClearSpace-1) — cero misiones comerciales
Económicamente inviable: el satélite capturador se quema junto al escombro
Efecto piñata: capturar un objeto degradado lo rompe en 10.000 fragmentos irrastreables
Arma de doble uso: cualquier satélite que atrapa a otro es, por definición, un arma espacial
7. Cambio de Paradigma
"No lancemos barredoras. Encendamos las luces."

La solución no es remover la basura — es detectarla con precisión para poder esquivarla.

8. La Solución — Forward Scattering
Convertir la red de satélites existente en un sistema masivo de radares biestáticos mediante Forward Scattering: las señales de radio que miles de satélites ya emiten son interceptadas y analizadas para detectar escombros sin lanzar un solo satélite nuevo. El principio físico es el mismo que el WiFi Sensing doméstico, aplicado al espacio.

9. Canales de Detección
Tres fuentes de señal ya disponibles, reutilizadas como red de vigilancia:

GNSS → LEO: GPS, Galileo, GLONASS, BeiDou (L-band 1.2–1.6 GHz)
Starlink ↔ Starlink: malla de 6.000+ nodos con ISL óptico y Ku/Ka band
Iridium ↔ Iridium: 66 satélites × 4 cross-links a 23 GHz, cobertura polo a polo 24/7
10. Cómo Funciona — El flujo
Detectar — identificar fragmentos invisibles para radares terrestres
Observar — medir velocidad y dirección del objeto
Predecir — calcular trayectoria futura con precisión milimétrica
Desviar — enviar comandos automáticos de evasión al satélite en riesgo
11. Validación Empírica
Se montó una red de ESP32 con WiFi y se logró observar posición, velocidad y trayectoria de múltiples objetos cruzando entre los nodos — el mismo principio físico, con hardware 10× menos preciso que un radar orbital real y con interferencia terrestre. Si funciona aquí, en órbita la sensibilidad será órdenes de magnitud superior.

12. Escalabilidad
Cuanto más crece la constelación de satélites, más fuerte se vuelve la solución: más nodos = más cruces de señal = más resolución. Además, el modelo de ML aprende con cada detección.

13. Estado actual
Las simulaciones están hechas. El software corre. El programa existe.

14. Equipo — Cierre
"No podemos permitirnos perder la nueva frontera de la humanidad cuando recién estamos empezando a conquistarla."

Agustín Arévalo · Nicolás Quintana · Benjamín Varas · Harold Camas