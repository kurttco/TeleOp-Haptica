# Teleoperación con Sensado de Fuerza — xArm Lite 6

**TE3001B · Fundamentación de Robótica**  
Computational Robotics Lab · Tecnológico de Monterrey · 2026  
Prof. Alberto Muñoz

---

## Descripción

Sistema de teleoperación maestro-esclavo entre dos manipuladores **xArm Lite 6** con detección de contacto por fuerza usando sensores artesanales de esponja conductora de grafito y microcontroladores ESP32.

**Video de demostración:** (https://youtu.be/9yoCLF5ZJVQ))  

---

## Características principales

- Control cartesiano del brazo maestro mediante sensores de presión en 6 direcciones (R / L / Fwd / Back / Top / Bottom)
- Esclavo que replica el movimiento del maestro en tiempo real vía ROS2
- Detección de contacto con 5 sensores de fuerza en el efector del esclavo
- Bloqueo sincronizado de ambos brazos al superar umbral de 3.0 N (con histéresis)
- Jacobiano geométrico calculado dinámicamente desde el árbol TF de ROS2
- Pseudoinversa amortiguada (DLS, λ=0.01) para cinemática inversa diferencial
- Visualización en tiempo real de Fx, Fy, Fz y |F|

---

## Estructura del repositorio

```
teleop_xarm_lite6/
├── code/
│   ├── master_input.py        # Control cartesiano del brazo maestro
│   ├── dual_mirror.py         # Espejo M→E + detección de colisión
│   ├── force_reader5.py       # Lectura serial ESP32 esclavo → /slave/force
│   └── force_display5.py      # Visualización Fx/Fy/Fz en tiempo real
├── firmware/
│   └── slave_force_sensor.ino # Firmware ESP32 para los 5 sensores del esclavo
├── data/
│   ├── sim/                   # Logs de experimentos en simulación (.xlsx)
│   │   ├── log_experimento1.xlsx
│   │   ├── log_experimento2.xlsx
│   │   └── log_experimento3.xlsx
│   └── real/                  # Logs de experimentos con robots físicos (.xlsx)
│       ├── log_real1.xlsx
│       ├── log_real2.xlsx
│       └── log_real3.xlsx
├── figures/                   # Gráficas generadas del análisis
│   ├── diagram_bloques.png
│   ├── fig1_tracking.png
│   ├── fig2_error.png
│   ├── fig3_metrics.png
│   └── fig4_summary.png
└── docs/
    └── reporte_final.tex      # Reporte IEEE en LaTeX
```

---

## Requisitos

### Software
- Ubuntu 22.04 / 24.04
- ROS2 Jazzy (`ros-jazzy-desktop`)
- xArm ROS2 packages: [`xarm_ros2`](https://github.com/xArm-developer/xarm_ros2)
- Python 3.10+

### Dependencias Python
```bash
pip install pyserial numpy matplotlib rclpy
```

### Hardware
- 2× xArm Lite 6 (UFACTORY)
- 2× ESP32 (NodeMCU-32S o similar)
- 11× sensores de esponja-grafito artesanales (6 maestro + 5 esclavo)
- 11× resistencias de 10 kΩ (divisores de voltaje)
- Protoboard, cables dupont, cinta adhesiva, grafito en polvo

---

## Ejecución

Abrir 5 terminales simultáneas:

```bash
# Terminal 1 — Lanzar robots (reales)
ros2 launch xarm_moveit_config dual_lite6_moveit_realmove.launch.py \
    robot_ip_1:=192.168.1.179 robot_ip_2:=192.168.1.154

# Terminal 1 — Alternativa: simulación
ros2 launch xarm_moveit_config dual_lite6_moveit_fake.launch.py

# Terminal 2 — Espejo maestro→esclavo + detección de colisión
python3 code/dual_mirror.py --force-stop-threshold 3.0 --force-resume-threshold 2.0

# Terminal 3 — Control del brazo maestro
python3 code/master_input.py --port /dev/ttyUSB1 --speed 1.0

# Terminal 4 — Lector de fuerza del esclavo
python3 code/force_reader5.py --port /dev/ttyUSB0

# Terminal 5 — Visualización (opcional)
python3 code/force_display5.py
```

---

## Calibración de sensores ESP32

1. Abrir `firmware/slave_force_sensor.ino` en Arduino IDE
2. Cambiar `CALIBRATION_MODE = true`
3. Flashear y abrir el Monitor Serial (115200 baud)
4. Anotar los valores ADC en reposo → `BASELINE[]`
5. Presionar cada sensor firmemente → anotar el mínimo → `PRESSED[]`
6. Cambiar `CALIBRATION_MODE = false` y reflashear

---

## Formato de datos

Cada archivo `.xlsx` contiene 13 columnas sin encabezado:

| Col | Descripción | Unidad |
|-----|-------------|--------|
| 0 | Timestamp Unix | s |
| 1–6 | Posiciones articulares del **maestro** (J1–J6) | rad |
| 7–12 | Posiciones articulares del **esclavo** (J1–J6) | rad |

Frecuencia de muestreo: **10 Hz**

---

## Análisis de datos

Los tres archivos de simulación son **acumulativos** (cada uno es un superconjunto del anterior). Los tres experimentos reales son sesiones independientes. Para extraer los segmentos correctos de la simulación:

```python
import numpy as np, openpyxl

d1 = np.array(list(openpyxl.load_workbook('data/sim/log_experimento1.xlsx')
                   .active.iter_rows(values_only=True)), dtype=float)
d2 = np.array(list(openpyxl.load_workbook('data/sim/log_experimento2.xlsx')
                   .active.iter_rows(values_only=True)), dtype=float)
d3 = np.array(list(openpyxl.load_workbook('data/sim/log_experimento3.xlsx')
                   .active.iter_rows(values_only=True)), dtype=float)

sim1 = d1[3172:3912]        # Exp 1: 2026-03-14 09:03–09:09 (342 s)
sim2 = d3[len(d1):len(d2)]  # Exp 2: 2026-03-14 09:13–09:15  (92 s)
sim3 = d3[len(d2):]         # Exp 3: 2026-03-14 09:17–09:18  (40 s)
```

---

## Resultados

| Par | Condición | RMS global [°] | Retardo |
|-----|-----------|---------------|---------|
| 1 | Simulación (342 s) | 1.03° | ~0 ms |
| 1 | Robot real (287 s) | 0.33° | ~150 ms |
| 2 | Simulación (92 s) | 0.73° | ~0 ms |
| 2 | Robot real (118 s) | 0.42° | ~150 ms |
| 3 | Simulación (40 s) | 0.93° | ~0 ms |
| 3 | Robot real (62 s) | 1.10° | ~150 ms |

---

## Autores

- Nombre Alumno 1 — `matricula1@tec.mx`
- Nombre Alumno 2 — `matricula2@tec.mx`
- Nombre Alumno 3 — `matricula3@tec.mx`

---

## Referencias

- [xArm Python SDK](https://github.com/xArm-developer/xArm-Python-SDK)
- [xArm ROS2](https://github.com/xArm-developer/xarm_ros2)
- B. Siciliano et al., *Robotics: Modelling, Planning and Control*, Springer, 2009
- R. Murray, Z. Li, S. Sastry, *A Mathematical Introduction to Robotic Manipulation*, CRC Press, 1994
