# AMR-Inspect

**Sistema Robótico Autónomo de Inspección y Monitoreo de Riesgos en Almacenes Industriales**

Producto Académico Colaborativo (PAC) — Maestría en Inteligencia Artificial, Universidad Continental.

AMR-Inspect es un robot móvil autónomo (Pioneer 3-DX) que patrulla un almacén, evita obstáculos con 16 sonares y se detiene a inspeccionar cuando su cámara detecta contenedores o marcadores de riesgo de color rojo. Está implementado en el simulador gratuito y de código abierto **Webots R2025a**, con un controlador en **Python 3 + OpenCV**.

---

## Integrantes

| Integrante | Rol |
|---|---|
| Cáceres Montejo, Yherald César | Integrante |
| Mancilla Felices, Carlos Josué | Integrante |
| Vargas Rodríguez, Javier Simón | Líder del equipo |

**Docente:** Miguel Andrés Solís Cid

---

## Arquitectura

```
PERCEPCIÓN                 PROCESAMIENTO                         DECISIÓN                ACTUACIÓN
16 sonares so0–so15  ──►  Inversión lookup-table (0–5 m)  ──►  Ponderación Braitenberg ─┐
                                                          └──►  Proximidad crítica      ─┼──►  FSM  ──►  Motores
Cámara RGB 640×480   ──►  Segmentación HSV (rojo > 3.5 %) ─────────────────────────────┘          (ω_L, ω_R)
```

| Componente | Especificación |
|---|---|
| Robot | Pioneer 3-DX, tracción diferencial (r = 0.0975 m, L = 0.33 m) |
| Sensores de distancia | 16 sonares (alcance 0–5 m); los 8 frontales intervienen en la evasión |
| Visión | Cámara RGB frontal de 640×480, montada en el `extensionSlot` |
| Actuadores | Motores `left wheel` y `right wheel` en modo velocidad (máx. por software: 4.2 rad/s) |
| Control | `pac_controller.py`: evasión de Braitenberg + máquina de estados finitos |

### Máquina de estados

| Estado | Comportamiento | Transición |
|---|---|---|
| `FORWARD` | Avance recto a 0.41 m/s | Obstáculo → `LEFT`/`RIGHT`; < 0.25 m → `REVERSE`; riesgo visual → `INSPECTION` |
| `RIGHT` / `LEFT` | Giro en el sitio hasta despejar ambos lados | Despejado → `FORWARD` |
| `REVERSE` | Retroceso en arco (35 ciclos, o 30 tras una inspección) | Temporizador agotado → `FORWARD` |
| `INSPECTION` | Detención total durante 40 ciclos | → `REVERSE` |

---

## Estructura del repositorio

```
amr-inspect/
├── controllers/
│   └── pac_controller/
│       └── pac_controller.py     # Controlador del robot
├── worlds/
│   └── [nombre_del_mundo].wbt    # Almacén simulado de 7 × 7 m
├── docs/
│   └── INFORME_PAC_AMR_INSPECT.docx
├── requirements.txt
└── README.md
```

---

## Requisitos

- [Webots R2025a](https://cyberbotics.com) (gratuito, Windows / Linux / macOS)
- Python 3.10 o superior
- Dependencias de Python:

```bash
pip install -r requirements.txt
```

> Si OpenCV no está instalado, el controlador desactiva la visión y el robot solo realiza la evasión de obstáculos.

---

## Ejecución

1. Clonar el repositorio:
   ```bash
   git clone https://github.com/[USUARIO]/amr-inspect.git
   ```
2. Instalar las dependencias en el intérprete de Python que usa Webots (**Tools → Preferences → Python command**).
3. Abrir `worlds/[nombre_del_mundo].wbt` en Webots.
4. Verificar que el campo `controller` del nodo `Pioneer3dx` sea `pac_controller`.
5. Presionar **Play** (▶). Los cambios de estado se muestran en la consola de Webots.

---

## Parámetros principales

Definidos al inicio de `pac_controller.py`:

| Parámetro | Valor | Descripción |
|---|---|---|
| `MAX_SPEED` | 4.2 rad/s | Velocidad angular base de las ruedas |
| `MIN_DISTANCE` | 1.25 m | Distancia desde la que los sonares influyen en el giro |
| `WHEEL_WEIGHT_THRESHOLD` | 80 | Peso acumulado que dispara un giro |
| Proximidad crítica | 0.25 m | Distancia en `so2`–`so5` que fuerza el retroceso |
| Umbral de visión | 3.5 % | Fracción de píxeles rojos que activa `INSPECTION` |

---

## Limitaciones conocidas

- Solo detecta rojo (tono HSV 0–10 y 170–180); el naranja no se detecta.
- Cualquier objeto rojo grande (por ejemplo, un extintor) activa la inspección.
- No guarda memoria de los riesgos ya inspeccionados.
- Los sonares traseros no se usan durante el retroceso.

---

## Créditos

La lógica de evasión parte del controlador de ejemplo `pioneer3dx_collision_avoidance` de Cyberbotics (Webots) y fue adaptada y ampliada por el equipo con retroceso ante proximidad crítica, detección visual de riesgos y los estados `REVERSE` e `INSPECTION`.

Webots es software de código abierto bajo licencia Apache 2.0 — © Cyberbotics Ltd.
