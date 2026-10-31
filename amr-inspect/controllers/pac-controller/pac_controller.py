"""
Controlador de Evasión de Obstáculos y Percepción para Pioneer 3-DX en Webots.
Implementa el algoritmo de Braitenberg calibrado con lookup-table inversa de Webots
para los transductores ultrasónicos so0-so15, complementado con visión HD (640x480)
y visualización en tiempo real con HUD interactivo mediante OpenCV.
"""

from dataclasses import dataclass
from typing import Tuple, List, Optional, Any
from enum import Enum
import math
import sys

# Wrappers de seguridad para dependencias externas
try:
    from controller import Robot, DistanceSensor, Camera, Motor
except ImportError:
    Robot = DistanceSensor = Camera = Motor = object

try:
    import cv2
    import numpy as np
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


# Constantes oficiales del Pioneer 3-DX en Webots
MAX_SPEED: float = 4.2           # rad/s nominal seguro (límite motor 5.24)
MAX_SENSOR_VALUE: float = 1024.0 # Rango máximo de Pioneer3DistanceSensor
MIN_DISTANCE: float = 1.25       # Distancia en metros para activar evasión anticipada
WHEEL_WEIGHT_THRESHOLD: float = 80.0

# Pesos cinemáticos Braitenberg oficiales para Pioneer 3-DX
# sensors[i] = (peso_rueda_izquierda, peso_rueda_derecha)
# so0 a so3 (lado izquierdo) -> aceleran rueda izquierda / frenan derecha para girar a la derecha
# so4 a so7 (lado derecho) -> aceleran rueda derecha / frenan izquierda para girar a la izquierda
BRAITENBERG_WEIGHTS: List[Tuple[float, float]] = [
    (180.0,   0.0),  # so0: 90° izq
    (240.0,   0.0),  # so1: 50° izq
    (380.0,   0.0),  # so2: 30° izq
    (750.0,   0.0),  # so3: 10° frontal-izq
    (  0.0, 750.0),  # so4: 10° frontal-der
    (  0.0, 380.0),  # so5: 30° der
    (  0.0, 240.0),  # so6: 50° der
    (  0.0, 180.0),  # so7: 90° der
    (  0.0,   0.0),  # so8 - so15: sensores traseros
    (  0.0,   0.0),
    (  0.0,   0.0),
    (  0.0,   0.0),
    (  0.0,   0.0),
    (  0.0,   0.0),
    (  0.0,   0.0),
    (  0.0,   0.0),
]


class State(Enum):
    FORWARD = "FORWARD"
    LEFT = "LEFT"
    RIGHT = "RIGHT"
    REVERSE = "REVERSE"
    INSPECTION = "INSPECTION"


@dataclass(frozen=True)
class VisionInspectionResult:
    """Resultado inmutable de la inspección visual por frame."""
    hazard_detected: bool
    hazard_ratio: float
    hazard_box: Optional[Tuple[int, int, int, int]] = None


class VisionSystem:
    """Segmentación visual de anomalías de riesgo con OpenCV en resolución 640x480 con telemetría HUD en tiempo real."""

    def __init__(self, camera: Optional[Any], width: int = 640, height: int = 480, enable_gui: bool = True):
        self.camera = camera
        self.width = width
        self.height = height
        self.enable_gui = enable_gui

    def process_frame(
        self,
        current_state: str = "FORWARD",
        min_sonar_dist: float = 5.0
    ) -> VisionInspectionResult:
        """Captura el frame actual de la cámara Webots, ejecuta segmentación HSV y proyecta la interfaz HUD."""
        if not self.camera or not CV_AVAILABLE:
            return VisionInspectionResult(hazard_detected=False, hazard_ratio=0.0)

        raw = self.camera.getImage()
        if raw is None:
            return VisionInspectionResult(hazard_detected=False, hazard_ratio=0.0)

        try:
            # Reconstrucción de la matriz BGRA de Webots a espacio BGR
            img = np.frombuffer(raw, np.uint8).reshape((self.height, self.width, 4))
            bgr = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

            # Rango espectral para cilindros de riesgo químico (rojo / bermellón industrial)
            mask_lower = cv2.inRange(hsv, np.array([0, 90, 60]), np.array([10, 255, 255]))
            mask_upper = cv2.inRange(hsv, np.array([170, 90, 60]), np.array([180, 255, 255]))
            raw_hazard = cv2.bitwise_or(mask_lower, mask_upper)

            # Filtrado morfológico para eliminar ruido especular
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            filtered_mask = cv2.morphologyEx(raw_hazard, cv2.MORPH_OPEN, kernel)
            filtered_mask = cv2.morphologyEx(filtered_mask, cv2.MORPH_DILATE, kernel)

            # Ratio de píxeles activados respecto al encuadre total
            ratio = float(cv2.countNonZero(filtered_mask)) / float(self.width * self.height)
            is_hazard = ratio > 0.030

            # Bounding box del objetivo de mayor área
            hazard_box: Optional[Tuple[int, int, int, int]] = None
            contours, _ = cv2.findContours(filtered_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest) > 200:
                    hazard_box = cv2.boundingRect(largest)

            # Renderizado de ventana interactiva HUD
            if self.enable_gui:
                self._draw_hud(bgr, is_hazard, ratio, hazard_box, current_state, min_sonar_dist)

            return VisionInspectionResult(
                hazard_detected=is_hazard,
                hazard_ratio=ratio,
                hazard_box=hazard_box
            )
        except Exception:
            return VisionInspectionResult(hazard_detected=False, hazard_ratio=0.0)

    def _draw_hud(
        self,
        frame: np.ndarray,
        is_hazard: bool,
        ratio: float,
        box: Optional[Tuple[int, int, int, int]],
        current_state: str,
        min_sonar_dist: float
    ) -> None:
        """Dibuja telemetría y marcadores de detección en la ventana de OpenCV."""
        display = frame.copy()

        # Delinear el objetivo de riesgo si fue identificado
        if box is not None:
            bx, by, bw, bh = box
            box_color = (0, 0, 255) if is_hazard else (0, 165, 255)
            cv2.rectangle(display, (bx, by), (bx + bw, by + bh), box_color, 2)
            cv2.putText(
                display,
                f"OBJETIVO QUIMICO: {ratio * 100:.1f}%",
                (bx, max(24, by - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                box_color,
                2
            )

        # Barra superior HUD con fondo semitransparente
        cv2.rectangle(display, (0, 0), (self.width, 40), (20, 20, 20), -1)
        cv2.line(display, (0, 40), (self.width, 40), (80, 80, 80), 1)

        # Colores dinámicos por estado de navegación
        state_colors = {
            "FORWARD": (0, 255, 0),
            "LEFT": (0, 220, 255),
            "RIGHT": (0, 220, 255),
            "INSPECTION": (0, 0, 255),
            "REVERSE": (255, 0, 255)
        }
        active_color = state_colors.get(current_state, (200, 200, 200))

        # 1. Estado de la máquina finita
        cv2.putText(
            display,
            f"ESTADO: {current_state}",
            (15, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            active_color,
            2
        )

        # 2. Telemetría ultrasónica
        sonar_label = f"Sonar: {min_sonar_dist:.2f}m" if min_sonar_dist < 5.0 else "Sonar: Despejado"
        cv2.putText(
            display,
            sonar_label,
            (250, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (220, 220, 220),
            1
        )

        # 3. Protocolo de seguridad
        sec_text = "ALERTA: RIESGO" if is_hazard else "ZONA SEGURA"
        sec_color = (0, 0, 255) if is_hazard else (0, 255, 0)
        cv2.putText(
            display,
            sec_text,
            (self.width - 230, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            sec_color,
            2
        )

        # Mira central de inspección
        cx, cy = self.width // 2, self.height // 2
        cross_color = (0, 0, 255) if is_hazard else (0, 255, 0)
        cv2.drawMarker(display, (cx, cy), cross_color, markerType=cv2.MARKER_CROSS, markerSize=18, thickness=1)

        try:
            cv2.imshow("AMR-Inspect | Camara HD Pioneer 3-DX (640x480)", display)
            cv2.waitKey(1)
        except Exception:
            self.enable_gui = False


class PioneerObstacleAvoidance:
    """Controlador principal de navegación robusta anti-colisión."""

    def __init__(self):
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())

        # Actuadores de ruedas
        self.left_wheel = self.robot.getDevice("left wheel")
        self.right_wheel = self.robot.getDevice("right wheel")
        self.left_wheel.setPosition(float("inf"))
        self.right_wheel.setPosition(float("inf"))
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)

        # Transductores ultrasónicos so0-so15
        self.sensors: List[Any] = []
        for i in range(16):
            s = self.robot.getDevice(f"so{i}")
            if s:
                s.enable(self.time_step)
                self.sensors.append(s)

        # Cámara HD
        self.camera = self.robot.getDevice("camera")
        if self.camera:
            self.camera.enable(self.time_step)

        w = self.camera.getWidth() if self.camera else 640
        h = self.camera.getHeight() if self.camera else 480
        self.vision = VisionSystem(self.camera, w, h, enable_gui=True)

        self.state = State.FORWARD
        self.reverse_timer = 0
        self.inspection_timer = 0

    def step(self) -> bool:
        if self.robot.step(self.time_step) == -1:
            return False

        # Conversión de distancias ultrasónicas con lookup-table inversa de Webots
        wheel_weight_total = [0.0, 0.0]
        critical_proximity = False
        min_frontal_distance = 5.0

        for i in range(min(16, len(self.sensors))):
            val = self.sensors[i].getValue()
            if val <= 0.0:
                continue

            # Inversión oficial de la lookup-table de Webots para Pioneer3DistanceSensor:
            # Distancia en metros = 5.0 * (1.0 - val / 1024.0)
            distance = 5.0 * (1.0 - (val / MAX_SENSOR_VALUE))

            if i in [2, 3, 4, 5]:
                if distance < min_frontal_distance:
                    min_frontal_distance = distance
                # Detección de contacto inminente (a menos de 0.25 metros)
                if distance < 0.25:
                    critical_proximity = True

            if distance < MIN_DISTANCE:
                speed_modifier = 1.0 - (distance / MIN_DISTANCE)
                wheel_weight_total[0] += BRAITENBERG_WEIGHTS[i][0] * speed_modifier
                wheel_weight_total[1] += BRAITENBERG_WEIGHTS[i][1] * speed_modifier

        # Procesamiento visual en cada iteración del lazo de control
        vision_result = self.vision.process_frame(
            current_state=self.state.value,
            min_sonar_dist=min_frontal_distance
        )

        # 1. Inspección de riesgo por cámara
        if vision_result.hazard_detected and self.state != State.INSPECTION:
            self.state = State.INSPECTION
            self.inspection_timer = 40

        if self.state == State.INSPECTION:
            self.left_wheel.setVelocity(0.0)
            self.right_wheel.setVelocity(0.0)
            self.inspection_timer -= 1
            if self.inspection_timer <= 0:
                self.state = State.REVERSE
                self.reverse_timer = 30
            return True

        # 2. Rutina de retroceso si quedó acorralado
        if self.state == State.REVERSE:
            self.left_wheel.setVelocity(-0.6 * MAX_SPEED)
            self.right_wheel.setVelocity(-0.3 * MAX_SPEED)
            self.reverse_timer -= 1
            if self.reverse_timer <= 0:
                self.state = State.FORWARD
            return True

        # 3. Retroceso inmediato ante contacto inminente en esquinas o paredes
        if critical_proximity:
            self.state = State.REVERSE
            self.reverse_timer = 35
            return True

        # 4. Máquina de estados con histéresis anti-choque
        speed = [0.0, 0.0]

        if self.state == State.FORWARD:
            if wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD:
                # Obstáculo a la izquierda -> girar a la derecha
                speed[0] = 0.75 * MAX_SPEED
                speed[1] = -0.75 * MAX_SPEED
                self.state = State.RIGHT
            elif wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD:
                # Obstáculo a la derecha -> girar a la izquierda
                speed[0] = -0.75 * MAX_SPEED
                speed[1] = 0.75 * MAX_SPEED
                self.state = State.LEFT
            else:
                # Camino despejado: avance continuo
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED

        elif self.state == State.RIGHT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or
                    wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                speed[0] = 0.8 * MAX_SPEED
                speed[1] = -0.8 * MAX_SPEED
            else:
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                self.state = State.FORWARD

        elif self.state == State.LEFT:
            if (wheel_weight_total[0] > WHEEL_WEIGHT_THRESHOLD or
                    wheel_weight_total[1] > WHEEL_WEIGHT_THRESHOLD):
                speed[0] = -0.8 * MAX_SPEED
                speed[1] = 0.8 * MAX_SPEED
            else:
                speed[0] = MAX_SPEED
                speed[1] = MAX_SPEED
                self.state = State.FORWARD

        # Aplicación directa de velocidades calibradas a los motores
        self.left_wheel.setVelocity(speed[0])
        self.right_wheel.setVelocity(speed[1])
        return True

    def run(self) -> None:
        try:
            while self.step():
                pass
        finally:
            if CV_AVAILABLE:
                try:
                    cv2.destroyAllWindows()
                except Exception:
                    pass


if __name__ == "__main__":
    controller = PioneerObstacleAvoidance()
    controller.run()
