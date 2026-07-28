"""Vision-result overlay kept independent from control logic."""

from typing import Any, Iterable

from robot_car.perception.events import VisionEvent


def draw_overlay(image: Any, events: Iterable[VisionEvent]) -> Any:
    """Draw detected boxes and polar target values without affecting control decisions."""
    import cv2

    canvas = image.copy()
    for event in events:
        box = event.payload.get("bbox_xyxy")
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            continue
        x1, y1, x2, y2 = (int(round(float(value))) for value in box)
        color = (48, 196, 104) if event.event_type == "BALL_TARGET" else (0, 183, 255)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        center_x = (x1 + x2) // 2
        cv2.circle(canvas, (center_x, y2), 4, color, -1)
        label = f"{event.event_type} {event.confidence:.2f}"
        if event.event_type == "BALL_TARGET":
            label += (f" {event.payload.get('bearing_mdeg', 0) / 1000.0:+.1f}deg"
                      f" {event.payload.get('range_mm', 0)}mm")
        cv2.putText(canvas, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2, cv2.LINE_AA)
    return canvas
