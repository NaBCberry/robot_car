"""Vision-result overlay kept independent from control logic."""

from typing import Any, Iterable

from robot_car.perception.events import VisionEvent


def draw_overlay(image: Any, events: Iterable[VisionEvent]) -> Any:
    """Draw detected boxes and state values without affecting control decisions."""
    import cv2

    canvas = image.copy()
    for event in events:
        box = event.payload.get("bbox_xyxy")
        if not isinstance(box, (list, tuple)) or len(box) != 4:
            continue
        x1, y1, x2, y2 = (int(round(float(value))) for value in box)
        is_balance = event.event_type == "BALL_BALANCE_STATE"
        color = (48, 196, 104) if event.event_type in {"BALL_TARGET", "BALL_BALANCE_STATE"} else (0, 183, 255)
        if is_balance:
            roi = event.payload.get("roller_roi_xyxy")
            if isinstance(roi, (list, tuple)) and len(roi) == 4:
                rx1, ry1, rx2, ry2 = (int(round(float(value))) for value in roi)
                cv2.rectangle(canvas, (rx1, ry1), (rx2, ry2), (255, 190, 0), 1)
                center_x = int(round(float(event.payload.get("center_x_px", (rx1 + rx2) / 2))))
                cv2.line(canvas, (center_x, ry1), (center_x, ry2), (255, 190, 0), 1)
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
        center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(canvas, (center_x, center_y if is_balance else y2), 4, color, -1)
        label = f"{event.event_type} {event.confidence:.2f}"
        if event.event_type == "BALL_TARGET":
            label += (f" {event.payload.get('bearing_mdeg', 0) / 1000.0:+.1f}deg"
                      f" {event.payload.get('range_mm', 0)}mm")
        elif is_balance:
            label += (f" x={event.payload.get('position_mm', 0):+d}mm"
                      f" v={event.payload.get('velocity_mm_s', 0):+d}mm/s"
                      f" a={event.payload.get('acceleration_mm_s2', 0):+d}mm/s2")
        cv2.putText(canvas, label, (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, color, 2, cv2.LINE_AA)
    return canvas
