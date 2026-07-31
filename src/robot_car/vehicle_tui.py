"""Minimal curses frontend for the vehicle action dispatcher."""

from __future__ import annotations

import curses
import time
from typing import Any, Dict


ACTION_NAMES = {
    0: "STOP",
    1: "ROLLER_HOME",
    2: "LINE_LAP_TO_A",
    3: "ROLLER_SWEEP",
    4: "LINE_TO_B_BALANCE_CENTER",
    5: "LINE_LAP_BALANCE_CENTER",
    6: "LINE_LAP_BALANCE_TARGET",
}

ACTION_TASKS = {
    0: "停止",
    1: "摆杆电机回零",
    2: "题2 巡线一圈回A",
    3: "题3 滚珠+50后-50",
    4: "题4 巡线到B并保持中心",
    5: "题5 巡线一圈保持中心",
    6: "题6 巡线一圈保持指定位置",
}


def run_vehicle_tui(daemon: Any, roller_config: Dict[str, Any] | None = None) -> None:
    """Run the interactive selector while the daemon control loop runs in a thread."""
    curses.wrapper(_main, daemon, roller_config or {})


def _main(screen: Any, daemon: Any, roller_config: Dict[str, Any]) -> None:
    screen.nodelay(True)
    screen.keypad(True)
    curses.curs_set(0)
    input_buffer = ""
    while not daemon.stop_event.is_set():
        key = screen.getch()
        if key in (ord("q"), ord("Q")):
            daemon.request_action(0, source="tui")
            daemon.stop_event.set()
            break
        if key in (ord("s"), ord("S"), 27):
            daemon.request_action(0, source="tui")
            input_buffer = ""
        elif key in (10, 13):
            if input_buffer:
                try:
                    daemon.request_action(int(input_buffer), source="tui")
                except (TypeError, ValueError) as error:
                    daemon.set_ui_error(str(error))
                input_buffer = ""
        elif key == curses.KEY_BACKSPACE or key in (8, 127):
            input_buffer = input_buffer[:-1]
        elif 48 <= key <= 57 and len(input_buffer) < 3:
            input_buffer += chr(key)
        _draw(screen, daemon, roller_config, input_buffer)
        time.sleep(0.05)


def _draw(screen: Any, daemon: Any, roller_config: Dict[str, Any], input_buffer: str) -> None:
    screen.erase()
    snapshot = daemon.ui_snapshot()
    action = snapshot["action"]
    control = roller_config.get("control", roller_config)
    pid = control.get("position_pid", {})
    vel = control.get("velocity_pid", {})
    angle = control.get("angle_pid", {})
    screen.addnstr(0, 0, "RDK vehicle control TUI", max(1, curses.COLS - 1))
    status = (f"PID p={_pid(pid)} v={_pid(vel)} a={_pid(angle)} | "
              f"当前动作={action.get('action_id') or '-'}({ACTION_NAMES.get(action.get('action_id'), '-')}) "
              f"状态={action.get('status')} 阶段={action.get('phase') or '-'} | "
              f"下位机动作={action.get('last_remote_action_id') or '-'} | "
              f"球误差={_number(action.get('ball_error_mm'))}mm | "
              f"UART RX={snapshot['gateway']['received']} ACK={snapshot['gateway']['acks']}")
    screen.addnstr(1, 0, status, max(1, curses.COLS - 1))
    screen.addnstr(3, 0, f"动作映射: 0{ACTION_TASKS[0]} | 1{ACTION_TASKS[1]} | {ACTION_TASKS[2]}",
                   max(1, curses.COLS - 1))
    screen.addnstr(4, 0, f"动作映射: {ACTION_TASKS[3]} | {ACTION_TASKS[4]} | {ACTION_TASKS[5]} | {ACTION_TASKS[6]}",
                   max(1, curses.COLS - 1))
    screen.addnstr(6, 0, "输入动作编号并回车，S=停止，ESC=停止，Q=退出",
                   max(1, curses.COLS - 1))
    screen.addnstr(7, 0, f"> {input_buffer}", max(1, curses.COLS - 1))
    if snapshot.get("ui_error"):
        screen.addnstr(8, 0, f"错误: {snapshot['ui_error']}", max(1, curses.COLS - 1))
    screen.refresh()


def _pid(value: Dict[str, Any]) -> str:
    return f"{float(value.get('kp', 0)):.2f},{float(value.get('ki', 0)):.2f},{float(value.get('kd', 0)):.2f}"


def _number(value: Any) -> str:
    return "-" if value is None else f"{float(value):+.1f}"
