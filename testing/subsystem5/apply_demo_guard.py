#!/usr/bin/env python3
"""Safely patch only B5's latest task_obstacle.c without overwriting team edits.

Run from any working directory. Idempotent: an already patched file is left
unchanged. Refuses to modify unknown versions of the two guarded snippets.
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TASK = ROOT / 'app_program/subsystem5/task_obstacle.c'
text = TASK.read_text(encoding='utf-8')
if '#if S5_DEMO_SENSOR_ONLY' in text:
    print('B5 demo guard already installed; no changes made')
    raise SystemExit(0)
old_mode = '''    tm_printf((UB*)"[S5] SENSOR=REAL SERVO=REAL MOTORS=%s LINE=UNAVAILABLE\\n",
              ROBOT_ENABLE_DRIVE ? "GUARDED FORWARD/STOP" : "DISABLED");'''
new_mode = '''    tm_printf((UB*)"[S5] SENSOR=REAL SERVO=REAL MOTORS=%s LINE=UNAVAILABLE\\n",
              S5_DEMO_SENSOR_ONLY ? "DEMO STOP-ONLY" :
              (ROBOT_ENABLE_DRIVE ? "GUARDED FORWARD/STOP" : "DISABLED"));'''
old_release = '''                    b2_request_forward(); /* Heartbeat every valid clear sample. */
                    if (!driving) {
                        tm_printf((UB*)"[S5-DRIVE] valid clear=%d cm; FORWARD requested (verify B2 FORWARD and wheel movement)\\n",
                                  (int)cm);
                        driving = true;
                    }'''
new_release = '''#if S5_DEMO_SENSOR_ONLY
                    /* Demo mode: keep the B2 STOP latch closed even on
                     * confirmed clear readings. Do not move the robot. */
                    b2_request_stop();
                    driving = false;
#else
                    b2_request_forward(); /* Heartbeat every valid clear sample. */
                    if (!driving) {
                        tm_printf((UB*)"[S5-DRIVE] valid clear=%d cm; FORWARD requested (verify B2 FORWARD and wheel movement)\\n",
                                  (int)cm);
                        driving = true;
                    }
#endif'''
if text.count(old_mode) != 1 or text.count(old_release) != 1:
    raise SystemExit('ERROR: task_obstacle.c does not match expected B5 snippets. '
                     'No changes made; review your latest main version.')
patched = text.replace(old_mode, new_mode).replace(old_release, new_release)
TASK.write_text(patched, encoding='utf-8')
print('B5 sensor-only demo guard installed in:', TASK)
