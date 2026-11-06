# Size-aware obstacle avoidance

See [README_BUDDY5.md](README_BUDDY5.md) for the complete scanning, size-aware
planning, B2 movement and B3 line recovery implementation, configuration,
calibration requirements and tests.

Size limits, chassis geometry and clearance margins are configured in
`app_program/robot_config.h`. The default firmware now follows the line, avoids a valid profiled obstacle,
reacquires the original line and resumes following. The executable B2/B3
navigation bridge is installed at startup. Physical calibration remains
unverified. Set `ROBOT_ENABLE_BYPASS=0` for the legacy forward/STOP profile.
