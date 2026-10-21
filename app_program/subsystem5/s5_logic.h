#ifndef S5_LOGIC_H
#define S5_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include "obstacle_scan.h"

/* Software-demo thresholds. NOT certified physical collision clearances. */
#define S5_STOP_CM 25.0f
#define S5_CLEAR_CM 40.0f
#define S5_MAX_RANGE_CM 450.0f
#define S5_FRONT_INVALID_LIMIT 3u
#define S5_WAIT_LIMIT 30u /* stop ACK / scan ACK: 3 seconds at 100ms/tick */
#define S5_MOTION_LIMIT 200u /* bypass/reverse: 20 seconds */
#define S5_RECOVERY_LIMIT 150u /* line search: 15 seconds */
/* Demonstration assumptions: adjust from measured chassis geometry later. */
#define S5_CHASSIS_HALF_WIDTH_CM 8.0f
#define S5_EXTRA_MARGIN_CM 8.0f

/* Five angles centered on nearest coarse detection, at 10-degree steps. */
void s5_make_fine_angles(uint16_t center_deg, uint16_t out[S5_SCAN_COUNT]);

typedef enum {
    S5_FOLLOW, S5_WAIT_STOP, S5_COARSE, S5_FINE, S5_REVERSE, S5_AVOID,
    S5_RECOVER, S5_DONE, S5_FAULT
} S5State;
typedef enum {
    S5_CMD_NONE, S5_CMD_STOP, S5_CMD_SCAN_COARSE,
    S5_CMD_SCAN_FINE, S5_CMD_REVERSE, S5_CMD_GO_LEFT, S5_CMD_GO_RIGHT,
    S5_CMD_FIND_LINE, S5_CMD_RESUME, S5_CMD_HALT
} S5Command;
typedef enum { S5_SIDE_NONE, S5_SIDE_LEFT, S5_SIDE_RIGHT } S5Side;
typedef enum {
    S5_ERR_NONE, S5_ERR_FRONT_SENSOR, S5_ERR_STOP_TIMEOUT,
    S5_ERR_SCAN_TIMEOUT, S5_ERR_NO_CLEAR_ROUTE,
    S5_ERR_FINE_SCAN, S5_ERR_MOTION_TIMEOUT, S5_ERR_LINE_TIMEOUT,
    S5_ERR_INSUFFICIENT_CLEARANCE, S5_ERR_REVERSE_TIMEOUT
} S5Error;

typedef struct {
    bool front_valid;
    float front_cm;
    bool stopped;
    bool coarse_ready;
    S5Scan coarse;
    bool fine_ready;
    S5Scan fine;
    bool motion_complete;
    bool line_found;
} S5Input;

typedef struct {
    bool valid;
    float nearest_cm;
    float nearest_lateral_cm; /* x: negative left, positive right */
    float nearest_forward_cm; /* y: forward distance */
    uint16_t nearest_angle_deg;
    uint16_t left_boundary_deg;
    uint16_t right_boundary_deg;
    float estimated_width_cm;
    float center_x_cm; /* left/right offset from forward axis; right positive */
    float center_y_cm; /* distance forward from sensor */
    float required_lateral_clearance_cm;
    float left_clearance_cm; /* minimum measured coarse left rays */
    float right_clearance_cm; /* minimum measured coarse right rays */
    /* Distance readings do not prove a driveable route: simulation only. */
    S5Side bypass_side;
} S5Profile;

typedef struct {
    S5State state;
    S5Error error;
    S5Side side;
    S5Profile profile;
    uint16_t ticks_in_state;
    uint16_t fine_center_deg;
    uint8_t reverse_attempts;
    S5Scan last_coarse;
    uint8_t invalid_front_count;
    uint16_t events;
} S5Controller;

void s5_controller_init(S5Controller *c);
S5Command s5_controller_step(S5Controller *c, const S5Input *in);
S5Side s5_choose_side(const S5Scan *scan);
S5Profile s5_estimate_profile(const S5Scan *fine, S5Side side);
const char *s5_state_name(S5State s);
const char *s5_command_name(S5Command a);
const char *s5_error_name(S5Error e);
#endif
