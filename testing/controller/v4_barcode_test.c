/* Exercise the real v4 state machine with the shared controller mocks. */
#define main legacy_controller_test_main
#include "controller_test.c"
#undef main

static VEHICLE_CONTROLLER reading(void)
{
    VEHICLE_CONTROLLER c = fresh();
    c.running = true;
    c.mode = MODE_AUTO;
    c.bar_phase = BAR_READ;
    c.bar_ms = input.now_ms;
    c.bar_mark_um = c.travel_um;
    c.drive_sent = true;
    c.last_drive_ms = 0;
    c.last_speed = vc4_mm_s_to_cps(60);
    return c;
}

int main(void)
{
    VEHICLE_CONTROLLER c = reading();
    /* Exact raw readings from the user's floor/junction screenshots.
     * Exercise channel scaling before testing the actual control path. */
    uint16_t floor_r = b3_line_cal_scale(195, B3_LINE_GP26_WHITE, B3_LINE_GP26_BLACK);
    uint16_t floor_l = b3_line_cal_scale(142, B3_LINE_GP27_WHITE, B3_LINE_GP27_BLACK);
    uint16_t tape_r = b3_line_cal_scale(355, B3_LINE_GP26_WHITE, B3_LINE_GP26_BLACK);
    uint16_t tape_l = b3_line_cal_scale(842, B3_LINE_GP27_WHITE, B3_LINE_GP27_BLACK);
    assert(floor_r == 0 && floor_l == 0 && tape_r == 1000 && tape_l == 1000);
    assert(B3_LINE_IS_JUNCTION(tape_r, tape_l));
    c.bar_phase = BAR_IDLE;
    input.centre = tape_l;
    input.left = floor_r;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(sent.steering_permille < 0); /* physical GP27 left */
    c = reading();
    c.bar_phase = BAR_IDLE;
    input.centre = floor_l;
    input.left = tape_r;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(sent.steering_permille > 0); /* physical GP26 right */
    c = reading();
    (void)v4_accept_barcode(&c, B3_NAV_LEFT);
    for (int i = 0; i < 10; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 2000;
        input.centre = tape_l;
        input.left = tape_r;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    }
    for (int i = 0; i < 20 && c.v4_phase == V4_FOLLOW; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 2000;
        input.centre = floor_l;
        input.left = floor_r;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    }
    assert(c.v4_phase == V4_CROSS && c.navigation == B3_NAV_LEFT);
    c = reading();
    /* Decode delivery must survive a tick spent waiting for clearance. */
    input.clearance = false;
    vehicle_controller_step(&c, &input, B3_NAV_LEFT);
    assert(c.running && c.bar_phase == BAR_IDLE && c.bar_wait_clear);
    assert(c.pending_navigation == B3_NAV_LEFT);
    input.clearance = true;
    input.centre = 950;
    input.left = 50;
    input.now_ms += VC_PERIOD_MS;
    vehicle_controller_step(&c, &input, B3_NAV_NONE);
    assert(c.pending_navigation == B3_NAV_LEFT && sent.steering_permille < 0);
    c = reading();
    input.clearance = false;
    vehicle_controller_step(&c, &input, B3_NAV_SLOW);
    assert(c.slow_mode && c.bar_phase == BAR_IDLE && c.pending_navigation == B3_NAV_NONE);
    /* A second decode from the consumed strip must not repeatedly wipe
     * curve memory or reset the steering filter. */
    c = reading();
    assert(v4_accept_barcode(&c, B3_NAV_LEFT));
    c.trk.filt = 600;
    c.trk.bias = 400;
    assert(v4_accept_barcode(&c, B3_NAV_LEFT));
    assert(c.trk.filt == 600 && c.trk.bias == 400);
    assert(v4_accept_barcode(&c, B3_NAV_SLOW));
    assert(c.trk.filt == 600 && c.trk.bias == 400);
    /* Repeated stop/start cycles must not treat the previous run's bars
     * or ignored sensor as a fresh barcode on the first following tick. */
    c = reading();
    for (int run = 0; run < 5; ++run) {
        c.bar_phase = BAR_READ;
        c.strip_known = c.strip_active = c.bars_active = true;
        c.strip_edges = VC4_BAR_STRIP_EDGES + 4;
        c.bar_ignore = -1;
        c.v4_time_valid = c.travel_time_valid = true;
        command(&c, RCMD_STOP, 0);
        command(&c, RCMD_START, 0);
        assert(!c.strip_active && !c.strip_known && !c.bars_active);
        assert(c.bar_ignore == 0 && !c.v4_time_valid && !c.travel_time_valid);
        input.centre = 950;
        input.left = 50;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.bar_phase == BAR_IDLE && c.v4_phase == V4_FOLLOW);
        assert(sent.steering_permille < 0);
    }
    /* High raw spikes cannot instantly satisfy the lift debounce on even
     * timestamps. Sustained readings still stop after the real interval. */
    c = reading();
    input.raw26 = input.raw27 = VC4_LIFT_IR_RAW;
    input.now_ms = 100;
    assert(!v4_lifted(&c, &input));
    input.now_ms += 20;
    input.raw26 = input.raw27 = 0;
    assert(!v4_lifted(&c, &input));
    input.raw26 = input.raw27 = VC4_LIFT_IR_RAW;
    assert(!v4_lifted(&c, &input));
    input.now_ms += VC4_LIFT_IR_MS + 1;
    assert(v4_lifted(&c, &input));
    c = reading();
    /* A stale right recovery and alternating bars must never start a
     * sweep, even during a quiet gap in GP28 edge activity. */
    c.trk.filt = -900;
    c.trk.exit_side = -1;
    c.trk.lost = true;
    for (int i = 0; i < 80; ++i) {
        input.now_ms += VC_PERIOD_MS;
        input.centre = i % 2 ? 950 : 50;
        input.left = i % 2 ? 50 : 950;
        input.barcode_active = i % 3 != 0;
        c.travel_um += 1000;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.running && c.bar_phase == BAR_READ);
        assert(c.v4_phase == V4_FOLLOW && !c.trk.lost);
        assert(sent.type == MOTION_CMD_DRIVE && sent.steering_permille == 0);
        assert(sent.target_speed_cps == vc4_mm_s_to_cps(60));
        assert(c.trk.jct_state == 0);
    }
    /* Initial detection must also override a stale lost-line flag. */
    c = reading();
    c.bar_phase = BAR_IDLE;
    c.strip_known = c.strip_active = true;
    c.strip_level = input.right;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    c.strip_last_um = c.travel_um;
    c.trk.lost = true;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(c.bar_phase == BAR_READ && c.v4_phase == V4_FOLLOW);
    assert(sent.steering_permille == 0);

    /* Reject -> bounded reverse, never a recovery TURN. */
    c = reading();
    c.travel_um = 100000;
    input.barcode_rejects = 1;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(c.bar_phase == BAR_BRAKE && c.bar_tries == 1);
    assert(c.v4_phase == V4_FOLLOW);
    input.now_ms += VC_PERIOD_MS;
    assert(v4_barcode(&c, &input));
    assert(c.bar_phase == BAR_REVERSE && reversed_cm > 0);
    result_ready = true;
    result.command_id = c.v4_motion_id;
    result.status = MOTION_STATUS_COMPLETED;
    assert(v4_barcode(&c, &input));
    assert(c.bar_phase == BAR_SETTLE);
    input.now_ms += VC4_BAR_WAIT_MS;
    assert(!v4_barcode(&c, &input));
    assert(c.bar_phase == BAR_READ && !c.trk.lost);
    /* The lead-in accelerates to the retry target instead of freezing at
     * the first startup increment while there are no barcode edges. */
    input.barcode_active = false;
    input.centre = input.left = 100;
    for (int i = 0; i < 30; ++i) {
        input.now_ms += VC_PERIOD_MS;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(sent.steering_permille == 0);
    }
    assert(sent.target_speed_cps == vc4_mm_s_to_cps(VC4_BAR_READ_MM_S));

    /* Exhaustion skips this strip and keeps driving, without re-arming
     * retries on its remaining bars. A later strip can still be read. */
    c = reading();
    c.bar_tries = VC4_BAR_RETRIES;
    c.strip_known = c.strip_active = true;
    c.strip_level = input.right;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    c.strip_start_um = c.travel_um;
    c.strip_last_um = c.travel_um;
    input.barcode_rejects = 1;
    UINT previous_drives = drives;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(c.running && c.bar_phase == BAR_IDLE && drives > previous_drives);
    assert(sent.type == MOTION_CMD_DRIVE && sent.steering_permille == 0);
    assert(c.bar_clear_known && c.v4_phase == V4_FOLLOW);
    for (int i = 0; i < 10; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 1000;
        c.strip_last_um = c.travel_um;
        input.right ^= 1;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.running && c.bar_phase == BAR_IDLE);
    }
    c.travel_um += VC4_BAR_REARM_UM + 1;
    /* A real clear gap rearms barcode reading. */
    c.strip_active = false;
    input.barcode_active = false;
    input.now_ms += VC_PERIOD_MS;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(!c.bar_wait_clear);
    c.strip_active = true;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    c.strip_start_um = c.strip_last_um = c.travel_um;
    input.now_ms += VC_PERIOD_MS;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(c.running && c.bar_phase == BAR_READ && c.bar_tries == 0);

    /* Decode B (LEFT) while the old barcode episode and masking are still
     * active. Steering must resume now; turning waits for a real junction. */
    c = reading();
    c.strip_known = c.strip_active = true;
    c.strip_level = input.right;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    c.strip_last_um = c.travel_um;
    c.bars_active = true;
    c.bar_ignore = -1;
    input.barcode_active = true;
    input.centre = 900;
    input.left = 100;
    v4_drive(&c, &input, B3_NAV_LEFT, vc4_mm_s_to_cps(120));
    assert(c.bar_phase == BAR_IDLE && c.pending_navigation == B3_NAV_LEFT);
    assert(c.v4_phase == V4_FOLLOW && sent.type == MOTION_CMD_DRIVE);
    assert(sent.steering_permille < 0 && !c.bars_active);
    /* Old GP28 episode remains active; it must not block the crossbar. */
    for (int i = 0; i < 10; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 2000;
        c.strip_last_um = c.travel_um;
        input.centre = 750;
        input.left = 700; /* realistic non-saturated crossbar */
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.v4_phase == V4_FOLLOW && c.pending_navigation == B3_NAV_LEFT);
    }
    input.centre = input.left = 100;
    for (int i = 0; i < 20 && c.v4_phase == V4_FOLLOW; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 2000;
        c.strip_last_um = c.travel_um;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    }
    assert(c.v4_phase == V4_CROSS && c.navigation == B3_NAV_LEFT);
    assert(c.v4_jct_advance_cm > 0 && c.v4_jct_advance_cm < VC4_JCT_ADVANCE_CM);
    assert(c.pending_navigation == B3_NAV_NONE);
    v4_cross(&c, &input); /* advance from sensors to axle */
    result_ready = true;
    result.command_id = c.v4_motion_id;
    result.status = MOTION_STATUS_COMPLETED;
    v4_cross(&c, &input);
    assert(sent.type == MOTION_CMD_TURN && sent.target_angle_deg == VC4_JCT_LEFT_TURN_DEG);
    assert(sent.turn_direction == MOTION_TURN_LEFT);
    assert(sent.target_speed_cps == vc4_mm_s_to_cps(VC4_JCT_LEFT_TURN_MM_S));
    v4_cross(&c, &input);
    assert(c.v4_phase == V4_FOLLOW && c.navigation == B3_NAV_NONE);
    c = reading();
    c.bar_phase = BAR_IDLE;
    c.v4_phase = V4_CROSS;
    c.v4_sub = 1;
    c.navigation = B3_NAV_RIGHT;
    v4_cross(&c, &input);
    assert(sent.type == MOTION_CMD_TURN && sent.target_angle_deg == 90);
    assert(sent.turn_direction == MOTION_TURN_RIGHT);
    assert(sent.target_speed_cps == vc4_mm_s_to_cps(VC4_TURN_MM_S));

    /* A decode can arrive before the controller enters its read phase. */
    c = reading();
    c.bar_phase = BAR_IDLE;
    c.strip_known = c.strip_active = true;
    c.strip_level = input.right;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    input.barcode_active = true;
    input.centre = 900;
    input.left = 100;
    v4_drive(&c, &input, B3_NAV_LEFT, vc4_mm_s_to_cps(120));
    assert(c.bar_clear_known && c.bar_phase == BAR_IDLE);
    assert(sent.steering_permille < 0);
    /* GP28 starts another episode inside rearm distance. It must neither
     * mask steering nor freeze the old speed while following the curve. */
    c.strip_start_um = c.travel_um + 1;
    c.last_speed = vc4_mm_s_to_cps(120);
    for (int i = 0; i < 5; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 1000;
        c.strip_last_um = c.travel_um;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.bar_phase == BAR_IDLE && sent.steering_permille < 0);
    }
    assert(sent.target_speed_cps < vc4_mm_s_to_cps(120));
    /* Even beyond the old distance cooldown, continuing GP28 chatter
     * cannot reacquire steering ownership after a successful decode. */
    for (int i = 0; i < 90; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 2000;
        input.right ^= 1;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.bar_wait_clear && c.bar_phase == BAR_IDLE);
        assert(sent.steering_permille < 0 && c.pending_navigation == B3_NAV_LEFT);
    }

    /* Reacquisition during a sweep needs continuous tape, and seeds the
     * next following tick toward that line rather than starting straight. */
    c = reading();
    c.bar_phase = BAR_IDLE;
    c.v4_phase = V4_SEARCH;
    c.v4_sub = 2;
    c.v4_motion_id = 42;
    c.recovery_ms = input.now_ms;
    input.centre = 950;
    input.left = 50;
    result_ready = false;
    v4_search(&c, &input);
    assert(c.v4_phase == V4_SEARCH);
    input.now_ms += VC_PERIOD_MS;
    input.centre = 100;
    v4_search(&c, &input);
    assert(c.v4_line_seen_ms == 0);
    input.centre = 950;
    for (int i = 0; i < 6 && c.v4_phase == V4_SEARCH; ++i) {
        input.now_ms += VC_PERIOD_MS;
        v4_search(&c, &input);
    }
    assert(c.v4_phase == V4_FOLLOW && c.trk.filt > 0);

    /* Losing a curve first requests a forward arc, in either direction,
     * without reverse or discrete turns. Tape found there resumes following. */
    for (int side = -1; side <= 1; side += 2) {
        c = reading();
        c.bar_phase = BAR_IDLE;
        c.trk.filt = side * 800;
        input.centre = input.left = 100;
        v4_start_search(&c, &input);
        assert(c.v4_sub == 5);
        input.now_ms += VC_PERIOD_MS;
        v4_search(&c, &input);
        assert(sent.type == MOTION_CMD_DRIVE && sent.direction == MOTION_DIR_FORWARD);
        assert(sent.steering_permille * side < 0);
        input.centre = side > 0 ? 950 : 50;
        input.left = side > 0 ? 50 : 950;
        for (int i = 0; i < 6 && c.v4_phase == V4_SEARCH; ++i) {
            input.now_ms += VC_PERIOD_MS;
            v4_search(&c, &input);
        }
        assert(c.v4_phase == V4_FOLLOW && c.trk.filt * side > 0);
    }
    c = reading();
    c.bar_phase = BAR_IDLE;
    input.centre = input.left = 100;
    v4_start_search(&c, &input);
    input.now_ms += VC4_SEARCH_ARC_MS;
    v4_search(&c, &input);
    assert(c.v4_sub == 0 && c.v4_phase == V4_SEARCH);
    assert(c.running); /* arc timeout hands over to search, not end-run */

    c = reading();
    c.bar_phase = BAR_REVERSE;
    c.v4_motion_id = 42;
    result_ready = true;
    result.command_id = 42;
    result.status = MOTION_STATUS_CANCELLED;
    assert(v4_barcode(&c, &input));
    assert(!c.running && c.bar_phase == BAR_IDLE);

    c = reading();
    c.bar_phase = BAR_BRAKE;
    input.now_ms += VC_RESCAN_TIMEOUT_MS + 1;
    assert(v4_barcode(&c, &input));
    assert(!c.running && c.bar_phase == BAR_IDLE);
    puts("PASS v4 barcode straight hold, bounded retry, skip unread strip and motion failure stops");
    return 0;
}
