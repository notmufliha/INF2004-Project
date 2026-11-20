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

/* B2 steering for the barcode steering limit at the read speed. */
static INT bar_steer_limit(void)
{
    int32_t cps, permille;
    vc4_wheels(VC4_BAR_STEER_MAX, 60, VC4_SLOW_MM_S, &cps, &permille);
    return (INT)(permille < 0 ? -permille : permille);
}

static INT iabs_i(INT v) { return v < 0 ? -v : v; }

int main(void)
{
    VEHICLE_CONTROLLER c = reading();
    /* A stale right recovery and alternating bars must never start a
     * sweep, even during a quiet gap in B3 edge activity (GP28 keeps
     * changing, so the strip stays active), and steering stays limited. */
    c.trk.filt = -900;
    c.trk.exit_side = -1;
    c.trk.lost = true;
    c.strip_known = c.strip_active = true;   /* a read starts on bars */
    c.strip_level = input.right;
    c.strip_edges = VC4_BAR_STRIP_EDGES;
    c.strip_last_um = c.travel_um;
    for (int i = 0; i < 80; ++i) {
        input.now_ms += VC_PERIOD_MS;
        input.centre = i % 2 ? 950 : 50;
        input.left = i % 2 ? 50 : 950;
        input.barcode_active = i % 3 != 0;
        input.right = i % 2 ? 900 : 100;
        c.travel_um += 1000;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
        assert(c.running && c.bar_phase == BAR_READ);
        assert(c.v4_phase == V4_FOLLOW && !c.trk.lost);
        assert(sent.type == MOTION_CMD_DRIVE);
        assert(iabs_i(sent.steering_permille) <= bar_steer_limit());
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
    assert(iabs_i(sent.steering_permille) <= bar_steer_limit());

    /* After the bars, still in the read (no decode yet): a curve that
     * starts right after the barcode must be followed, not driven
     * straight off (the line under the left sensor = turn left). */
    c = reading();
    c.strip_known = true;
    c.strip_level = input.right;
    input.barcode_active = false;
    input.centre = 950;
    input.left = 50;
    for (int i = 0; i < 20; ++i) {
        input.now_ms += VC_PERIOD_MS;
        c.travel_um += 1000;
        v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    }
    assert(c.bar_phase == BAR_READ && c.v4_phase == V4_FOLLOW);
    assert(sent.steering_permille < -bar_steer_limit());
    input.centre = input.left = 100;

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

    /* Exhaustion stops the run; it cannot re-arm retries or issue DRIVE. */
    c = reading();
    c.bar_tries = VC4_BAR_RETRIES;
    input.barcode_rejects = 1;
    UINT previous_drives = drives;
    v4_drive(&c, &input, B3_NAV_NONE, vc4_mm_s_to_cps(120));
    assert(!c.running && c.bar_phase == BAR_IDLE && drives == previous_drives);
    assert(published == ROBOT_IDLE);

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
    puts("PASS v4 barcode straight hold, quiet gaps, bounded retry and failure stops");
    return 0;
}
