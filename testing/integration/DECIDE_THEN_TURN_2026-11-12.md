# Reduce detection range and avoid unnecessary rescans

Changed stop detection from 25 to 20 cm and restart from 30 to 25 cm.
Turn clearance changes from 20 to 15 cm so the reduced detection threshold
does not eliminate the interval in which an outward turn is permitted.
Chassis geometry and turn clearance still require physical validation.

Turn candidates now use a distinct 25 cm outer-ray opening threshold,
rather than the fixed 40 cm silhouette classification. A side between
25 and 40 cm is therefore eligible for a turn and live forward monitoring.
The 40 cm classification remains in measured obstacle profiling; these
are different decisions. The course-bound fallback keeps the obstacle
latched when its coarse front sample is slightly above STOP but still
below RELEASE, avoiding rejection from ordinary measurement variation
during a stationary scan.

Regression tests verify a coarse front sample at STOP+1 cm with a 28 cm
left opening produces GO_LEFT rather than WAIT_ROUTE, and its mirrored
case selects RIGHT. Invalid samples, insufficient turn clearance, and no
eligible opening still prevent movement. Existing tests retain the
one-second forward leg, opposite turn, subsequent forward pass, and
bounded line reacquisition. The straight trim remains 80 permille.

All host suites passed; the B5 suite was rerun with the added regression.
The network firmware generated successfully. No physical floor-test pass
is claimed. The servo range remains 70–110 degrees.

Startup marker: `B5-DECIDE-THEN-TURN`.
`S5-DECISION` logs the front/side ranges and turn thresholds for a scan
that still cannot produce a route. A genuinely blocked route still waits
and rescans instead of driving blindly.

UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `466B43972DD03773E70CE3892C848CF00915A34834700BF88AB3C36FFAAEDC20`.
