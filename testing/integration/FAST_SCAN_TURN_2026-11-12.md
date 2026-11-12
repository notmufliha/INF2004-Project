# Faster obstacle decision and straight-trim refinement

Set straight forward DRIVE trim to 100 permille (+10% left, -10% right),
the midpoint between the reported 80-permille left drift and 120-permille
slight right drift. This remains a floor-tuning trial with encoder feedback.

A usable coarse scan now selects and commits to GO_LEFT/GO_RIGHT directly.
It first tries a measured profile, then the explicit course-size bounds.
The course-bound fallback also accepts a single contiguous centre hit when
the obstacle was independently detected and the scan has a valid side
opening. It does not fabricate a measured width. Fine scanning is retained
only when no coarse route can be produced. No opening still means a stopped
route wait; the change does not force a turn into blocked space.

Scan timing changes:

- Three agreeing pings per angle instead of five candidates; all three
  must agree. One bounded retry remains, and two valid echoes cannot pass.
- Settling after servo travel is 120 ms instead of 220 ms.
- Servo steps remain 3 degrees but occur every 20 ms instead of 35 ms.
- Removed the extra 65 ms delay after each measured ray. Servo travel and
  settling already exceed it, and the driver independently enforces a
  65 ms minimum between physical triggers.

A consistent five-ray coarse scan therefore uses 15 pings rather than 25
and can avoid the entire second sweep. No exact hardware scan duration is
claimed; retries, echo range and physical settling affect actual duration.
Servo travel remains 70–110 degrees. Stop/restart remain 20/25 cm, and the
one-second forward leg and opposite turn remain in the maneuver sequence.

Validation: all host suites passed; the B5 suite passed again after adding
left/right direct-coarse decisions, the reduced delay budget, three-ping
consensus, and rejection of a two-valid-echo batch before retry. The network
firmware built successfully. Faster servo timing and the new trim require
physical verification; host tests do not certify those measurements.

Startup marker: `B5-FAST-SCAN-TURN`, with `straight_trim=100`.
UF2: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt.uf2`.
SHA256: `F586756A7FB67A147D5088CAC71903AB17201BCB9FB10CAE606529DDABBFFEB2`.
