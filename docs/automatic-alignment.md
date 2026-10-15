# Automatic display alignment

Enable **Follow display movement** in the web UI, then save the configuration.
Existing installations keep fixed-position recognition until this is enabled.
The reference ROIs and segment profiles are retained. Automatic alignment is
intended for bright seven-segment digits on one panel; all numeric ROIs must
move together. Dark-on-light displays and oversized geometry fail explicitly
as `unsupported_geometry`.

The default search covers ±24 horizontal and ±12 vertical pixels. The maximum
is ±32/±20. Movement outside that window requires recentering the reference
ROIs. Rotation, scale changes, and perspective changes are not corrected.

## Recognition

`src/registration_core.h` runs in both firmware and the native replay tests.
Each frame searches the entire bounded window, avoiding a tracking error that
could accumulate over time or lock onto the previous number.

For each translation the recognizer estimates foreground brightness across
the digit group, and background brightness independently in each digit. It
uses a relative threshold at 60% of the background-to-foreground interval.
It requires exact seven-segment patterns, with the existing, narrow italic
seven correction retained. A weak ON sample must have extra brightness
headroom. Blank positions are allowed only where ROI configuration permits.

The score combines segment separation and a penalty for bright pixels away
from the proposed illuminated strokes. Blank hour positions have less weight
than lit digits, so their darkness cannot overwhelm a poorly aligned minute.
The best competing *different reading* must be at least five score points
behind the winner. These are heuristic scores, not probability percentages.
Thresholds and score constants were checked against the local corpus; they
are not a guarantee for unseen lighting or layouts.

The winning translation applies to numeric and symbol ROIs. It does not
rewrite the reference coordinates. The web Test Detection result reports
offset, score, competing-reading margin, per-digit thresholds, and elapsed
time. Cyan dashed outlines show the translated ROIs and segment samples.
The preview and detection frame are separate captures; recapture and retest
if the scene moves between them.

## State semantics

The existing three-frame numeric majority remains in use. Registration mode
publishes null numeric values when alignment is ambiguous or invalid, rather
than replaying an indefinitely stale duration. Display activity is determined
separately from numeric validity. Strong multi-digit evidence can establish
activity even without an accepted reading. Partial evidence with uncertain
activity publishes null activity; it must not become a fabricated OFF state.
Home Assistant's MQTT binary-sensor template maps null to `None` (unknown),
as documented by [Home Assistant](https://www.home-assistant.io/integrations/binary_sensor.mqtt/).
`blank_as_zero` remains numeric interpretation only.

The wire result includes `alignment_state`: `aligned`, `unreadable`,
`inactive`, or (in single-frame diagnostics) `unsupported_geometry`.
Blank displays have null numeric values and inactive activity. An active
unreadable frame has unknown numbers, with active or unknown activity.

## Verification

The native replay uses local, Git-ignored washer evidence in the adjacent
`new_home` checkout. It reads only segment geometry from the historical config;
credentials never enter the replay input or test output.

```bash
g++ -std=c++11 -O2 -Wall -Wextra -Werror -fsanitize=address,undefined \
  tests/registration_core_test.cpp -o /tmp/washer-registration-unit
ASAN_OPTIONS=detect_leaks=0 /tmp/washer-registration-unit
/usr/bin/python3 tests/check_registration.py \
  --current ../new_home/data/washer-recognition/captures/20261015-native-registration-control-141.jpg
/usr/bin/python3 tests/check_registration.py --augment --allow-clipped-abstention \
  --current ../new_home/data/washer-recognition/captures/20261015-native-registration-control-141.jpg
```

Leak detection is disabled because the execution sandbox uses ptrace; address
and undefined-behavior sanitizers remain enabled. All ten synthetic digits
are tested at three horizontal positions, along with blank and bounds cases.

The real corpus has 20 historical cases, including both lighting conditions
and three blank-display controls, plus the 2026-10-15 `1:41` control. Historical
reference placement comes from the existing labelled corpus; the search itself
receives no expected reading or answer offset. Augmentations translate frames
by up to ±15/±10 pixels and scale brightness by 0.65, 1.0, or 1.25. Augmentations
of the already displaced new frame that lie outside the configured search
window are excluded explicitly.

The extended replay includes gamma 0.8 and 1.2. With the new control there
are 432 cases: 426 correct readings/inactive controls and six explicit
clipped-frame abstentions. The safety gate passes 432/432. Strict numeric
readability still fails those six abstentions; they are not counted as reads.

The strict readability test omits `--allow-clipped-abstention`. Its known limit
is six over-brightened copies of the same lights-on `0:29` frame: clipping
narrows the difference between `29` and `28`, so the recognizer rejects the
reading while keeping activity true. The safety gate allows these explicit
abstentions, but never permits a wrong numeric value or a false active blank.
This distinction must remain visible in verification reports.

Before claiming hardware acceptance, verify the new firmware version,
registration config readback, recognition latency, repeated active reads,
MQTT/Home Assistant updates, and persistence after reboot. Use camera evidence
to verify physical blankness; failed old fixed-position recognition cannot
prove the display is off.

## Recovery

Disable `registration.enabled` and save to restore the original recognition
path without losing any ROI geometry. For a firmware rollback, use the
application-only `bin's/firmware-v1.0.024.bin` through the OTA endpoint. Its
SHA-256 before this work was
`8bb88f86090dc039d9d3c2984800fd74e77f3c497356aa18868188ba7d770d0f`.
Never upload the merged factory image through application OTA.

## Hardware validation, 2026-10-15

Firmware `1.0.026` was deployed to the washer reader. Application image SHA-256:
`7dc67d938a6216f1c8c05440a034d067f292cfb2263e35ecaecb68acd556ad55`.
Two HTTP multipart uploads aborted without changing the running firmware;
the existing Arduino OTA service then transferred the same application image
successfully. The running version was checked after reboot.

Automatic alignment is enabled with the documented defaults. Comparing full
configuration exports confirmed that only `registration` changed. A separate
reader reboot then preserved the entire saved configuration exactly.

The visible display read `1:41`. The initial successful read and all three
post-reboot checks found `dx=-12, dy=-5` without changing the reference ROIs.
Post-reboot recognition took 932–944 ms per frame. Camera health and MQTT
connectivity were verified. Fresh Home Assistant updates showed hours `1`,
minutes `41`, combined duration `101 min`, and display activity `on`.

In the three checks immediately before the persistence reboot, two frames
decoded correctly and one was rejected with a competing-reading margin of
4.71 (below the required 5). Activity remained true; no wrong value was
published. This is a deliberate abstention, not a successful numeric read.
The subsequent three-frame MQTT pipeline delivered the correct duration.

When the machine timed out during the earlier deployment attempt, three
evaluations returned inactive/null, and a camera frame independently
confirmed the display was physically blank. Work stopped until the user
turned it back on. Room lights were not controlled during this deployment;
the two lighting conditions remain covered by the offline corpus, rather
than a controlled live lights-on/off acceptance run.

Local configuration backups and diagnostic JSON remain Git-ignored under
`../new_home/data/washer-recognition/device/`, including
`config-20261015-registration-deploy-before.json`,
`config-20261015-registration-enabled.json`,
`config-20261015-registration-after-reboot.json`, and
`evaluate-20261015-registration-post-reboot-{1,2,3}.json`.
