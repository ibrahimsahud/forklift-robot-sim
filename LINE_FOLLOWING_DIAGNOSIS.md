# Line Following Diagnosis

This note captures why the previous `LINE_PRIMARY_ROUTE` attempts failed and
what the line controller is expected to do.

## Why The Previous Controller Failed

The line detector was working. The failures were control failures:

- The robot could see the line but remained far off-center.
- The first controller behaved like weak steering correction instead of active
  line capture.
- In some runs, the angle term partially fought the centering term during
  acquisition.
- The robot rotated somewhat, but it did not consistently re-position itself
  onto the line before trying to continue.
- The line then drifted out of the camera ROI and the controller stopped.

So the problem was not map data, route YAML, or HSV thresholds. It was the
difference between "seeing a line" and "recovering onto a line."

## Definitions

### Line Detection

The camera can segment a visible line and estimate:

- line center offset in the image
- line angle / heading inside the ROI

Detection alone is not enough.

### Line Following

Once the robot is already near-centered and heading-aligned with the line, it
can drive forward while making continuous small corrections.

### Line Capture / Acquisition

When the robot starts off the line or significantly misaligned, it must first
pull the line toward the image center and keep it in view. This may require:

- turning in place
- creeping slowly
- using a short corrective arc

This is a different problem from ordinary following.

### Pose Alignment To The Line

Even if the line is visible and roughly centered, the robot may still not be
parallel to it. Alignment uses both lateral error and line-angle error so the
robot becomes properly oriented along the line before forward progress.

## Target FSM

The intended controller FSM is:

1. `SEARCH_LINE`
2. `CAPTURE_LINE`
3. `ALIGN_TO_LINE`
4. `FOLLOW_LINE`
5. `LINE_LOST_STOP`
6. `FAIL_SAFE`

### Success Signals

The following indicate correct behavior:

- line error decreases over time
- centered state becomes true
- aligned state becomes true
- forward travel occurs mainly during aligned/follow states
- the segment reaches its target waypoint

### Failure Signals

The following indicate failure:

- line remains visible but error does not decrease
- centered ratio stays low
- the line exits the image during capture
- the robot rotates without converging onto the line
- the controller reaches `LINE_LOST_STOP` or `FAIL_SAFE`
