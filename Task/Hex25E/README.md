# Hex25E route task

The Hex25E task uses only the wheel encoders and JY61P IMU for navigation.
The removed eight-channel line sensor is not initialized, sampled, used for
control, or included in telemetry.

## Start and route

USERKEY is the local start/stop control. Pressing it while stopped immediately
treats the current chassis position and yaw as node O; there is no all-black
start-marker wait. The configured route is:

```text
O -> A -> G -> F -> H -> M -> H -> C -> B -> A -> O
```

O-to-A is 370 mm. Every regular map edge is 400 mm. Encoder average distance
determines the end of each edge, while the JY61P yaw loop keeps every straight
segment on its captured heading. In-place turn wheel travel is excluded by
braking once and taking a new encoder snapshot before the next edge.

The first visit to A, F, H, M and C stops for 500 ms. After those stops, the
normal +/-60 degree route changes use the opposite 300 degree direction under
a continuous yaw-rate loop. M rotates 360 degrees, brakes for 500 ms, and then
rotates the remaining 180 degrees.
The final angle loop removes the small stop error before the next edge.

## Heading-letter record

During each long inspection turn, the task combines the current Y node with
the accumulated gyro turn angle and records a letter whenever the nose is
within 5 degrees of that labelled direction. O is deliberately skipped. The
deduplicated route record is:

```text
N, M, J, K, L, E, A, I, G, F, H, C, D, B
```

The current ASCII letter, 14-bit A-to-N mask, count, and 14-byte ordered log
are available in `Task_Hex25E_Status` and the `g_TASK_HeadingLetter*` debug
variables. Zero means the nose is not currently aligned with a labelled
direction. These fields are not added to the compact VOFA stream.

## Indicators

- Normal: RGB green, LED1 on, LED2 off.
- Fault: RGB red, LED1 off, LED2 on.

## Bluetooth and VOFA

Bluetooth uses logical serial transport 0 at 115200 baud. VOFA FireWater sends
one six-column numeric row every 50 ms:

```text
IMU.yaw_deg,ODO.route_mm,ROUTE.next_node,WHEEL.actual_l_mm_s,WHEEL.actual_r_mm_s,FAULT.code
```

The route odometer contains straight travel only; dwell and turn travel are
excluded. A yaw value of -9999 means the IMU value is unavailable. The serial
backend is recovered if TX makes no progress for 300 ms, and the VOFA header is
resent after recovery.
