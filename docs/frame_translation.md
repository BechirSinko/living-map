# Frame Translation

Robots work in a **private frame**; humans and the command post need **real GPS**. The gateway
converts between the two, so the robots never need to know GPS.

## Local frame
- Origin: the tunnel entrance, the one point with a known GPS fix.
- +x: the direction the Writer faced at start.
- +y: 90 degrees to the left of +x (counter-clockwise).
- Units: meters.

## Anchor
Measured once at the entrance and stored in the gateway: `(lat0, lon0, heading_deg)`, where
`heading_deg` is the compass bearing of +x (degrees clockwise from true north). The heading can be read from a
magnetometer (corrected for magnetic declination) or taken from the entrance geometry on a map.

## Math
Step 1, rotate local meters into East/North meters:

```
East  =  x*sin(psi) - y*cos(psi)
North =  x*cos(psi) + y*sin(psi)
```

Step 2, convert meters to degrees with WGS-84 meters-per-degree at `lat0`:

```
m_lat = 111132.92 - 559.82*cos(2*lat0) + 1.175*cos(4*lat0)
m_lon = 111412.84*cos(lat0) - 93.5*cos(3*lat0)
lat = lat0 + North / m_lat
lon = lon0 + East  / m_lon
```

The inverse (`gps_to_local`) is used to send the Executor mission targets back in its own frame.

Sanity checks: with heading 0, +x points north and +y points west. With heading 90, +x points east and
+y points north. Both are covered in `tests/test_frames.py`.

## Accuracy and error sources
| Source | Effect | Mitigation |
|---|---|---|
| Dead-reckoning drift (encoders, IMU) | Position error grows with distance | Beacons act as landmarks; the Executor re-anchors at each beacon |
| Wrong entrance heading | Error grows linearly with distance (1 degree = 1.7 m per 100 m) | Measure heading carefully, cross-check with a map |
| Entrance GPS error (a few m) | Constant offset on the whole map | Use a good fix, average multiple readings |
| Flat-earth approximation | Under a centimeter over 300 m | None needed at tunnel scale |

## Reference code
`livingmap/frames.py`: `local_to_gps`, `gps_to_local`, `local_to_enu`, `enu_to_local`.
