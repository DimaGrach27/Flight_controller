for start server simulation
gz sim -v 4 -r /Users/dhrachov/Projects/Embedded/Flight_controller_v1/Simulator/GazeboHil/worlds/one_axis_hil.sdf -s

for start gui simulation
./scripts/run_gui.sh

The GUI config opens the FPV camera topic:

```text
/X3/fpv_camera/image
```

and overlays a goggles-style OSD subscribed to:

```text
/fc/telemetry/osd
```

The HIL plugins publish a compact goggles-style telemetry string there with arm state,
mode, battery, attitude, gyro rates, RC input, motor output, torque, and ground-truth
position when available.
