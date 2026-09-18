# Isaac Sim practice scene

This folder provides a safe, simulation-only entry point for practicing the MH5 workcell.
It does not enable ROS, TCP, MoveIt, or physical robot commands.

## Launch

With Isaac Sim installed and its Python environment activated:

```bash
./simulation/launch_sim.sh
```

The scene contains the training workcell and target cube. The original physical
integration scripts remain in `robot_integration/`; use them only after reviewing
the safety runbook and configuring your own machine.

The checked-in USD is intentionally small and may reference the original MH5
asset layout. For a fully self-contained robot asset, place the licensed MH5 and
RG2-FT USD/mesh files under `simulation/assets/` and update the references in the
scene. Do not commit credentials, calibration captures, or private IP addresses.
