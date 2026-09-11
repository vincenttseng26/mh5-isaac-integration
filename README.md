# MH5 + Isaac Sim 5.0 + ZED integration

This workspace contains the reproducible integration layer for the Motoman MH5,
RG2-FT gripper, ZED point cloud/RGB stream, Isaac Sim and MoveIt.

## Architecture

- **Isaac PC**: Isaac Sim scene, calibrated ZED overlay, staged grasp client.
- **CYC PC** (`192.168.50.10`): arm state gateway `8769`, MoveIt planner `8770`,
  gripper gateway `8768`, and ZED sender tunneled to local `127.0.0.1:8765`.
- **Physical arm**: controlled only through the guarded staged gateway. Commands
  are one-shot and return to `MIRROR / DISARMED`.

## Verified workflow

```bash
cd /home/vincent/Desktop/mh5_isaaclab_reach_lift_v0/integration_workspace
python3 robot_integration/run_staged_grasp.py --stage pre_grasp --confirm
python3 robot_integration/run_staged_grasp.py --stage grasp --confirm
python3 robot_integration/run_staged_gripper.py --stage close --confirm
python3 robot_integration/run_staged_grasp.py --stage lift --confirm
python3 robot_integration/run_staged_grasp.py --stage home --confirm
```

The downward-opening gripper uses an 18 mm grasp-height compensation. Always
check the live scene and keep the emergency stop accessible.

## Start services

Use the scripts in `robot_integration/` to start the CYC gateways and ZED SSH
tunnel. Do not commit private keys, device credentials or local calibration
captures.

See `isaac-integration-checklist.md` and `robot_integration/first_connection_runbook.md`.

## Tests

```bash
PYTHONPATH=robot_integration:. pytest -q robot_integration/test_*.py camera_calibration/tests
```
