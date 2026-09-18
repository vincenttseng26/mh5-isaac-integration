# First-Connection Runbook (FS100 / MH5)

This runbook outlines the steps for the first physical connection to the FS100 controller.
**WARNING: Strict read-only operations only. No motion commands, parameter writes, or servo-on operations are permitted.**

## Prerequisites
- **FS100 Controller Discovery & Setup Checklist** is 100% completed.
- The area inside the robot's physical reach is completely clear of personnel.
- You have physical access to the Teach Pendant E-Stop.
- Ethernet cable is connected between the PC and the Controller's LAN port directly (isolated network).

## Phase 1: Read-Only Probe
0. **Configuration Verification**: Ensure `fs100_config.json` has `enabled: true`, `read_only: true`, and the correct IP/port are explicitly filled by the user.
1. **Dry-Run Test**: Execute the pure mock probe to verify the local software stack and JSON schema before making any network changes:
   ```bash
   python3 -m robot_integration.probe_cli --dry-run
   ```
2. **Network Initialization**: Configure the PC network interface to the same subnet as the controller.
3. **Launch Logging**: Start the local logging service to record all incoming packets (wireshark/tcpdump on isolated interface).
4. **Execute Probe Script**: Run the read-only script that opens the connection and queries the following using `FS100Transport`:
   - Current Controller State (Play/Teach/Remote, E-Stop, Alarm).
   - Current Joint Angles (in degrees/radians).
   - Current Cartesian TCP Position (in mm).
5. **Validation**: 
   - Verify the read joint angles exactly match the values displayed on the Teach Pendant.
   - Verify the read TCP position exactly matches the Teach Pendant.
6. **Disconnect**: Gracefully close the TCP/UDP connection.

**Note:** There is no Phase 2. First connection is strictly limited to read-only state verification. Any APIs that write to the controller, change states, or turn on the servo must NOT be implemented or used.
