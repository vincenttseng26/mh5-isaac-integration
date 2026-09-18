# FS100 Controller Discovery & Setup Checklist

Before attempting any connection to the physical MH5 / FS100 controller, the following information must be gathered and configured on-site:

## 1. Option & License Verification
- [ ] **Ethernet Server / High-Speed Ethernet**: Is the option enabled in the controller's Maintenance Mode?
- [ ] **MotoPlus**: If using MotoPlus, is the MotoPlus function enabled?
- [ ] **Required Hardware**: Are there any additional network boards required, or are we using the standard LAN1 port?

## 2. Mode Settings (Remote / Play / Teach)
- [ ] **Key Switch**: Is the key switch on the Teach Pendant set to **Remote** mode (required for PC control)?
- [ ] **Permissions**: Are Management Mode or appropriate security levels active to allow parameter changes if needed?

## 3. Ethernet & Network Settings
- [ ] **Controller IP Address**: ___________
- [ ] **Subnet Mask**: ___________
- [ ] **Gateway**: ___________
- [ ] **Target Port**: ___________ (Often 11000 for standard Ethernet Server, but must be verified).
- [ ] **Ping Check**: From a completely isolated, disconnected subnet, can the PC ping the controller IP? (No port scanning allowed).

## 4. Coordinate System & Units
- [ ] **Pulse or Degree/mm**: Does the controller output positions in native encoder pulses or user units (degrees/mm)?
- [ ] **Joint Units**: Degrees or Radians (if applicable)?
- [ ] **Cartesian Units**: Millimeters (mm) or Inches?

## 5. Frames Definition (Base / Tool / User)
- [ ] **Base Frame**: Where is the physical origin (0,0,0) of the Base frame located on the robot base?
- [ ] **Tool Frame (TCP)**: Which Tool number (0-63) is active? What are the precise X, Y, Z, Rx, Ry, Rz offsets?
- [ ] **User Frame**: Are any User frames defined and active for the task?

## 6. System Clock
- [ ] **Clock Sync**: Are the controller's internal date and time set correctly to match the PC for accurate logging?

## 7. Safety & Recovery
- [ ] **E-Stop locations**: Teach Pendant, Controller Cabinet, External barriers.
- [ ] **Recovery**: Procedure to reset an E-Stop or Alarm state via Teach Pendant.
