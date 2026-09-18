# FS100 Controller PC Communication Notes

## Yaskawa Official / First-Hand Information Sources

1. **High-Speed Ethernet Server Function (HSE)**
   - **Manual:** "FS100 Options Instructions for High-Speed Ethernet Server Function"
   - **Applicability:** Used for basic read/write of robot data, job editing, and real-time status monitoring from a PC. Also used internally by MotoCom. 
   - **Communication:** Typically TCP/IP sockets via standard Ethernet port (LAN1).
   - **Requirements:** Requires a paid option/license from Yaskawa to be activated.

2. **MotoCom SDK**
   - **Applicability:** Official PC-side library/SDK (DLLs for Windows, sometimes shared libraries) providing an API over the standard Ethernet Server protocol.
   - **Requirements:** Requires the "Ethernet Server Function" (or High-Speed Ethernet Server) option on the controller side. Often sold as a licensed software package.

3. **MotoPlus SDK**
   - **Applicability:** C/C++ SDK that runs directly on the FS100 controller. Allows developers to write custom server applications (e.g., exposing a custom UDP/TCP API for 8ms control loops).
   - **Requirements:** MotoPlus development environment (IDE/Compiler) and the MotoPlus option enabled on the controller.

## Unknowns / Action Items
- **Actual packet bytes/structure:** Yaskawa Ethernet Server protocols are proprietary. We must rely on an official Yaskawa manual (containing the actual binary/ASCII packet formats) or a vendor-provided mock/SDK to implement the socket layer. We cannot guess the byte layout.
- **License status:** Is the "Ethernet Server", "High-Speed Ethernet Server", or "MotoPlus" option already purchased and activated on this specific FS100 unit?

## Reference Links
- [Yaskawa Document Search Portal](https://www.motoman.com/en-us/service-support/documentation)
- General reference: https://www.motoman.com/en-us/
