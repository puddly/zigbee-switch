# Firmware for Xiaomi Wireless Switch

Hold down the left button (when looking at the PCB with the LEDs on top) to join an netowork when booting.
You then need to bind to the first endpoint's On/Off cluster (0x0006) to receive attribute reports. Only the right button works for now.
Sleeping has been disabled so it will drain your battery very quickly. A stable 3V power supply is recommended.

## Flashing

On the wireless switch the small row of five pads on the bottom of the PCB expose a JTAG interface. Alternatively, find each pad's associated testpoint so 
you can solder a thicker wire to it.

You will need to short together a few pins to put the board into programming mode. This can be done in software with an FTDI adapter's two programmable pins 
or just by hooking up some buttons.

## Building

 1. Install NXP studio
 2. Install Zigbee 3.0 SDK
 3. Install NXP studio plugins from SDK
 4. Import project

## TODO

 - [ ] Rename occupancy sensor to button
 - [ ] Add shared codebase for all Xiaomi switches (both the hardwired and the wireless ones)
 - [ ] Re-enable sleep mode
 - [ ] Make the second button work
