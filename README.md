2CAN is both hardware and software

Hardware: Atmel AT90CAN32 + Microchip MCP2515 + FTDI FT232R 
Software: embedded (avr-gcc) + host (python/tornado) + UI (HTML5/websockets)

The goal is to build a flexible CAN analyzer which can be inserted in-line
in a working CAN environment for the purposes of reverse-engineering an
existing network.  To facilitate this, frames are captured going in both
directions, and individual frames can be filtered out to observe how the
system behavior is affected.

This is a work in progress.


