# ReaperRemote

ReaperRemote is a remote control for Reaper/Ultraschall.fm. It connects via Bluetooth LE to the Computer and sends MIDI messages to Reaper.

The remove can be configured by using simple AT commands from the host PC over the wired serial interface or the Bluetooth Uart.


## AT-Commands

### Setting the action for button 1, down state
AT+BTD1?

AT+BTD1=B0017F
AT+BTU1=B00100

AT+BTD2=900040
AT+BTU2=800040

AT+NAME=MyRemote1
