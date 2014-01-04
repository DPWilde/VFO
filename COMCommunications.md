Standard of COM Communications
======
Standards
------
1. First Character is command
2. Commands terminated by carriage return
3. Commands and arguments are seperated by spaces

To Arduino
======

Set Frequency
------
`F <frequency in Hz>`

Oscillator ON
------
`Y`

Oscillator OFF
------
`N`

Set Start Frequency
------
`L <frequency in Hz>`

Set Stop Frequency
------
`H <frequency in Hz>`

Set Interval
------
`I <frequency in Hz>`

Start Scan
------
`S`

Stop Scan
------
`X`

Return Presets
------
`P`

Send Custom Preset
------
`C <name> <start frequency in Hz> <stop frequency in Hz>`

To PC
======

Send Data for Freq
------
`D <frequency in Hz> <Voltage1> <Voltage2>`

Return Preset
------
`P <index> <name> <start frequency in Hz> <stop frequency in Hz>`

Examples
======

Start Scan
------
PC:`L 3500000`

PC:`H 3800000`

PC:`I 5000`

PC:`S`

When scan finished:

Anylyser:`X`

Get Presets
------
PC:`P`

Anylyser:`P 0 80m 3500000 3800000`

etc. For all presets

Send Custom Preset
-------
PC:`C 60m 5258500 5406500`

Anylyser:`P 12 60m 5258500 5406500`
