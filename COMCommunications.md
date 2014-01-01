Standard of COM Communications
======
Standards
------
1. First two Characters are command
2. Commands terminated by carriage return

To Arduino
======

Set Frequency
------
`F <frequency in Hz>`

Oscillator ON
------
`ON`

Oscillator OFF
------
`OF`

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
`SS`

Return Presets
------
`PR`

Send Custom Preset
------
`SP<name> <start frequency in Hz> <stop frequency in Hz>`

To PC
======

Send Data for Freq
------
`D <frequency in Hz> <Voltage1> <Voltage2>`

Return Preset
------
`PR<index> <name> <start frequency in Hz> <stop frequency in Hz>`

Examples
======

Start Scan
------
PC:`L 3500000`

PC:`H 3800000`

PC:`I 5000`

PC:`S`

When scan finished:

Anylyser:`SS`

Get Presets
------
PC:`PR`

Anylyser:`PR0 80m 3500000 3800000`

etc. For all presets

Send Custom Preset
-------
PC:`SP60m 5258500 5406500`

Anylyser:`PR12 60m 5258500 5406500`
