# A Tempo-BT communication extension for smpmgr & BLE scanning

## Scanning via bluetoothctl

`blutetoothctl` is a modern tool capable of locating BLE devices.  It requires bluetooth group membership to run on Ubuntu, but does not reqire root access.

In this example, we invoke the `bluetoothctl` command and pass in `scan on` at the '#' command prompt.  Output can then be parsed. Sending a Ctrl-D or closing STDIN terminates the process.

Example output - in this case `Tempo-BT-0004` is an operational Tempo-BT device.

```bash
Agent registered
[CHG] Controller DC:A6:32:DB:FE:C0 Pairable: yes
[bluetooth]# scan on
Discovery started
[CHG] Controller DC:A6:32:DB:FE:C0 Discovering: yes
[NEW] Device C8:43:CA:EB:FE:6D Tempo-BT-0004
[NEW] Device A8:51:AB:94:7E:03 A8-51-AB-94-7E-03
[CHG] Device C8:43:CA:EB:FE:6D RSSI: -76
[NEW] Device 62:CD:E5:D8:31:0A 62-CD-E5-D8-31-0A
```

## Custom Tempo commands supported by smpmgr

We have developed a smpmgr custom plugin. This polugin adds several tempo-BT-specific commands to the core command set.

```
# Get help
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo --help
                                                                                                                                                                                         
 Usage: smpmgr tempo [OPTIONS] COMMAND [ARGS]...                                                                                                                                               
                                                                                                                                                                                               
 Tempo-BT custom commands (Group 64)                                                                                                                                                           
                                                                                                                                                     
                                                                                                                                                                                               
╭─ Options ───────────────────────────────────────────────────────────────────────────────────────╮
│ --help          Show this message and exit.                                                     │
╰─────────────────────────────────────────────────────────────────────────────────────────────────╯
╭─ Commands ──────────────────────────────────────────────────────────────────────────────────────╮
│ session-list     List all logging sessions on the device.                                       │
│ storage-info     Get storage statistics from the device.                                        │
│ led-on           Turn on the LED with specified color.                                          │
│ led-off          Turn off LED override (return to app control).                                 │
│ logger-start     Start logging (will auto-arm if needed).                                       │
│ logger-stop      Stop logging.                                                                  │
│ logger-arm       Arm the logger.                                                                │
│ logger-disarm    Disarm the logger.                                                             │
│ logger-control   Generic logger control command.                                                │
╰─────────────────────────────────────────────────────────────────────────────────────────────────╯
```

## Get information about the current primary storage (either SD card or QSPI)
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo storage-info

⠴ Connecting to Tempo-BT-0004... OK
          Tempo-BT Storage Info           
┏━━━━━━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━┓
┃ Property    ┃ Value                    ┃
┡━━━━━━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━┩
│ Backend     │ internal                 │
│ Total Space │ 6.9 MB (7,208,960 bytes) │
│ Free Space  │ 6.9 MB (7,192,576 bytes) │
│ Used        │ 1%                       │
└─────────────┴──────────────────────────┘
```

## Get information about the current primary storage (either SD card or QSPI)
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo session-list

⠦ Connecting to Tempo-BT-0004... OK
No sessions found

# or outpput that will include this JSON object

{
  "sessions": [
    {"name": "20250115/ABC12345", "is_dir": true, "size": 0},
    {"name": "20250115/DEF67890", "is_dir": true, "size": 0},
    {"name": "20250117/7BF3655C", "is_dir": true, "size": 0}
  ],
  "count": 3
}


```

## Start logging (will auto-arm if in IDLE state)
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo logger-start
```

## Stop logging
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo logger-stop
```

## Arm the logger
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins logger-arm
```

## Disarm the logger
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo logger-disarm
```

## Or use the generic control with action parameter
`smpmgr --ble Tempo-BT-0004 --plugin-path=plugins logger-control start`

## Override blink the RGB LED with a given color
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo led-on orange

⠴ Connecting to Tempo-BT-0004... OK
LED set to orange (RGB 255,128,0)
```

## Disable LED override
```
smpmgr --ble Tempo-BT-0004 --plugin-path=plugins tempo led-off

⠼ Connecting to Tempo-BT-0004... OK
LED override disabled - returned to app control
```

# Standard smpmgr commands useful to Temp-BT interactions

## The standard fs management should support delete
`smpmgr --ble Tempo-BT-0004 --plugin-path=plugins fs delete /lfs/logs/20250117/12345678/flight.csv`