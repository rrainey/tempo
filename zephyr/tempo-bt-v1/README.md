

## Tempo-BT V1 Project Structure:

```
tempo-bt-v1/
├── CMakeLists.txt              # Main build configuration
├── prj.conf                    # Kconfig settings
├── boards/
│   └── nrf5340dk_nrf5340_cpuapp/
│       └── tempo_v1.overlay    # Device tree overlay with hardware config
├── config/
│   └── partitions.yml          # Partition layout documentation
├── include/
│   └── app_init.h              # Application initialization headers
└── src/
    ├── main.c                  # Main application entry
    └── app_init.c              # Storage initialization
```

The actual partition configuration is in the device tree overlay.
MCUboot partitions will be added when we enable DFU support.