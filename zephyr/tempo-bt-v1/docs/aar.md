# After Action Review application

## Key functions and characteristics

**Runs on a RaspBerry Pi 5** The application shall be designed to run on a raspberry pi 5 Linux system.  

**Capable of interacting with many Tempo-bt loggers concurrently** The application will be pre-paired with up to 200 tempo-bt loggers.  The application will scan for tempo-bt loggers coming in range and, once detected, will query the device and automatically upload via Bluetooth and store any complete log files for interactive analysis.

**A separate iOS App allows users to configure a Tempo-BT device** We will create a separate basic IOS application.  This application would be designed to allow the owner of a single tempo-bt device to provide information to the device identifying the skydiver.  Each time a new log file is created, this information would be included at the start of the file to allow specific skydivers to be distinguished when the log files are analyzed.
