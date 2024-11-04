# Visualizer

This Python application runs on the desktop and is designed to pair up with the "fustion_tests" Arduino application.

1. Start VS Code.

1. Within VS Code, open the "desktop\visualizer" folder.

2. Within VS Code, open the TERMINAL window.

3. From the TERMINAL window, set up Python venv (recommended)

```
>  Set-ExecutionPolicy ExecutionPolicy RemoteSigned
>  python -m venv .venv
>  .venv\Scripts\activate  
```

4. Install the requisite Python packaages

```
> cd desktop\visualizer
> pip install -r requirements.txt
> pip install PyOpenGL PyOpenGL_accelerate PyQt5 pyserial vedo pywavefront
```

Note that the "requirement.txt file must be used to ensure the correct version of pyglet is installed to sypport PyWavefront.

5. Open a second VS Code Window. From that, open the "fusion_tests" PlatformIO project folder.

6. Connect your Tempo device via USB. Double tap the Tempo board's reset button to place it in upload mode (the red LED on the Tempo board will "throb" when the board is in upload mode). 

7. Build and upload the "fusion_tests" application. 

8. Use VS Code's "Serial monitor" tab to determine which Windows COM port is allocated to the Tempo device's USB serial interface (on my system, this is "COM8")

9. Edit the visualizer.py file and update the COM port setting to reflect your system's configuration

10. Run the visualizer application. Press Q to quit.

```
> python visualizer.py
```