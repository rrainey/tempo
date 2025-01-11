# Visualizer

This Python application runs on the desktop and is designed to pair up with the "fusion_tests" Arduino application allowing you to 
visualize the current calculated orientation of the Tempo device. 

This application demonstrates an important capability of the Tempo device: Tempo uses the on-board Inertial Measurement chip to compute
the skydiver's orientation in real time. This computed orientation is part of what's
logged during a jump.

You will run the fusion_tests application
on your Tempo device while it is connected to your desktop machine
via a USB cable.  The fusion_tests application sends a continuous stream
of orientation data to the desktop machine, which will be
interpreted and rendered by the running visualizer app.

### Preparing your Python environment and run the application

1. Start VS Code.

2. Within VS Code, open the "desktop\visualizer" folder.

3. Open the TERMINAL panel in the open VS Code folder.

4. From the TERMINAL panel, set up Python venv (recommended)

```
>  Set-ExecutionPolicy ExecutionPolicy RemoteSigned
>  python -m venv .venv
>  .venv\Scripts\activate  
```

The Set-ExecutionPolicy change was required on my Windows machine in order to
allow Python's venv scripts to run properly.  If you are concerned about
the security implications of running this command,, you may safely switch to some other
Python environment manager, such as conda, although the specific steps required will be different than what's shown here.

5. Install the requisite Python packaages

```
> cd desktop\visualizer
> pip install -r requirements.txt
> pip install PyOpenGL PyOpenGL_accelerate 
```

Note that the "requirement.txt file must be used to ensure the correct version of pyglet is installed to sypport PyWavefront.

*Installation of the PyOpenGL_accelerate package is optional*. Be aware that installation may require extra steps to complete installation of this package.  At the time that I write ths, it appears that 
this package requires that Microsoft Visual Studio be installed on your Windows machine in order to perform
the installation.  In my case, I used Google to [locate and install a prebuilt version of this package](https://stackoverflow.com/questions/77346740/install-pyopengl-for-windows-under-python-3-12).  You may be using
a different version of Python, so you'll likely need to reserach this
on your own to find a working solution.

5. Open a second VS Code Window. From that, open the "fusion_tests" PlatformIO project folder.

6. Connect your Tempo device via USB. Double tap the Tempo board's reset button to place it in upload mode (the red LED on the Tempo board will "throb" when the board is in upload mode). 

7. Build and upload the "fusion_tests" application project.

8. Set the Tempo device down on your desk. Orient it so that the USB connector (and cable) come out of the left side of the exclosure.

8. Use VS Code's "Serial monitor" tab to determine which Windows COM port is allocated to the Tempo device's USB serial interface (on my system, this is "COM8")

9. Edit the visualizer.py file and update the COM port setting to reflect your system's configuration


10. Run the visualizer application. You can then pick up the Tempo enclosure and reorient it to any position.  The rendered display should mirror all orientation changes.

```
> python visualizer.py
```

11. Press Q to quit the application.
