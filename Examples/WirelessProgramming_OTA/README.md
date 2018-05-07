![GUI v1.6](https://raw.githubusercontent.com/LowPowerLab/RFM69/master/Examples/WirelessProgramming_OTA/OTAGUI.png)
# Wireless Programming for Moteinos


## v1.6 Changes
- various bugs fixed
- protocol improved to support variable HEX record length
- removed the delay logging since it was causing some glitching

## v1.5 Changes
Since v1.5 you can now run this app in several ways:
- natively via the WirelessProgramming.exe GUI app
- the windows GUI can also invoke the OTA.py script via embedded IronPython engine (parameters from GUI pass to the OTA.py script)
- cross platform straight from Python (2.7 runtime) by supplying parameters (run `python OTA.py -h` for details)
<br/>
You must download pythonLibs.zip and OTA.py and place them in the same directory as WirelessProgramming.exe if you plan to run the protocol via the OTA.py script.
You may need to download the entire repository as a ZIP file.
