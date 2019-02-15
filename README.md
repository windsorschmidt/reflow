# Reflow Oven Controller

_"Yeah man, but it's a **dry** heat!."_ —PFC William L. Hudson

## About

Hardware and firmware sources for a 2-channel reflow (toaster) oven controller.

In spite of great alternatives such as [Controleo3](http://www.whizoo.com/controleo3), I decided to build my own controller for personal use. The design is intended to be simple and self-contained. Comments and pull requests are welcome!

![3D Rendering of PCB](pcb.png?raw=true "3D Rendering of PCB")

## Hardware Features

- Two triac-controlled power outputs (no external SSRs needed)
- AC line zero-crossing detection for reduced switching noise
- Temperature sensor using MAX31855 K-type thermocouple amplifier
- Compact user interface with OLED display and Cherry MX key-switches
- Data-logging output though USB and serial-to-Bluetooth module
- Tone transducer for audible alerts (e.g. reflow cycle complete)
- Mounts in a dedicated enclosure for simple wiring using IEC connectors
- [Aliens](https://en.wikipedia.org/wiki/Aliens_(film)) film inspired aesthetics

## Firmware Features

- Non-profiling PID routine controls triac outputs to track the active reflow profile
- Reflow profile can be viewed & edited at run-time (currently no ability to save edits)

![Main and profile UI displays](ui.png?raw=true "Main and profile UI displays")

## References

- [This project](https://www.allaboutcircuits.com/projects/controlling-ac-mains-with-a-microcontroller-for-fun-and-profit) was used as a basis for the triac control and zero-crossing detection circuitry
- [This Reddit user](https://www.reddit.com/user/rich-creamery-butter) provided [Assembly pics](https://imgur.com/a/sCKgO) used as a guide when converting my own oven
- The excellent [u8g2](https://github.com/olikraus/u8g2) library used to render user-interface graphics and text
- [Yusuke Kamiyamane](mailto:p@yusukekamiyamane.com)'s Tempesta Seven font used in the gnuplot script
- [KiCad](http://kicad-pcb.org/) EDA suite used for hardware schematic capture and PCB layout

## Disclaimer

This information is supplied "as is" and without warranties of any kind, express, implied, or statutory including, but not limited to, any implied warranty of merchantability, noninfringement or fitness for a particular purpose and no responsibility is assumed by the author for its use, nor for any infringements of patents or other rights of third parties that may result from its use.

In no event will the author be liable for any loss or damage including without limitation, indirect or consequential loss or damage, or any loss or damage whatsoever arising from or in connection with the use of this information.

## License

[MIT license](LICENSE)
