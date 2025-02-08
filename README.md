# TCS Intercom Controller

Controls the TCS home intercom system by implementing its proprietary bus protocol.

The code has been implemented and tested on an Arduino Pro Mini clone board based on the ATmega328P microcontroller.

The TCS proprietary protocol encoding/decoding routines have been borrowed from https://github.com/atc1441/TCSintercomArduino. At this point, I would like to thank the author for his outstandig contribution.

This project uses Git submodules. In order to get its full source code, please clone this Git repository to your local workspace, then execute the follwoing command from within the repository's root directory: `git submodule update --init`. Alternatively, you may simply download and unzip `tcs-intercom-x.y.z-full.zip` from the latest release under https://github.com/microfarad-de/tcs-intercom/releases, then open `tcs-intercom.ino` using the Arduino IDE.

Unless stated otherwise within the source file headers, please feel free to use and distribute this code under the GNU General Public License v3.0.

## Circuit Diagram

<p align="center">
<img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/tcs-intercom-schematic.png" alt="drawing"/>
</p>

[tcs-intercom-schematic.pdf](https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/tcs-intercom-schematic.pdf)

## Gallery

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/layout.jpg" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/perspective-1.jpg" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/perspective-2.jpg" alt="drawing" width="600"/>
 </p>

 <p align="center">
 <img src="https://raw.githubusercontent.com/microfarad-de/tcs-intercom/master/doc/perspective-3.jpg" alt="drawing" width="600"/>
 </p>


## Notes

This project can be compiled from command line using the via makefile using the following commands:

* Compile: `make`
* Remove compiled artefacts: `make clean`
* Upload to Arduino board: `make upload`
* To generate the release archive: `make release`

This project is configured for using the clangd VSCode plugin for code indexing. In order to use the clangd plugin, `compile_commands.json` file needs to be generated. This file is already part of the rpository, however it needs to be regenerated if new source files have been added to the project. The generation of `compile_commands.json` has been performed using the `compileddb` Python tool (`pip install compiledb`, https://pypi.org/project/compiledb/). The generation si performd using the following command: `compiledb make`.

