# Picapot Irrigation Controller


This repository contains the latest firmware for the Picapot Irrigation Controller HT328.3.2.

The hardware design, user manual, 3D printable enclosure, and complete project documentation are freely available at:

https://www.picapot.com

## Build and Upload

A precompiled `.hex` file is available for direct upload to the microcontroller. If you don't need to change anything of the program, you can just upload the precompiled firmware.

### Compile from Source 

The code has been developed using the Arduino IDE and should be compiled using the GyverCore board package. You can add this board to the Arduino IDE by adding the following line to **Settings → Additional Boards Manager URLs**:

```text
https://alexgyver.github.io/package_GyverCore_index.json
```

**GyverCore board settings**

- B.O.D.: `2.7V [Default]`
- Bootloader: `Without bootloader [Warning!]`
- Clock: `External 16 MHz`
- Clock out: `Disable [Default]`
- Compiler version: `AVR-GCC v5.4 or 7.3 [Default]`
- Save EEPROM: `Enable`
- Initialization: `Enable [Default]`
- Serial: `Default Serial`
- System timer: `Enable [Default]`

### Upload to the Controller

The compiled `.hex` file can be uploaded to the microcontroller via port P1 using an AVR programmer and the [AVRDUDE](https://github.com/avrdudes/avrdude/releases) command-line tool.

The first command sets the correct fuses:

```console
avrdude -c avrispmkii -p atmega328p -B 100 -v -U lfuse:w:0xEF:m -U hfuse:w:0xC7:m -U efuse:w:0xFD:m
```

The second command uploads the program to flash memory:

```console
avrdude -c avrispmkii -p atmega328p -B 0.5 -v -U flash:w:picapot-controller-ht328.ino.hex:i
```

The programmer used in the commands above is the AVR ISP MKII (`avrispmkii`). If you are using a different programmer, update the programmer ID in the commands accordingly.

To run the commands as shown, AVRDUDE must be available in the system `PATH`, and the `.hex` file must be in the terminal's current working directory.

### First Run

During the first execution of the firmware, the controller scans the 1-Wire network to detect the internal DS18B20 temperature sensor and stores its address in EEPROM. **It is important that no external sensors are connected at this stage**; otherwise, the controller will not be able to distinguish between the internal and external sensors.

## Repository Contents

- **picapot-controller-ht328.ino** — Firmware source code
- **picapot-controller-ht328.ino.hex** — Precompiled firmware for 128×64 SPI OLED displays based on SSD1306 and SSD1309 controllers

## Dependencies

- **Wire (I²C)**  
  https://github.com/esp8266/Arduino/blob/master/libraries/Wire/Wire.h

- **OneWire**  
  https://github.com/PaulStoffregen/OneWire

- **Dusk2Dawn**  
  https://github.com/Picapot/Dusk2Dawn  
  (fork of https://github.com/dmkishi/Dusk2Dawn)

- **SSD1306Ascii**  
  https://github.com/Picapot/SSD1306Ascii  
  (fork of https://github.com/greiman/SSD1306Ascii)
