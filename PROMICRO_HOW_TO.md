# How to get PebbleOS up and running on a ProMicro nRF52840
## Prerequisite
You will need:
- A ProMicro/Supermini nRF52840
- A W25Q flash module
- GC9A01 240x240 display
- 4 buttons
- Something to flash firmware onto the board via SWD. I use both the J-Link debugger on the nRF52840 DK and the ST-Link V2 that comes with a Nucleo board. For this guide, I recommend the Nucleo ST-Link since it also doubles as a USB-UART adapter (this will come in handy later)
- A USB-UART adapter, if you are not using the STM32 Nucleo's ST-Link

## Building the firmware
Clone the fork, and follow [these instructions](https://pebbleos.readthedocs.io/en/latest/getting_started.html) for building the firmware until the end of the Building section. Note that the arguments for the `./waf configure` command are as follows:
- `--board=promicro`
- `openocd_jtag`: If you're using an ST-Link, use `openocd_jtag=swd_stlink`. If you're using a J-Link debugger like on the nRF52840 DK, use `openocd_jtag=swd_jlink`. For other debuggers, consult the JTAG_OPTIONS table in [waftools/openocd.py](https://github.com/liaaa444m/pebble-firmware/blob/main/waftools/openocd.py)

## Flashing the firmware
1. Connect your debugger of choice to the ProMicro. There should be 4 pads underneath the ProMicro board labeled VDD, DIO, CLK, and GND. Depending on your debugger of choice, the wiring will be different. Below is how I connected my ST-Link and my nRF52840 DK's J-Link to the ProMicro:
    - ST-Link:
        - VDD_Target to the VCC pin or the VDD pad (truthfully I don't entirely know why connecting this pin to the VCC pin works, but it works and it doesn't look like anything is broken. Proceed at your own risk.)
        - SWCLK to the CLK pad
        - GND to GND (either pad or pin should be fine, but when I used the VCC pin, GND wad connected to the GND *pin*, so if you're gonna use the VCC pin you should probably use the GND pin as well since that worked for me.)
        - SWDIO to the DIO pad
        - The ProMicro board **has** to be externally powered. Simply powering the board through the USB port will do the job.
    - nRF52840 DK: (**DO NOT POWER THE BOARD EXTERNALLY USING THIS METHOD**)
        - VDD_nRF' to VDD (it **must** be the VDD pad, otherwise the chip won't be properly powered and flashing will fail)
        - VDD_nRF to SWDSEL (both pins have to be connected on the DK, otherwise you'd be flashing the DK's chip)
        - SWDIO to the DIO pad
        - SWDCLK to the CLK pad
        - GND to GND (I generally use the pad)
Personally, I think the ST-Link method is much easier to work with since you only have to deal with the DIO and CLK pds, and for that all you really have to do is touch them with jumper wires. 

2. Flash the firmware using `./waf flash_fw`.

*Note: I have yet to write the bootloader for the ProMicro board, so right now the board boots right into the OS itself instead of through the bootloader like the actual watches. This does mean that the watch cannot be shut down through the OS, nor can it be updated OTA. Unfortunately I only just got the firmware to run well enough on the board before my summer break ended, so it'll take a while for me to get to writing the bootloader. You are more than welcome to help, but do remember to change the firmware's flash offset size in src/fw/wscript. I would not recommend trying to make a functioning watch using this fork of PebbleOS for the time being. I do highly recommend making a desk clock though, just not a battery-powered one.* 

## Actually getting the firmware to boot
1. Connect the W25Q module to the board as follows:
    - VCC to VCC
    - GND to GND
    - CS to 029
    - CLK to 115
    - DO (or D0) to 113
    - DI (or D1) to 111
2. Connect your USB-UART adapter or the Nucleo ST-Link to the board as follows:
    - Adapter's TX to 008
    - Adapter's RX to 006
3. Power the board, wait for about 10 seconds, then run `./waf image_resources --tty=$SERIAL_ADAPTER`, with `$SERIAL_ADAPTER` being whatever your adapter shows up as on your machine. For a Nucleo ST-Link and a Mac, the tty address is usually /dev/tty.usbmodemXXXXX
4. During the `image_resources` flash, one of these cases will happen:
    - If the command succeeds, congratulations, you've done the impossible. gg i guess
    - If the command throws an error after writing data for a while, there's a good chance the resource flashing has actually worked. Reset the board and check the console output using `./waf console --tty=$SERIAL_ADAPTER` (or look at the screen if you've connected one. If it succeeded, you should see the charging coffee mug icon.)
    - If the command fails before it prints `Erasing...` and says something about NoneType, **power cycle** the board and try again. If that doesn't work, just keep trying until it works. If *that* doesn't work either, god help you.

You should now *finally* have PebbleOS running on your ProMicro board. Have fun! If there's any trouble or questions, you're welcome to ask them in the Discord thread linked in the README.

## Pin assignment for the GC9A01 display, buttons, and battery
- GC9A01: (Despite some pins being labeled as SCL and SDA, basically every GC9A01 display module uses SPI)
    - VCC to VCC
    - GND to GND
    - SCL to 017
    - SDA to 020
    - CS to 022
    - DC to 024
    - RST to 100
    - BLK to VCC (if your module has one. Mine doesn't, so I haven't been able to make configurable display brightness work.)
- Buttons: connect one terminal to the corresponding pin, and the other to ground
    - Back: 011
    - Up: 104
    - Select: 106
    - Down: 031
- Battery:
    - Connect the battery's positive and negative terminals to B+ and B- respectively
    - You can hook the battery to a voltage divider (specifically one that divides the voltage in half) and connect it to pin 002 for a crude way to measure battery percentage. Very inaccurate.
