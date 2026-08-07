This is where the bootloader for the platforms are. You will most likely not touch this part of the
project for a while, so just leave it be for the time being.
You stil need to deal with platform_capabilities.py though.
In the wscript file, nucleo's bit depth is set to 8. I don't know if that is what we need (pretty sure the lowest the GC9A01 displays take is 12 bits), but we'll cross that bridge when we get there.

Okay damnit it seems like you might actually have to write the bootloader because this thing NEEEEDS CPU_FLAGS and those are defined in the boards' individual wscript

Each platform should be independent from one another.

For more information about the bootloader design:
https://pebbletechnology.atlassian.net/wiki/display/DEV/Bootloader+Contract
