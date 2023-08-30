# GDB Debugging of STM32 with BlackMagic Probe
Jacob Chisholm - QFSAE
---

## Step 1: Install GDB
https://developer.arm.com/downloads/-/gnu-rm

## Step 2: Generate `firmware.elf`

#### PlatformIO:

Run the PlatformIO build task. The `firmware.elf` binary is located in `cwd\build\genericstm32f*\`

#### Bare Metal

Either use the `make build` command from the projects Makefile, or run `arm-none-eabi-gcc main.c -mcpu=cortex-m4

## Step 3: Open GDB environment

run command `gdb` or `arm-none-eabi-gdb`

## Step 4: Load Target

First, use device manager or `lsusb` to locate the debug port. Most times this is `COM3`. Then run the command `target extended remote COM3` replacing `COM3` with whatever port the debugger is on.

Once you have run the target command, also run the command `set mem inaccessible-by-default off`. More information about this command can be found <a href="https://black-magic.org/usage/gdb-commands.html">here</a> on the black magic website.

## Step 5: Locate and attach GDB to the target

Start by using the command `monitor swdp`. If this command returns no devices, try `monitor jtag`.

Then use the command `attach 1` replacing `1` with the correct location of the target.

## Step 6: Load the Firmware onto the Target

To start debugging, load the `firmware.elf` onto the stm32 using the `file firmware.elf` command. If asked to load new symbol tables, select yes.

## Step 7: Set Break/Watch Points

Currently I have found watch points more useful. They stop the program upon a variable changing its value. Watch points report the old and new value once changed. Once the watch point stops the program, the program can be resumed using the `continue` command, or `c` for short.

Set a watch point with the command  `watch var` where   `var` is the variable we are tracking for changes.