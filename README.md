# Mercury 2 Programmer

The `mercury2_prog` application is a command-line utility for programming the [Mercury 2 FPGA board](https://micro-nova.com/mercury-2) via the onboard FT2232H USB-to-serial adapter. It can currently communicate with the S25FL116K (16Mbit), S25FL132K (32Mbit) and S25FL164K (64Mbit) SPI flash chips.

Port A of the FT2232H is used by this application for configuration. Port B is free for user applications. See the [Mercury 2 schematic](https://www.micro-nova.com/s/Mercury2_schematic.pdf) for a detailed look at how the FT2232H is wired to the FPGA and SPI flash.

## Usage

- `-h` or `--help` prints a help message describing these options
- `-l` or `--list` lists all visible FTDI devices, and attempts to find Mercury 2 boards
- `-e` or `--erase` erases the entire SPI flash on the Mercury 2
- `-r` or `--read` reads the entire SPI flash and saves it to `dump.bit`
- `-w` or `--write` writes to the SPI flash using the specified file. For example: `./mercury2_prog -w test.bit`. (Note that this command will automatically erase the required blocks before writing.)

## Build

The `mercury2_prog` application is compiled on Linux using `make`. This has been tested on Ubuntu 18.04 using gcc 7.4.0 and mingw32-gcc-7.3. The build produces two binaries:
- `mercury2_prog` - a native x86-64 Linux application
- `mercury2_prog.exe` - a cross-compiled x86-64 Windows application

Note that you must `sudo apt-get install gcc-mingw-w64-x86-64` to cross-compile for Windows.

## Sample Output

```
$ ./mercury2_prog -w mercury2_blink.bit
.----------------------------------.
|       MERCURY 2 PROGRAMMER       |
|      (c) 2019 MicroNova LLC      |
|        www.micro-nova.com        |
'----------------------------------'

Reading bitstream file 'mercury2_blink.bit'

Size is 231.8 kBytes = 237345 bytes = 1898760 bits
  928 x 256B pages   (927 whole pages + 33 byte remainder)
   58 x 4kB  sectors (57 whole sectors + 3873 byte remainder)
    4 x 64kB blocks  (3 whole blocks + 40740 byte remainder)

Reading file...
Checking for FTDI devices...3 FTDI devices found
Device 0:
  - Description = 'Mercury FPGA'
  - Serial #    = '77'
  - FTDI Chip   = FT232R @ 12Mbps
  - Opened      = No
Device 1:
  - Description = 'Mercury 2 FPGA A'
  - Serial #    = '100A'
  - FTDI Chip   = FT2232H @ 480Mbps
  - Opened      = No
Device 2:
  - Description = 'Mercury 2 FPGA B'
  - Serial #    = '100B'
  - FTDI Chip   = FT2232H @ 480Mbps
  - Opened      = No

Found Mercury 2 FPGA board at FTDI device 1.
FTDI device 1 opened successfully.
Configuring FTDI for SPI communication...OK
Found flash: S25FL132K 32Mbit

***** WRITING FLASH *****
Erasing block 3 / 3 = 100.0%
Writing page 927 / 927 = 100.0%
Programming time : 2.019 sec
Effective speed  : 114.413 kBytes/sec
```
