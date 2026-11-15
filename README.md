# gromecu

Read and flash the ECU of a Honda Grom (MSX125) or Monkey (Z125M) over the
diagnostic K-line, from the command line, with a single static binary.

It is a Zig port of the K-line protocol code in
[HondaECU](https://github.com/RyanHope/HondaECU) / eculib by Ryan M. Hope,
with timing details and real-ECU captures from
[ECUemu](https://github.com/atoid/ECUemu). Unlike HondaECU it
talks to the adapter through the operating system's serial driver, so there
is no libftdi/libusb, no driver swapping, and any USB-serial K-line adapter
works (FTDI, CH340, CP210x).

> **Status: not yet tested on a motorcycle.** Everything here has been
> verified against the reference source code and a simulated ECU, not against
> hardware. Flashing an ECU can leave the bike unable to run until it is
> re-flashed. Read "What has and has not been verified" before using `write`.

## Supported ECUs

| Name      | Bike                      | ECM ID           | Image | Write offset |
|-----------|---------------------------|------------------|-------|--------------|
| `K26-911` | Grom / MSX125 2013-2015   | `01 01 35 05 01` | 48 k  | `0x4000`     |
| `K26-931` | Grom / MSX125 2013-2015   | `01 01 35 01 01` | 48 k  | `0x4000`     |
| `K26-B13` | Grom / MSX125 2016-2019   | `01 02 13 05 01` | 64 k  | `0x8000`     |
| `K26-C31` | Grom / MSX125 2016-2019   | `01 02 57 05 01` | 64 k  | `0x8000`     |
| `K0F-A01` | Monkey / Z125M 2018-2019  | `01 02 ca 05 01` | 96 k  | `0x8000` \*  |

The ECU is identified by the ECM ID it reports, not by model year: run
`gromecu info` to see which one your bike has. A 2015 Grom should be one of
the first two rows and a 2019 Monkey the last, but other markets may carry
ECUs that are not in this table; those can still be read, and flashed only
with an explicit `--offset` and `--force`.

\* HondaECU's own table lists only the ID and part number for the Monkey.
The offset comes from its README; the image size and ID location were taken
from the stock image in `HondaECU/bins`. Treat this row as less certain.

## Hardware

A USB to K-line adapter (a USB-serial converter plus a K-line level shifter;
HondaECU's README has a schematic) connected to the bike's red 4-pin
diagnostic connector: K-line, ground and +12 V. The battery must be healthy;
on a bench the ECU needs a supply that can deliver about 2.5 A.

## Building

Needs [Zig](https://ziglang.org) 0.17.

    zig build -Doptimize=ReleaseSafe        # -> zig-out/bin/gromecu
    zig build test                          # unit + simulator tests
    scripts/pty-test.sh                     # end-to-end over a pty (Linux)

Cross-compiling works the usual way, e.g. `-Dtarget=aarch64-linux`,
`-Dtarget=x86_64-windows`, `-Dtarget=aarch64-macos`.

### Linux setup

The adapter appears as `/dev/ttyUSB0` (or `/dev/ttyACM0`). You need
permission to open it: add yourself to the group that owns the device,
usually `dialout` (Debian, Fedora, Void) or `uucp` (Arch), and log in again.

    ls -l /dev/ttyUSB0
    sudo usermod -aG dialout "$USER"

If you previously used HondaECU with its udev rule or unbound the `ftdi_sio`
kernel driver, undo that: gromecu wants the normal kernel serial driver.

## Usage

    gromecu ports                     # which serial ports are there
    gromecu info                      # state, ECM ID, model, flash count
    gromecu dtc                       # trouble codes   (dtc --clear to erase)
    gromecu live                      # rpm, TPS, temperatures, battery, ...
    gromecu read stock.bin            # dump the flash
    gromecu check tune.bin            # size / checksum / which ECU it is for
    gromecu write tune.bin            # erase and flash

The port is taken from `--port`, else `$GROMECU_PORT`, else the only USB
serial port present. `gromecu --help` lists every option. `--trace` prints
each frame on the wire, which is the first thing to look at if something does
not work.

All commands except `read` want the ignition on and the kill switch on run.

### Reading

    gromecu read stock.bin

The ECU only allows its flash to be read after a password is sent right
after power-on, so the tool asks you to switch the ignition off and then on,
and continues by itself. A Grom dump takes a few minutes (12 bytes per
request). Afterwards it reports the size, whether the checksum is good and
which ECU the image belongs to. **Keep this file**: it is your way back.

Cycle the ignition once more when it is done.

The tool notices the ignition by whether the K-line echoes. If your cable
keeps the line powered with the ignition off (the tool then never sees
"off"), or you use `--no-echo`, pass `--manual` and press Enter after each
step instead. The dump is saved as it is read, so an interrupted read
leaves a partial file.

Reads start at address 0, which is HondaECU's default; whether that is the
first byte of the image on every ECU here is not confirmed. If the result
does not look like an image for your ECU, the tool suggests the alternative
`--start`/`--length` to try.

`--start` and `--length` read part of the flash. Without `--length` the read
runs until the ECU refuses, as HondaECU does.

### Writing

    gromecu check tune.bin
    gromecu write tune.bin

Before anything is sent, `write` refuses when:

- the image's 8-bit checksum is not zero (the ECU would not run it),
- the size is not a multiple of 128 bytes,
- the image cannot be identified as being for a supported ECU,
- the image is for a different ECU type than the one connected.

`--force` overrides the last two; `--model` and `--offset` say what an
unrecognised image is. If you edited a map, fix the checksum first with
`--fix-checksum <addr>`; the address of the checksum byte depends on the ECU
and is not guessed for you (HondaECU's table gives `0x0` for these ECUs, but
on the 2013-2015 Grom that byte is part of a vector table, so check the
definition file you tuned with).

Then it shows what it is about to do, asks you to type `yes`, and runs the
sequence: enter flash mode, wait 11 s, erase, write, finalize. The ECU
itself confirms the result at the end. Ctrl-C is ignored while flashing.

### If a flash is interrupted

The ECU is built for this: with an incomplete image it stays in a recovery
mode in which it still accepts a flash. Leave the ignition on and run the
same `write` command again. If that does not connect, switch the ignition
off and on and run it once more. `gromecu info` shows the state
(`recovery` or `flash mode`). If the ECU reports a flash error, `write`
stops before erasing and asks for an ignition cycle first, as the reference
does.

## Trying it without a bike

    gromecu --port sim:K26-911 info            # built-in simulated ECU
    gromecu emu --model K0F-A01                # simulated ECU on a pty (Linux)

`emu` prints a `/dev/pts/N` path to use as `--port` from another terminal;
pressing Enter in the emulator's terminal toggles the ignition.

## What has and has not been verified

Verified:

- Request frames, both checksums, the read-address encoding and the flash
  block layout are tested byte-for-byte against output of the reference
  Python code and against frames ECUemu captured from a real ECU.
- The command sequences and delays for wake-up, state detection, security
  access, reading, entering flash mode, erasing, writing (including the
  64-byte fallback and the retry handling) and finalizing are ported
  line by line from eculib 1.0.9 and HondaECU's `kline.py`.
- Read, flash, flash with a lost acknowledgement, and recovery after an
  interrupted flash run end to end against the simulated ECU, both
  in-process and through a Linux pty with the real serial code.

Not verified:

- **Nothing has been run against a real ECU.** The simulated ECU answers the
  way the reference code expects; it shows the port is self-consistent, not
  that the bike agrees.
- The simulated K-line cannot check the 10400 baud rate or the wake-up pulse
  electrically.
- macOS and Windows serial code compiles but has never been run.

Inferred, because the newest eculib (1.0.37, which the current HondaECU uses)
is no longer available and only 1.0.9 could be consulted:

- After the erase command the tool goes straight to polling the erase
  status; the reference first checked a result flag whose logic is lost.
- Commands are retried up to 3 times, HondaECU's shipped setting; how
  1.0.37 applied that setting per command is unknown (`--retries` changes it).
- "Secure" (flash readable) is detected by trying a read; the reference's
  exact detection order in 1.0.37 is unknown.
- The K-line presence check sends `0xff` and looks for its echo, as older
  HondaECU versions did.

A sensible first session on a real bike is `info`, `dtc`, `live` (all
harmless), then `read`, and comparing the dump against the stock image for
your ECU from `HondaECU/bins` with `gromecu check` / `cmp`. A dump that
matches byte for byte is strong evidence the link, timing and addressing are
right. Only then consider `write`.

## Layout

    src/transport.zig   byte transport interface (with time, for the simulator)
    src/serial.zig      serial port: Linux termios2, macOS, Windows
    src/protocol.zig    frames and checksums, with reference vectors
    src/ecu.zig         ECU operations: detect, DTCs, read, flash
    src/models.zig      supported ECUs and trouble-code texts
    src/sim.zig         simulated ECU
    src/emu.zig         simulated ECU on a pty
    src/main.zig        command line
    src/tests.zig       end-to-end tests against the simulator

## Credits and licence

Protocol knowledge comes from HondaECU and eculib by Ryan M. Hope (GPL-3) and
from ECUemu. As a derivative work this project is licensed under the GNU
General Public License v3; see `LICENSE`.
