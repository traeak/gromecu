#!/usr/bin/env bash
# End-to-end test through the kernel tty layer (Linux only): runs the
# simulated ECU on a pseudo-terminal and drives it with the real serial code.
#
#   zig build && scripts/pty-test.sh
set -euo pipefail

G=${GROMECU:-$(dirname "$0")/../zig-out/bin/gromecu}
G=$(realpath "$G")
T=$(mktemp -d)
EMU=
cleanup() { [ -n "$EMU" ] && kill "$EMU" 2>/dev/null; rm -rf "$T"; }
trap cleanup EXIT
cd "$T"

# Two valid images for the same ECU: "stock" is in the ECU, "tune" gets flashed.
head -c 49152 /dev/urandom > raw.bin
"$G" --force check raw.bin --fix-checksum 0 -o stock.bin >/dev/null
head -c 49152 /dev/urandom > raw.bin
"$G" --force check raw.bin --fix-checksum 0 -o tune.bin >/dev/null

mkfifo key
exec 3<>key                      # keep the fifo open; a newline toggles the ignition
"$G" emu --model K26-911 --image stock.bin --quiet <&3 >emu.out 2>emu.err &
EMU=$!
for _ in $(seq 50); do grep -q /dev/pts/ emu.out 2>/dev/null && break; sleep 0.1; done
PORT=$(grep -o '/dev/pts/[0-9]*' emu.out | head -1)
echo "emulator on $PORT"

echo "== info";  "$G" -p "$PORT" info
echo "== dtc";   "$G" -p "$PORT" dtc
echo "== live";  "$G" -p "$PORT" live --count 2

echo "== read"
( sleep 1.5; echo >&3; sleep 1.5; echo >&3 ) &      # ignition off, then on
"$G" -p "$PORT" read dump.bin --length 0x900
cmp <(head -c $((0x900)) stock.bin) dump.bin
echo "read matches"

echo "== write"
echo >&3; sleep 0.3; echo >&3; sleep 0.3            # power-cycle out of secure mode
"$G" -p "$PORT" --force --offset 0x4000 --yes write tune.bin

echo "== read back"
echo >&3; sleep 0.3; echo >&3; sleep 0.3
( sleep 1.5; echo >&3; sleep 1.5; echo >&3 ) &
"$G" -p "$PORT" read back.bin --start 0x8000 --length 0x400
cmp <(tail -c +$((0x8000 + 1)) tune.bin | head -c $((0x400))) back.bin
echo "PASS"
