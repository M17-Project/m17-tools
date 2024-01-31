# m17-tools
A set of M17 tools based on [original m17-cxx-demod toolset](https://github.com/mobilinkd/m17-cxx-demod) by Rob Riggs, WX9O.

## m17-mod-gui
A screenshot of `m17-mod-gui` running on Windows:

![obraz](https://user-images.githubusercontent.com/44336093/212475254-07605e95-427c-4a94-aff5-911f41005a0e.png)

## m17-demod
This program reads a 48k samples per second 16-bit, little-endian, single
channel, M17 4FSK baseband stream from STDIN and writes a demodulated/decoded
8k SPS 16-bit, single channel audio stream to STDOUT.

Some diagnostic information can be written to STDERR while the demodulator is
running.

## m17-mod
This program reads in an 8k samples per second, signed 16-bit little-endian, single channel raw audio
stream from STDIN and writes out an M17 4FSK baseband stream at 48k SPS,
16-bit, 1 channel to STDOUT.

## m17-gateway-link_mod
This program connects to the running [M17Gateway](https://github.com/g4klx/M17Gateway) instance via selected port.
It translates M17-over-IP traffic (reflector traffic) to M17 baseband. External system commands can be used
for PTT signal control, see `-T` and `-O` options' description below.

## m17-gateway-link_demod
This program connects to the running [M17Gateway](https://github.com/g4klx/M17Gateway) instance via selected port.
It translates M17 baseband to the M17-over-IP traffic (reflector traffic).

## Build

### Prerequisites

This code requires the codec2-devel, boost-devel, gtest-devel [and libgl-dev, libasound2-dev, and xorg-dev for the gui app] packages be installed.

It also requires a modern C++17 compiler (GCC 8 minimum).

### Build steps - All apps
    sudo apt-get install libgl-dev xorg-dev libasound2-dev 
    cd m17-tools
    cd thirdparty/rtaudio
    mkdir build
    cd build
    cmake ..
    make
    cd ../../../
	
    mkdir build
    cd build
    cmake ..
    make
    make test
    sudo make install
	
### Build steps - Command line apps only.
    cd m17-tools
    mkdir build
    cd build
    cmake .. -DBUILD_GUI_APPS=OFF
    make
    make test
    sudo make install

### Build for macOS using MacPorts
    sudo port install pkgconfig boost gtest portaudio

Then follow the build steps above from "cd m17-tools".

## Build Steps for local building under Anaconda for Windows

### Prequisites
- Microsoft Visual Studio 2019
- Miniconda (or Anaconda) x64 for Windows

### From a clean Conda environment

    conda config --add channels conda-forge
    conda create -n M17 vs2019_win-64 cmake ninja pkg-config boost-cpp gtest gmock gtest libcodec2
    conda activate M17

And then from the top level of the m17-tools repo, execute win_build.bat

## Running

### m17-demod

This program was designed to be used with RTL-SDR, specifically rtl-fm.

    rtl_fm -E offset -f 439.5M -s 48k | m17-demod -l | play -b 16 -r 8000 -c1 -t s16 -

You should run this in a terminal window that is at least 132 characters wide. It
will output diagnostic information on a single line in the window.

### m17-mod

    sox 8k.wav -t raw - |  ./m17-mod -S AB1CD -D AB2CD | ./m17-demod -l -d | play -q -b 16 -r 8000 -c1 -t s16 -

The input audio stream must be single channel, 16-bit little-endian, 8ksps.
The output of the modulator is 48ksps, 16-bit little-endian, single channel raw audio.

If you have a 16 bit stereo 48000 Hz wav file, it can be converted to the input audio format with a command like the following:

    sox /tmp/recorded.wav -r 8000 -b 16 -c 1 /tmp/8k.wav

To output a bitstream file:

    sox 8k.wav -t raw - | ./m17-mod -S AB1CD -x > m17.bin

This bitstream file can be fed into [m17-gnuradio](https://github.com/mobilinkd/m17-gnuradio) to
transmit M17 using a PlutoSDR, HackRF, or any SDR with an appropriate GNURadio sink. See the
specification document for more details on [file formats](https://spec.m17project.org/appendix/file-formats).

### Command line options

    -h for general help

#### m17-mod

    -S for the source callsign (alphanumeric string),
    -D for the destination callsign (alphanumeric string),
    -C for the Channel Access Number (0..15, default - 10),
    -x for binary output (M17 baseband as a packed bitstream),
    -r for raw audio output (single channel, signed 16-bit little endian, +7168 for the `+1` symbol),
    -s for symbols output,
    -d to dump additional debug info,
    -K for AES key setting (hex format without the leading `0x`) - automatically checks keylength,
    -T for PTT on command,
    -O for PTT off command,
    
#### m17-demod

    -l to enable LICH data dump,
    -d to dump additional debug info,
    -x for binary output,
    -r for raw audio output,
    -s for symbols output (signed 8-bit, 4.8k per second),
    -K for AES key, automatic mute if encrypted transmission and no key provided.

The demodulator produces diagnostic output which looks like:

    SRC: BROADCAST, DEST: AB3CD, TYPE: 0002, NONCE: 0000000000000000000000000000, CRC: bb9b
    dcd: 1, evm:    13.27%, deviation:   0.9857, freq offset:  0.03534, locked:   true, clock:        1, sample: 0, 0, 0, cost: 9

The first line shows the received link information.  The second line contains the following diagnostics.

 - **DCD** -- data carrier detect -- uses a DFT to detect baseband energy.  Very good at detecting whether data may be there.
 - **EVM** -- error vector magnitude -- measure the Std Dev of the offset from ideal for each symbol.
 - **Deviation** -- normalized to 1.0 to mean 2400Hz deviation for the input from the following command:
    `rtl_fm -F 9 -f 439.5M -s 18k | sox -t s16 -r 18k -c1 - -t raw - gain 9 rate -v -s 48k` -- the rates and gain are the important part.
 - **Frequency Offset** -- the estimated frequency offset from 0 based on the DC level of each symbol.  The magnutude has
    not been measure but should be around 1.0 == 2kHz using the same normalized input as for deviation.  Anything > 0.1
    or less than -0.1 requires review/calibration of the TX and RX frequencies.
 - **Locked** -- sync word detected. 
 - **Clock** -- estimated difference between TX and RX clock.  Will not work if > 500ppm.  Normalized to 1.0 -- meaning the clocks are equal.
 - **Sample** -- estimated sample point based on 2 clock recovery methods.  Should agree to within 1 always.  There are
    10 samples per symbol.  Should never jump by more than 1 per frame.  The third number is the "winner" based on
    certain heuristics.
 - **Cost** -- the normalized Viterbi cost estimate for decoding the frame.  < 5 great, < 15 good, < 30 OK, < 50 bad, > 80 you're hosed.

#### m17-gateway-link_mod:
    -A for M17Gateway address (optional)
    -P for M17Gateway port (default: 17011)
    -d to dump additional debug info,
    -b for binary output (M17 baseband as a packed bitstream),
    -T for PTT on command,
    -O for PTT off command

#### m17-gateway-link_demod:
    -A for M17Gateway address (optional),
    -P for M17Gateway port (default: 17011),
    -d to dump additional debug info,
    -l to enable LICH data dump

## Thanks

This code is based on previous work done by: Rob Riggs WX9O, Jay Francis KA1PQK, David Cherkus N1AI, and Paulo Duarte PU4THZ.

