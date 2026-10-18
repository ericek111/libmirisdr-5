LibMiriSDR-6
============

A library and tools for receivers built on the Mirics MSi2500 and MSi001 chips, such as the SDRplay RSP1, RSP1B, RSPdx-R2 and RSPduo.

This is another flavour of libmirisdr, started from the original libmirisdr-2 by Miroslav Slugeň, with additions by Leif Åsbrink SM5BSZ in libmirisdr-3-bsz and by Edouard Griffiths (f4exb) in libmirisdr-4, and by Erik Bročko  in libmirisdr-5.

This library is an API compatible drop-in replacement for libmirisdr-5, it only adds extra functions providing additional functionality.

<h2>Documentation</h2>

  - [Using the receivers with SoapySDR](doc/soapysdr.md): for SDR++, GQRX, CubicSDR, GNU Radio and other programs. Sample rates, filters, gain, settings and the device string.
  - [The C API](doc/c-api.md): for your own programs. Opening, tuning, streaming, baseband output, time stamps and scanning.


<h2>Building</h2>

    mkdir build
    cd build
    cmake ..
    make
    sudo make install

The SoapySDR module is built too when the SoapySDR development files are installed. To use the receivers without root, copy `mirisdr.rules` to `/etc/udev/rules.d/`.


<h2>What is new in this version</h2>

<h3>For users</h3>

  - **SDRplay RSP1B and RSPdx-R2 support.** Filters, notches, bias-T, the amplifier (in the gain) and pads, and the RSPdx-R2's three antenna inputs and direct sampling below 2 MHz.
  - **SDRplay RSPduo support.** Both tuners, alone or together (a low IF each, delivered as two labelled streams or two SoapySDR channels). Each tuner's filters, notches, amplifier (in the gain) and pads, the bias-T, the Hi-Z input, the tie that feeds both tuners from one port, and the reference output.
  - **A SoapySDR module.** Programs that use SoapySDR can use the receivers with `driver=mirisdr`. With `report=true` it logs what the receiver runs after each change: the rate, the IF, the NCO, direct sampling and more.
  - **Low sample rates.** Any rate down to about 3.5 ksps, through the tuner's low IF.
  - **High sample rates.** Up to 12 Msps with isochronous USB transfers, and about 27.5 Msps with bulk transfers.
  - **Better reception on all MSi001 receivers.**
  - **A stable stream.** The library loads its own firmware into the receiver when it opens it. Together they keep the stream running across rate changes, start and stop, and USB stalls, without resetting the receiver.

<h3>For developers</h3>

  - **One request for the tune and one for the stream.** A combination the receiver cannot do is refused. A value it has no exact step for is taken to the nearest, or refused on request (`strict`). The result says what is in force. The structs have room to grow, so programs keep working with newer versions.
  - **Complex float output** at any IF, with the frequency you set at 0 Hz.
  - **Time stamps** from a GNSS receiver's PPS output, so samples can be tied to real time, for example for TDOA. Without a PPS source the USB start of frame can be used. To use the SOF sync, you need a utility such as [sofcapture](https://github.com/BertoldVdb/sofcapture).
  - **Lost samples** are found reliably, from markers the firmware puts in the stream. Each buffer says where its gaps are, and gaps can be filled with zeros so the timing stays right.
  - **Events** say at which sample a tune or gain change took effect.
  - **Fast tuning and scanning.** A tune is a single USB request. Lists of frequencies can hop in the receiver itself, with a report for every hop. The tuner's calibration can be learned once and replayed.
  - **More stream options:** real modes that capture one converter and halve the USB data, a bypass of the receiver's own decimation filter for rates up to about 30 Msps, and an automatic choice of sample format.
  - **More low-level functionality:** GPIO pins, a serial port, an I2C master, the infrared input, and the EEPROM.
  - **A null device** that runs your own USB data through the library, for tests.

<h3>Tools</h3>

  - `miri_test` checks the receiver and the library for many known problems.
  - `miri_eeprom` reads, writes and backs up the EEPROM. It can also put the firmware there, with a USB id and serial of your choice. See the SoapySDR guide before you do this.
