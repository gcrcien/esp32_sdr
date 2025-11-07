ESP32 Phasing SDR Board
////////////////////////////////////////////////////////////////////////////////
///////This is a work in progress the board and firmware still have bugs////////
////////////////////////////////////////////////////////////////////////////////

Standalone software-defined radio front-end based on an ESP32 and an analog phasing (I/Q) receiver.
The goal of this project is to provide a compact HF SDR/panadapter board with:

Analog RF front-end (ring mixers + handmade transformers)

High-quality 24-bit baseband digitization

On-board audio output and 4" TFT UI

Direct I/Q output for connection to a PC SDR program

The PCB shown here is already fabricated and under test.

Main Features

ESP32-based SDR platform

Runs the demodulation / UI firmware (AM, SSB, CW, etc.).

Communicates with ADC, DAC and TFT over I²S / SPI.

RF front-end

SMA antenna input.

Ring mixers built with BAT01599R Schottky diode quads.

Handmade RF transformers for LO and RF ports.

RF pre-amplifier using a 2N3904.

Baseband section

Active I/Q low-frequency amplification with a TL072 dual op-amp.

I/Q output header for connection to a PC (panadapter / external SDR).

Data conversion

24-bit audio ADC for digitizing baseband 

Audio DAC for speaker/headphone output 

Audio output

On-board audio power stage for speaker or headphones.

Dedicated audio out connector.

User interface

4" TFT display for spectrum / waterfall / UI.

Rotary encoder for tuning and menu navigation.

Multiple push buttons for mode / band / configuration.

Power

USB-C connector for power and data.

On-board PT4056 Li-Ion charger.

Battery connector footprint.

Power switch and status LEDs.

Hardware Overview
RF Section

SMA input connector feeding the RF front-end.

RF pre-amp based on a 2N3904 transistor.

Double-balanced ring mixers using BAT01599R diode quads.

Handmade transformers for:

RF input matching

Local oscillator distribution

I/Q baseband coupling

Baseband & Conversion

TL072 dual op-amp used as baseband (I/Q) amplifier and anti-alias filtering.

I/Q signals:

Routed to the on-board ADC.

Also exposed on a dedicated header to feed an external PC-based SDR.

24-bit audio ADC connected to the ESP32 via I²S.

Audio DAC connected via I²S to drive the audio chain.

Audio & UI

Audio amplifier stage for speaker / headphones.

4" TFT display:

Used to draw FFT / waterfall and UI elements.

Rotary encoder + push buttons for:

Frequency tuning

Band selection

Demodulation mode and settings.

Power Management

PT4056 Li-Ion charger IC.

USB-C connector for charging and powering the board.

Battery connector footprint (single-cell Li-Ion).

Power rails for RF, analog and digital sections with appropriate decoupling.

Firmware

The firmware is written for the ESP32 and includes:

I²S double-buffered capture from the ADC.

I²S audio playback through the DAC.

Demodulation algorithms for AM, SSB and CW using I/Q phasing.

FFT and waterfall display on the 4" TFT.

Encoder and button handling for tuning and configuration.

Optional I/Q streaming to a PC through the I/Q output.

<img width="1203" height="666" alt="image" src="https://github.com/user-attachments/assets/26dbdbbf-3860-410c-b2b7-0fe3ee5c90c9" />![Imagen de WhatsApp 2025-11-06 a las 18 38 44_351f8baf](https://github.com/user-attachments/assets/0af3df99-e6d2-4e07-a1b5-46200d921110)


