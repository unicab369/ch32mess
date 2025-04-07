# A Useless Button
Example of a useless button. It doesn't do anything good except being great at what it does.
It demonstrates the power of repetition: if you repeat the samething many times over, it becomes you.

## how to use
If the user press the button once, they affirm what is displayed. 
Pressing it twice will double the affirmation, and the alignment with the affirmed content intensifies with each subsequent press.

## How it works 
The program employs a state machine to monitor button states, facilitating the detection of single or double button presses—a kind of software debouncer. When a button is pressed and released, it doesn't simply transition states twice; instead, it fluctuates multiple times due to the mechanical vibrations. It's hard to see unless you have a 3rd eye. The software debouncer incorporates a brief duration during which button readings are disregarded, allowing the button sufficient time to stabilize and the vibrations to settle before resuming the assessment of button states.

## circuit
Wire a button to PC0 and GND
wire a LED to PD0 and GND


### WCH Microcontroller Comparison

| Feature      | CH32V003    | CH32V203    | CH32V303     | CH32X035     | CH32F203     |
|--------------|-------------|-------------|--------------|--------------|--------------|
| **Core**     | RISC-V (V2) | RISC-V (V4) | RISC-V (V4A) | RISC-V (V4C) | ARM Cortex-M3 |
| **Max Frequency** | 48 MHz           | 144 MHz          | 144 MHz          | 144 MHz          | 72 MHz           |
| **Flash**/**SRAM** | 16 KB / 2KB     | 64 KB / 20 BK    | 256 KB / 64 KB   | 128 KB / 32 KB   | 64 KB / 20 KB    |
| **GPIOs**         | 18               | 42               | 80               | 50               | 37               |
| **ADC**           | 1x 12-bit (10ch) | 2x 12-bit (16ch) | 3x 12-bit (16ch) | 2x 12-bit (16ch) | 2x 12-bit (16ch) |
| **DAC**           | -                | 2x 12-bit        | 2x 12-bit        | 2x 12-bit        | 2x 12-bit        |
| **Timers**        | 1x 16-bit        | 4x 16-bit        | 10x 16/32-bit    | 6x 16-bit        | 4x 16-bit        |
| **DMA Channels**  | 5                | 12               | 12               | 10               | 12               |
| **USB**           | -                | USB FS 2.0       | USB FS/HS 2.0    | USB FS 2.0       | USB FS 2.0       |
| **CAN**           | -                | 1x CAN 2.0B      | 2x CAN 2.0B      | 1x CAN 2.0B      | 1x CAN 2.0B      |
| **I2C/SPI/UART**  | 1/1/2            | 2/2/3            | 3/3/5            | 2/2/4            | 2/2/3            |
| **Voltage Range** | 2.7-5.5V         | 2.7-5.5V         | 2.7-5.5V         | 2.7-5.5V         | 2.7-5.5V         |
| **Packages**      | TSSOP-20         | LQFP48, QFN32    | LQFP100, QFN68   | LQFP64, QFN48    | LQFP48, QFN32    |
| **Best For**      | Basic GPIO/LEDs  | USB devices      | High-performance | Mixed-signal     | ARM migration    |


Discord:
```init
```bash
```css
```diff
```fix
|| Spoiler Text ||
> blockquote