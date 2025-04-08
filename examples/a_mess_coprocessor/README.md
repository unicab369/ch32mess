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

| Model | Freq | Flash | SRAM | GPIO | TIM | WDOG | RTC | ADC | Tchkey | DAC | OPA | SPI | I2C | UART | CAN | USB | Ethnet | BLE | SDIO | VDD (V) | Package | Other Features|
|----|----|----|----|----|----|----|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| `CH32V002` | 48  | 16K  | 2K  | 18 | 5  | 1 | 1  | 1x10b | -   | - | - | 1 | 1 | 1 | - | -  | -      | -   | -  | 2.7-5.5 | TSSOP20 | SWD, WFE mode |
| `CH32V003` | 48  | 16K  | 2K  | 18 | 5  | 1 | 1  | 1x10b | -   | - | - | 1 | 1 | 1 | - | -  | -      | -   | -  | 2.7-5.5 | TSSOP20 |  power
| `CH32V004` | 48  | 32K  | 4K  | 26 | 6  | 1 | 1  | 1x12b | -   | - | - | 2 | 1 | 2 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP32  |B 2.0 
| `CH32V005` | 48  | 32K  | 4K  | 26 | 6  | 1 | 1  | 1x12b | -   | - | - | 2 | 1 | 2 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP32  |B PD  
| `CH32V006` | 48  | 64K  | 8K  | 42 | 7  | 1 | 1  | 2x12b | -   | 1 | - | 3 | 2 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |  contr
| `CH32V007` | 48  | 64K  | 8K  | 42 | 7  | 1 | 1  | 2x12b | -   | 1 | - | 3 | 2 | 3 | - | HS | -      | -   | -  | 2.7-5.5 | LQFP48  |B 2.0 
| `CH32X033` | 144 | 64K  | 20K | 42 | 10 | 1 | 1  | 2x12b | Yes | 2 | 2 | 4 | 2 | 4 | 1 | FS | -      | -   | 1  | 2.7-5.5 | LQFP48  |P, AES
| `CH32X035` | 144 | 128K | 32K | 52 | 12 | 1 | 1  | 3x12b | Yes | 2 | 2 | 4 | 3 | 5 | 2 | HS | 10/100 | -   | 1  | 2.7-5.5 | LQFP64  |hernet
| `CH32V103` | 72  | 64K  | 20K | 42 | 7  | 1 | 1  | 2x12b | -   | 1 | - | 2 | 2 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |rtex-M
| `CH32V203` | 144 | 64K  | 20K | 42 | 7  | 1 | 1  | 2x12b | -   | 1 | - | 2 | 2 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |SC-V V
| `CH32V208` | 144 | 128K | 64K | 52 | 10 | 1 | 1  | 2x12b | -   | 2 | - | 3 | 2 | 4 | 2 | HS | 10/100 | -   | 1  | 2.7-5.5 | LQFP64  |al USB
| `CH32V303` | 144 | 128K | 32K | 52 | 10 | 1 | 1  | 3x12b | Yes | 2 | 2 | 4 | 3 | 5 | 2 | HS | -      | -   | 1  | 2.7-5.5 | LQFP64  |U, Cry
| `CH32V305` | 144 | 256K | 64K | 80 | 16 | 1 | 1  | 3x12b | Yes | 2 | 2 | 6 | 4 | 8 | 2 | HS | 10/100 | -   | 2  | 2.7-5.5 | LQFP100 |al CAN
| `CH32V307` | 144 | 256K | 64K | 80 | 16 | 1 | 1  | 3x12b | Yes | 2 | 2 | 6 | 4 | 8 | 2 | HS | 10/100 | BLE5 | 2  | 2.7-5.5 | LQFP100 | WiFi optional |


Freq, Flash SRAM GPIO TIMER, WDOG, RTC, ADC, Touchkey DAC, OPA, SPI, I2C, UART, CAN USB, Ethernet, BLE, SDIO, VDD, Package, and other features

Discord:
```init
```bash
```css
```diff
```fix
|| Spoiler Text ||
> blockquote