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
| `CH32V002` | 48  | 16K  | 2K  | 18 | 1/1 | 1 | 1  | 1/6 | -   | - | - | 1 | 1 | 1 | - | -  | -   | -   | -  | 2.7-5.5 | TSSOP20 | SWD, WFE mode |
| `CH32V003` | 48  | 16K  | 2K  | 18 | 1/1 | 1 | 1  | 1/6  | -   | - | - | 1 | 1 | 1 | - | -  | -      | -   | -  | 2.7-5.5 | TSSOP20 |  power
| `CH32V004` | 48  | 32K  | 4K  | 26 | 1/1 | 1 | 1  | 1/6  | -   | - | - | 1 | 1 | 2 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP32  |B 2.0 
| `CH32V005` | 48  | 32K  | 4K  | 26 | 1/1 | 1 | 1  | 1/6  | -   | - | - | 1 | 1 | 2 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP32  |B PD  
| `CH32V006` | 48  | 64K  | 8K  | 42 | 1/1 | 1 | 1  | 1/6  | -   | - | - | 1 | 1 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |  contr
| `CH32V007` | 48  | 64K  | 8K  | 42 | 1/1 | 1 | 1  | 1/6  | -   | - | - | 1 | 1 | 3 | - | HS | -      | -   | -  | 2.7-5.5 | LQFP48  |B 2.0 
| `CH32X033` | 144 | 64K  | 20K | 42 | 2/1 | 1 | 1  | 2/10 | Yes | - | 2 | 1 | 1 | 4 | 1 | FS | -      | -   | 1  | 2.7-5.5 | LQFP48  |P, AES
| `CH32X035` | 144 | 128K | 32K | 52 | 2/1 | 1 | 1  | 2/10 | Yes | - | 2 | 1 | 1 | 5 | 2 | HS | 10/100 | -   | 1  | 2.7-5.5 | LQFP64  |hernet
| `CH32V103` | 72  | 64K  | 20K | 42 | 1/3 | 1 | 1  | 2/10 | -   | - | - | 2 | 1 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |rtex-M
| `CH32V203` | 144 | 64K  | 20K | 42 | 1/3 | 1 | 1  | 2/10 | -   | 2 | - | 2 | 2 | 3 | - | FS | -      | -   | -  | 2.7-5.5 | LQFP48  |SC-V V
| `CH32V208` | 144 | 128K | 64K | 52 | 1/3 | 1 | 1  | 2/10 | -   | 2 | - | 2 | 2 | 4 | 2 | HS | 10/100 | -   | 1  | 2.7-5.5 | LQFP64  |al USB
| `CH32V303` | 144 | 128K | 32K | 52 | 1/3 | 1 | 1  | 2/16 | Yes | 2 | 2 | 2 | 2 | 5 | 2 | HS | -      | -   | 1  | 2.7-5.5 | LQFP64  |U, Cry
| `CH32V305` | 144 | 256K | 64K | 80 | 4/4 | 1 | 1  | 2/16 | Yes | 2 | 2 | 2 | 2 | 8 | 2 | HS | 10/100 | -   | 2  | 2.7-5.5 | LQFP100 |al CAN
| `CH32V307` | 144 | 256K | 64K | 80 | 4/4 | 1 | 1  | 2/16 | Yes | 2 | 2 | 2 | 2 | 8 | 2 | HS | 10/100 | BLE5 | 2  | 2.7-5.5 | LQFP100 | WiFi optional |


CH32V002J4M6: $7.04/50
CH32V002F4P6: $8.29/50      

CH32V003J4M6: $6.34/50          ~ 0.16/each
CH32V003F4U6: $12.80/50
CH32V003F4P6: $11.13/50
CH32V003A4M6: $8.45/50

CH32V006K8U6: $9.00/50
CH32V006F8P6: $7.25/50

CH32X033F8P6: $4.35/20          ~0.22/each

CH32X035F8U6: $6.01/20          ~0.30/each
CH32X035G8U6: $6.57/20
CH32X035C8T6: $15.16/20
CH32X035F8U6: LCSC 30+          $0.26/each

CH32V103C8T6: $34.00/50         ~0.60/each
CH32V103C8T6: $60.94/100

CH32V203C8T6: $5.06/10          ~0.50/each
CH32V203F6P6: LCSC 72+          $0.38/each

CH32V208CBU6: LCSC 100+         $0.89/each

CH32V303CBT6: $28.04/50
CH32V303CBT6: $53.88/100

CH32V305RBT6: $25.71/20
CH32V305RBT6: $57.62/50         ~$1.15/each

CH591R: $13.00/20               ~$0.65/each
CH591F: $14.00/20
CH591D: LCSC 100+               $0.36/each

CH592F: $14.19/20               ~$0.71/each
CH592X: $14.92/20
CH592X: LCSC 100+               $0.55/each

CH583M QFN48: $24.79/20
CH583M QFN48: $52.38/50         ~$1.04/each
CH32M: LCSC 100+                $0.98/each

CH573 QFN: $7.38/10             ~0.73/each
CH573F: LCSC 100+               $0.61/each


Freq, Flash SRAM GPIO TIMER, WDOG, RTC, ADC, Touchkey DAC, OPA, SPI, I2C, UART, CAN USB, Ethernet, BLE, SDIO, VDD, Package, and other features

Discord:
```init
```bash
```css
```diff
```fix
|| Spoiler Text ||
> blockquote