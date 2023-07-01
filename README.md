# JavaScript 6502 Emulator

## Usage

``` shell
$ node emulator.js <binary file>
```

Once the emulator starts, the low byte of `PC` will be loaded
with the byte at `0xFFFC` in memory, and the high byte of `PC`
will be loaded with the byte at `0xFFFD`.

The emulator responds to the following commands:

| Command                   | Description                                                                                                           |
|---------------------------|-----------------------------------------------------------------------------------------------------------------------|
| `d`                       | Dump the last 100 CPU states.                                                                                         |
| `d` **n**<sub>10</sub>    | Dump the last **n**<sub>10</sub> CPU states, upto a maximum of 100 states.                                                         |
| `l`                       | Continue to evaluate instructions until a PC loop is detected.                                                        |
| `l` **n**<sub>16</sub>    | Continue to evaluate instructions until the PC equals **n**<sub>16</sub>.                                             |
| `peek` **n**<sub>16</sub> | View the byte stored at address **n**<sub>16</sub> in memory.                                                         |
| `q`                       | Quit                                                                                                                  |
| `s`                       | Step a single instruction. Print the decoded instruction and the register status after the instruction has evaluated. |
| `s` **n**<sub>10</sub>    | Step **n**<sub>10</sub> times. See `s`.                                                                               |

## TODO
- [ ] 6502 decimal mode. As of now, the decimal flag is ignored by arithmetic instructions.
- [ ] Accounting of cycle times. The emulator does not currently keep track of the number of cycles elapsed.
