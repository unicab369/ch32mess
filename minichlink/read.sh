#!/usr/bin/env bash

SYMBOL=$1
if [ -z "$SYMBOL" ]; then
   echo "Usage: $0 <symbol>"
   exit 1
fi

echo "Searching for symbol: '$SYMBOL'"

ADDR="0x$(riscv64-unknown-elf-objdump -t *.elf | rg $SYMBOL | awk '{print $1}')"

if [ -z "$ADDR" ]; then
   echo "Error: Symbol '$SYMBOL' not found in the object file."
   exit 1
fi

if ! [[ $ADDR =~ ^0x[0-9a-fA-F]+$ ]]; then
    echo "Error: Address must be in hexadecimal format (e.g., 0x20000000)."
    exit 1
fi

echo "Reading '$SYMBOL' at address: $ADDR"

commands=" -s 0x10 0x80000001" # Make the debug module work properly. 
commands+=" -s 0x20 0x0002a303" # Write wcode of lw x6,0(x5)
commands+=" -s 0x21 0x00100073" # Write wcode of ebreak
commands+=" -s 0x04 $ADDR" # Write address 0x20000000 to DATA0
commands+=" -s 0x17 0x00271005" # Abstract cmd data0->x5 and exec progbuf
commands+=" -m 0x16" # Read ABSTRACTCS
commands+=" -s 0x17 0x00221006" # Abstract cmd x6->data0
commands+=" -m 0x16" # Read ABSTRACTCS
commands+=" -m 0x04" # Read DATA0
commands+=" -s 0x10 0x40000001" # Resume(1<<30) without reset(1<<0)

echo $commands | nc localhost 4444
