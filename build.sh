#!/bin/zsh
cd gb-test && make && cd ..
dd if=hello-world.gb of=hello-world-trunc.gb count=8
bin2c hello-world-trunc.gb source/payload.c gbc_payload
#bin2c gb-test/data/font_tmp.bin source/gbc_vram.c gbc_vram
make
./upload.sh