#!/bin/sh
# Baut und startet den Host-Test der I2C-Zustandsmaschine.
#   test/i2c_model_test/run.sh
set -e
here=$(cd "$(dirname "$0")" && pwd)
out=$(mktemp -d)
trap 'rm -rf "$out"' EXIT
python3 - "$here/../../src/peripherals.cpp" "$out/model_part.inc" <<'PY'
import sys
src = open(sys.argv[1], encoding='utf-8').read()
a = src.index('void i2c_reset_txn()')
b = src.index('uint8_t i2c_read_byte(uint32_t addr)')
c = src.index('\n}\n', src.index('void i2c_write_byte(uint32_t addr, uint8_t val)')) + 3
open(sys.argv[2], 'w', encoding='utf-8').write(src[a:b] + src[b:c])
PY
g++ -std=c++17 -Wall -Wno-unused-function -I"$out" -o "$out/host_test" "$here/host_test.cpp"
"$out/host_test"
