#!/bin/bash
set -e

BOOT="$1"
APP="$2"
OUT="$3"

echo "🔍 比對 crypto 區段..."
if cmp -s "$BOOT" "$APP"; then
    touch "$OUT"
else
    echo "❌ Crypto 不一致，請確認 boot 與 app 使用相同版本的 mbedTLS" >&2
    exit 1
fi
