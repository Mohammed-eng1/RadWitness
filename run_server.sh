#!/usr/bin/env bash
# run_server.sh — تشغيل سيرفر RMS Rover v2 على الراسبري
# يفعّل البيئة الافتراضية ويشغّل السيرفر على المنفذ 8000.
# افتح بعده:  http://therover:8000  (محلياً أو عبر Tailscale)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# shellcheck disable=SC1091
source venv/bin/activate
exec python -m pi.web.server
