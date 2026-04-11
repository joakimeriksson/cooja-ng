#!/bin/bash
#
# Debug border-router-sky: run csim + tunslip6, then ping manually
#
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
CSIM_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
CONTIKI_DIR="${CONTIKI_DIR:-$CSIM_DIR/../contiki-ng}"
TUNSLIP6="$CONTIKI_DIR/tools/serial-io/tunslip6"

# Generate config
python3 "$SCRIPT_DIR/csc2json.py" \
    "$CONTIKI_DIR/tests/17-tun-rpl-br/03-border-router-sky.csc" \
    --contiki "$CONTIKI_DIR" --firmware-dir "$CSIM_DIR/firmware/cooja" \
    --js-native -o /tmp/br03-debug.json

# Remove command, long timeout
python3 -c "
import json
d=json.load(open('/tmp/br03-debug.json'))
d['serial_socket']['command']=''
d['timeout_ms']=300000
json.dump(d,open('/tmp/br03-debug.json','w'))
"

# Kill stale processes
fuser -k 60001/tcp 2>/dev/null || true
sleep 1

echo "=== Starting csim (background, logging to /tmp/br03-trace.log) ==="
"$CSIM_DIR/build/test_runner" mixed-multinode /tmp/br03-debug.json -v \
    > /tmp/br03-trace.log 2>&1 &
CSIM_PID=$!
echo "  csim PID: $CSIM_PID"

# Wait for serial socket
sleep 2

echo "=== Starting tunslip6 ==="
sudo "$TUNSLIP6" -a 127.0.0.1 fd00::1/64 &
SLIP_PID=$!
echo "  tunslip6 PID: $SLIP_PID"

echo ""
echo "=== Waiting 90s for RPL convergence ==="
for i in $(seq 9 -1 0); do
    sleep 10
    echo "  ${i}0s remaining... (RF frames so far: $(grep -c 'TX frame' /tmp/br03-trace.log 2>/dev/null || echo 0))"
done

echo ""
echo "=== Pinging Node 2 (1 hop): fd00::0212:7402:0002:0202 ==="
ping6 fd00::0212:7402:0002:0202 -c 3 -i 1 -W 2
echo ""
echo "=== Pinging Node 3 (2 hops): fd00::0212:7403:0003:0303 ==="
ping6 fd00::0212:7403:0003:0303 -c 3 -i 1 -W 2
echo ""
echo "=== Pinging Node 4 (3 hops): fd00::0212:7404:0004:0404 ==="
ping6 fd00::0212:7404:0004:0404 -c 3 -i 1 -W 2
PING_STATUS=$?

echo ""
echo "=== Cleanup ==="
sudo kill $SLIP_PID 2>/dev/null
kill $CSIM_PID 2>/dev/null
wait $CSIM_PID 2>/dev/null

echo ""
echo "=== Results ==="
if [ $PING_STATUS -eq 0 ]; then
    echo "PING SUCCESS"
else
    echo "PING FAILED"
fi
echo ""
echo "RF stats:"
grep -a "RF bytes\|CC2420 RX\|auto_ack" /tmp/br03-trace.log
echo ""
echo "Node 4 activity:"
grep -a "Node 4.*TX\|Node 4.*reachable\|Node 4.*Sending" /tmp/br03-trace.log | tail -5
echo ""
echo "Frames to Node 4 (non-DIO):"
grep -a "receivers.*4(" /tmp/br03-trace.log | grep -v "108\|103\|99\|33" | tail -5
echo ""
echo "Full log: /tmp/br03-trace.log"
