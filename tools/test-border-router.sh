#!/bin/bash
#
# Border-router test helper for csim
# Replaces Contiki-NG's test-border-router.sh and test-native-border-router.sh
# to avoid make/gcc dependencies. Runs tunslip6 or border-router.native directly.
#

# Arguments (same as Contiki-NG's test scripts):
CONTIKI=$1
BASENAME=$2
IPADDR=$3
WAIT_TIME=$4
PING_SIZE=${5:-56}
PING_DELAY=${6:-1}
COUNT=5
PREFIX="fd00::1/64"

# Skip sudo if already root (e.g., inside Docker)
if [ "$(id -u)" -eq 0 ]; then
    SUDO=""
else
    SUDO="sudo"
fi

# Detect native-border-router mode from test name
if echo "$BASENAME" | grep -q "native-border-router"; then
    # Native border-router: run border-router.native which handles SLIP+TUN
    BR_NATIVE="$CONTIKI/examples/rpl-border-router/build/native/border-router.native"
    if [ ! -x "$BR_NATIVE" ]; then
        echo "Error: border-router.native not found at $BR_NATIVE"
        echo "Build it: make -C $CONTIKI/examples/rpl-border-router TARGET=native"
        exit 1
    fi
    echo "Starting border-router.native -a localhost $PREFIX"
    $SUDO "$BR_NATIVE" -a localhost "$PREFIX" &
    MPID=$!
else
    # Embedded border-router: run tunslip6 to connect to csim's serial socket
    TUNSLIP6="$CONTIKI/tools/serial-io/tunslip6"
    if [ ! -x "$TUNSLIP6" ]; then
        echo "Error: tunslip6 not found at $TUNSLIP6"
        echo "Build it: make -C $CONTIKI/tools/serial-io tunslip6"
        exit 1
    fi
    echo "Starting tunslip6 -a 127.0.0.1 $PREFIX"
    $SUDO "$TUNSLIP6" -a 127.0.0.1 "$PREFIX" &
    MPID=$!
fi

printf "Waiting for network formation (%d seconds)\n" "$WAIT_TIME"
sleep "$WAIT_TIME"

# Ping the target node
echo "Pinging $IPADDR (count=$COUNT size=$PING_SIZE delay=$PING_DELAY)"
ping6 "$IPADDR" -c "$COUNT" -s "$PING_SIZE" -i "$PING_DELAY"
STATUS=$?

echo "Closing (pid $MPID)"
# Kill the sudo+child process. Must use sudo to kill root processes.
$SUDO kill "$MPID" 2>/dev/null
$SUDO kill -9 "$MPID" 2>/dev/null

if [ $STATUS -eq 0 ]; then
    echo "PING SUCCESS"
    exit 0
else
    echo "PING FAILED (status=$STATUS)"
    exit 1
fi
