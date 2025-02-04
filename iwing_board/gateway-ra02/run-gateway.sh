#!/bin/sh

WORKING_DIR=`dirname $0`

cd ${WORKING_DIR}

. ./ve

while true; do
    python -u gateway.py /home/cattrack/logs/gateway.log
    sleep 1
done
