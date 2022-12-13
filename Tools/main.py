#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""

CLI script to test uart TX/RX. The script sends a string to UART, 
then checks that the received string and the transmitted string match.
If the received string doesnt match the transmitted string, it's counted as an error.

usage: main.py [-h] [-r REPEAT] [-t TIMEOUT] com_port baudrate string

positional arguments:
  com_port              serial com port name (/dev/USBttyx, COMx)    
  baudrate              serial port baudrate (9600, 115200, etc)     
  string                string to send on serial port

options:
  -h, --help            show this help message and exit
  -r REPEAT, --repeat REPEAT
                        number of times the string is sent on serial port, default: 100
  -t TIMEOUT, --timeout TIMEOUT
                        UART receive timeout in milliseconds

"""

import argparse
import logging as log
import sys
import time
from typing import *

import serial
from alive_progress import alive_it

LOG: log.Logger = log.getLogger(__name__)


def main():

    arg_parser: argparse.ArgumentParser = argparse.ArgumentParser()

    arg_parser.add_argument(
        "com_port",
        help="serial com port name (/dev/USBttyx, COMx)",
        type=str
    )

    arg_parser.add_argument(
        "baudrate",
        help="serial port baudrate (9600, 115200, etc)",
        type=int
    )

    arg_parser.add_argument(
        "string",
        help="string to send on serial port",
        type=str
    )

    arg_parser.add_argument(
        "-r", "--repeat",
        help="number of times the string is sent on serial port, default: 100",
        type=int,
        default=100
    )
    
    arg_parser.add_argument(
        "-t", "--timeout",
        help="UART receive timeout in milliseconds",
        type=float,
        default=-1
    )

    arguments = arg_parser.parse_args()

    LOG.debug(f"{arguments=}")

    com_port: str = arguments.com_port
    baudrate: int = arguments.baudrate
    repetitions: int = arguments.repeat
    byte_string: str = arguments.string
    byte_string = byte_string.replace("\\r", "\r")
    byte_string = byte_string.replace("\\n", "\n")
    byte_string = bytearray(byte_string, "ascii")

    if arguments.timeout == -1:
        serial_timeout: float = 10 * (len(byte_string) / (baudrate / 10))
    else:
        serial_timeout: float = (arguments.timeout / 1000)

    LOG.info(
        f"Sending {len(byte_string)} bytes, for {repetitions} times, "
        f"using serial port {com_port}, baudrate: {baudrate}, rx timeout: {serial_timeout * 1000} milli-seconds."
    )

    serial_port: serial.Serial = serial.Serial(port=com_port, baudrate=baudrate, timeout=serial_timeout)
    
    errors: List[str] = list()
    receive_times: List[float] = list()
    start_time: float = time.time()
    
    for i in alive_it(range(repetitions), title="progress", length=60):

        tx_time: float = time.time()
        serial_port.write(byte_string)
        received_line = serial_port.read(size=len(byte_string))
        rx_time: float = time.time()

        if received_line == byte_string:
            receive_times.append((rx_time - tx_time)*1000)
        else:
            errors.append((i + 1, received_line))

            # LOG.debug("------------------")

    end_time: float = time.time()

    serial_port.close()

    if len(errors) > 0:
        LOG.error(f"There were {len(errors)} errors")
        for err in errors:
            # LOG.error(f"Line #{err[0]}, received: {err[1]}")
            LOG.debug(f"Line #{err[0]}, received: {err[1]}")

    LOG.info(f"Took {end_time - start_time} seconds")
    if receive_times:
        LOG.info(f"RTT avg={sum(receive_times)/len(receive_times)} ms, min={min(receive_times)} ms, max={max(receive_times)} ms")
    LOG.info("Done!")

if __name__ == "__main__":
    log.basicConfig(level=log.INFO)
    main()
