#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import argparse
from typing import *
from collections import namedtuple


class STM32F103CBTCx_Uart(object):
    
    def __init__(self, baudrate: int, clock: int) -> None:
        self.clock: int = clock * (1e6)
        self.baudrate: int = baudrate
        self.brr: Dict[str, int] = self.calculate_uart_brr(baudrate, clock)
        self.calculated_baudrate: float = clock / (16 * (self.brr["mantissa"] + (self.brr["fraction"] / 16)))
        self.error: float = self.calculate_baudrate_error(self.calculated_baudrate, baudrate)
    
    @staticmethod
    def calculate_uart_brr(baudrate: int, clock: int) -> Dict[str, int]:
        """Calculate STM32F103CBTx UART BRR value as a dictionary. 
        UART BRR is calculated using the equation:
            UART_BRR = clock / (16 * baudrate)
            manissa = int(UART_BRR) 
            fraction = round((UART_BRR % 1) * 16) # rounded to nearest integer (up or down)

        Args:
            baudrate (int): UART baudrate
            clock (int): UART clock

        Returns:
            Dict[str, int]: UART BRR as a dictionary, containing mantissa and fraction. If UART BRR value is < 1
        """
        brr: Dict[str, int] = dict(uart_div=0, mantissa=0, fraction=0)
        brr["uart_div"] = clock / (16 * baudrate)
        brr["mantissa"] = int(brr["uart_div"])
        brr["fraction"] = round((brr["uart_div"] % 1) * 16)
        return brr

    @staticmethod
    def calculate_baudrate_error(calculated: float, actual: float) -> float:
        """"""
        return abs(calculated - actual) / actual


def main():
    """
    """
    arg_parser: arg_parser.ArgumentParser = argparse.ArgumentParser(
        description="Calculate STM32F103CBTx UART->BRR value, actual UART baudrate and baudrate error, "
        "for a given baudrate and UART bus clock",
    )
    
    arg_parser.add_argument(
        "baudrate",
        help="UART baudrate",
        type=int
    )
    
    arg_parser.add_argument(
        "--pclk",
        help="UART bus clock in MHz (default: 8)",
        type=int,
        default=8,
    )
    
    args: argparse.Namespace = arg_parser.parse_args()
    
    clock: int      = args.pclk * (1e6)
    baudrate: int   = args.baudrate
    
    uart = STM32F103CBTCx_Uart(baudrate, clock)
    
    if uart.brr["uart_div"] < 1:
        print(f"Baudrate {baudrate} is too high for clock {clock}, can't calculate UART->BRR")
        return
    
    print(f"For UART clock {clock} and baudrate {baudrate}")
    print(f"BRR register value: {uart.brr['uart_div']}")
    print(f"Actual baudrate: {uart.calculated_baudrate}")
    print(f"Baudrate error: {round((uart.error * 100), 2)} %")


if __name__ == "__main__":
    main()
