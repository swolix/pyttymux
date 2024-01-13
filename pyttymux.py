#!/usr/bin/env python3
# PyTTYMux, based on SerialMux by Al Williams (https://github.com/wd5gnr/SerialMux)
#
# Copyright 2024, Sijmen Woutersen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the “Software”), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
#
#
# Multiplexes multiple (UART) channels over a single one using the following protocol;
#   * 0xFF is used as escape character
#   * 0xFF is escaped as 0xFF 0xFE
#   * Multiple 0xFF should considered as one
#   * 0xFF <1byte_channel_id> selects a channel (0-249)
#   * 0xFF 0xFD request otherside to retransmit channel id

import os, sys
import ctypes
import termios
import select
import argparse
import time
import logging

import serial

class TTYMux:
    def __init__(self, port, resend_channel_timer=10, **kwargs):
        self.ptys = {}
        self.resend_channel_timer = resend_channel_timer
        logging.info("Opening serial port on {}...".format(port))
        self.serial = serial.Serial(timeout=0, port=port, **kwargs)
        self.serial.nonblocking()
        self.libc = ctypes.cdll.LoadLibrary("libc.so.6")
        self.libc.ptsname.restype = ctypes.c_char_p
        self.running = False

    def open_channel(self, id, path):
        if self.running:
            raise Exception("Already running")
        elif id in self.ptys:
            raise Exception("Channel already open")
        elif id < 0:
            raise Exception("Channel must be >= 0")
        elif id >= 250:
            raise Exception("Channel must be < 250")

        pty = self.libc.posix_openpt(os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        if pty < 0: raise Exception("Could not open pty")
        self.libc.grantpt(pty)
        self.libc.unlockpt(pty)

        logging.info("Opening mux channel {} at {} (fd={})...".format(id, os.path.abspath(path), pty))

        attr = termios.tcgetattr(pty)

        # cfmakeraw
        attr[0] &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK | termios.ISTRIP | 
                     termios.INLCR | termios.IGNCR | termios.ICRNL | termios.IXON)
        attr[1] &= ~termios.OPOST
        attr[3] &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)
        attr[2] &= ~(termios.CSIZE | termios.PARENB)
        attr[2] |= termios.CS8

        attr[2] &= ~termios.CRTSCTS
        attr[2] |= (termios.CLOCAL | termios.CREAD)
        attr[2] &= ~termios.CSIZE
        attr[1] &= ~termios.OPOST
        attr[6][termios.VTIME] = 0
        attr[6][termios.VMIN] = 0

        termios.tcsetattr(pty, termios.TCSANOW, attr)

        # open slave device to avoid problems when the mux-client closes the mux fd
        ptsname = self.libc.ptsname(pty).decode("utf-8")
        sfd = open(ptsname)

        # setup link
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        os.symlink(ptsname, path)

        self.ptys[id] = (pty, path, sfd)

    def __del__(self):
        for id, (pty, link, sfd) in self.ptys.items():
            os.unlink(link)

    def run(self):
        self.running = True
        channel_update_time = 0
        unknown_channels = [False]*256
        blocking_channels = {}

        channels = {}
        poll = select.poll()
        for id, (pty, link, fd) in self.ptys.items():
            poll.register(pty, select.POLLIN)
            channels[pty] = id
        active_tx_channel = active_rx_pty = None
        rx_escaped = False
        poll.register(self.serial.fileno(), select.POLLIN)

        logging.info("Mux started")

        while self.running:
            for pty, events in poll.poll(1000):
                if (events & ~select.EPOLLIN) != 0:
                    raise Exception("Unkown event {:x} from fd {}".format(events, pty))
                if pty == self.serial.fileno():
                    for c in self.serial.read(128):
                        r = None
                        if c == 0xff:
                            rx_escaped = True
                        elif rx_escaped:
                            if c != 0xFF:
                                if c == 0xFE:
                                    r = b"\xff"
                                elif c == 0xFD:
                                    if not active_tx_channel is None:
                                        self.serial.write(bytes([0xff, active_tx_channel]))
                                        channel_update_time = time.time()
                                else:
                                    try:
                                        active_rx_pty = self.ptys[c][0]
                                    except KeyError:
                                        if not unknown_channels[c]:
                                            logging.error("Received data for unknown channel {}".format(c))
                                            unknown_channels[c] = True
                                        active_rx_pty = None
                                rx_escaped = False
                        elif not active_rx_pty is None:
                            r = bytes([c])
                        if not r is None and not active_rx_pty is None:
                            try:
                                os.write(active_rx_pty, r)
                            except BlockingIOError:
                                if not blocking_channels.get(active_rx_pty, False):
                                    logging.error("Dropping data for pty {} (blocked)".format(active_rx_pty))
                                    blocking_channels[active_rx_pty] = True
                            else:
                                blocking_channels[active_rx_pty] = False
                else:
                    id = channels[pty]
                    data = os.read(pty, 128)
                    if id != active_tx_channel:
                        self.serial.write(bytes([0xff, id]))
                        active_tx_channel = id
                        channel_update_time = time.time()
                    for c in data:
                        if c == 0xFF:
                            self.serial.write(b"\xff\xfe")
                        else:
                            self.serial.write(bytes([c]))
            if (not active_tx_channel is None and self.resend_channel_timer > 0 and
                time.time() - channel_update_time > self.resend_channel_timer):
                self.serial.write(bytes([0xff, active_tx_channel]))
                channel_update_time = time.time()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("port", help="Serial port")
    parser.add_argument("-b", "--baudrate", default=115200, help="Serial port baudrate")
    parser.add_argument("-p", "--ports", metavar="MUXPORT", action='append', help="Add mux port in channel_id:pty_path format")
    parser.add_argument("-v", "--verbose", dest="level", action='store_const', const=logging.INFO, default=logging.WARN, help="Verbose logging")
    parser.add_argument("-d", "--debug", dest="level", action='store_const', const=logging.DEBUG, help="Debug logging")

    args = parser.parse_args()

    logging.getLogger().setLevel(args.level)

    if args.ports is None:
        sys.stderr.write("At least one mux port is required\n")
        parser.print_usage()
        sys.exit(-1)

    ttymux = TTYMux(port=args.port, baudrate=args.baudrate)
    for port in args.ports:
        try:
            channel, path = port.split(":", 2)
            ttymux.open_channel(int(channel), path)
        except ValueError:
            sys.stderr.write("Mux port '{}' must be in channel_id:pty_path form\n".format(port))
            parser.print_usage()
            sys.exit(-1)

    ttymux.run()