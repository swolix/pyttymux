#!/usr/bin/env python3

# assumes pyttymux is running with a loopback on serial, and three mux channels, 'a', 'b', and 'c'

import sys, os
import time

import serial

if __name__ == "__main__":
    TIMEOUT=0.1

    print("Opening ports")
    a = serial.Serial(port='a', timeout=TIMEOUT)
    b = serial.Serial(port='b', timeout=TIMEOUT)
    c = serial.Serial(port='c', timeout=TIMEOUT)

    print("Basic communication")
    a.write(b"a")
    assert a.read() == b"a"
    assert b.read() == b""
    assert c.read() == b""

    b.write(b"b")
    assert a.read() == b""
    assert b.read() == b"b"
    assert c.read() == b""

    c.write(b"c")
    assert a.read() == b""
    assert b.read() == b""
    assert c.read() == b"c"

    print("Interleaved communication")
    a.write(b"AAA")
    b.write(b"BBB")
    c.write(b"CCC")
    assert c.read(10) == b"CCC"
    assert b.read(10) == b"BBB"
    time.sleep(5)
    assert a.read(10) == b"AAA"

    print("Escaped data")
    a.write(b"ff\xffff")
    assert a.read(100) == b"ff\xffff"

    print("Large buffer")
    a.timeout = b.timeout = c.timeout = 10
    a.write(os.urandom(1024))
    b.write(b"xyz")
    assert b.read(3) == b"xyz"
    assert len(a.read(2048)) == 1024

    print("Blocking port")
    for i in range(50):
        print("    {}/{}".format(i+1, 50))
        a.write(os.urandom(1024))
        b.write(os.urandom(4))
        assert len(b.read(4)) == 4

    print("Single channel throughput test")
    start_time = time.time()
    c.write(os.urandom(256))
    for i in range(400):
        c.write(os.urandom(256))
        x = c.read(256)
        assert len(x) == 256
    x = c.read(256)
    assert len(x) == 256
    duration = time.time() - start_time
    print("{} bytes/sec".format(401*256 / duration))

    print("Done")
