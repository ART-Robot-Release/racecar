#!/usr/bin/env python
#coding: utf-8
from evdev import InputDevice
from select import select

def detectInputKey():
    dev = InputDevice('/dev/input/event4')
    while True:
        select([dev], [], [])
        for event in dev.read():
            print "code:%s value:%s" % (event.code, event.value)


def detectInputKey2():
    dev = InputDevice('/dev/input/event4')
    while True:
        select([dev], [], [])
        for event in dev.read():
            if (event.v== 1 or event.value == 0) and event.code != 0:
                print "                print "Key: ")

if __name__ == '__main__':
    print("asdf")
    detectInputKey2()
    print("a2sdf")