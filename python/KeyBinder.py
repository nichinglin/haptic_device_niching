#!/usr/bin/python
import sys
import argparse
from BluetoothService import *
import curses
from bitstring import BitArray
import signal

stdscr = curses.initscr()
curses.cbreak()
curses.noecho()
stdscr.keypad(1)
stdscr.addstr(0,10,"Hit 'q' to quit\n waiting to connect")
stdscr.refresh()
key = ''
current_motor = 0;
intensities = [7, 7, 7, 7, 7]
frequencies = [3, 3, 3, 3, 3]
bluetooth = True
# a dictionary for bluetooth mac address
parser = argparse.ArgumentParser()
d_addrs ={\
        '1': '20:15:04:30:61:81', \
        '2': '20:15:04:30:59:73', \
        '3': '20:15:03:17:22:91', \
        '4': '20:14:12:15:25:88', \
        '5': '20:15:05:04:26:44', \
        '6': '20:15:09:14:05:84', \
        '7': '20:15:05:05:16:17', \
        '8': '20:15:05:04:19:97', \
        '9': '20:15:04:30:61:95', \
        '10': '20:15:05:04:08:73',\
        '11': '20:15:06:24:58:72',\
        '12': '20:15:07:20:27:64',\
        '13': '20:15:07:27:94:54',\
        '14': '20:15:03:17:28:24',\
        '15': '20:15:07:24:36:99',\
        '05': '98:D3:31:FC:25:57',\
        '51': '98:D3:31:FC:21:2E'\
        }

        #'0':'20:14:04:24:11:95',\
        #'1':'20:13:11:29:04:20',\

show_addrs = ''
for idx in d_addrs:
    show_addrs = show_addrs + '%s: %s\n' % (idx, d_addrs[idx])

parser.add_argument("-a", help=show_addrs)
parser.add_argument("-b", help=show_addrs)

toConnect = '20:13:11:29:04:20' #'20:14:04:24:11:95'

mode = 0
CALIBRATION_MODE = 0
TEST_MODE = 1

def signal_handler(signal, frame):
    curses.nocbreak(); 
    stdscr.keypad(0); 
    curses.echo()
    curses.endwin()
    print('Closing program safely!')
    sys.exit(0)

def update(bt, enable_bluetooth=False):
    global toConnect
    toSend = BitArray()
    for i in range(0,5):
        frequency = BitArray(uint=frequencies[i], length=3)
        intensity = BitArray(uint=intensities[i], length=3)
        state     = BitArray(uint=1, length=3)
        if i != current_motor:
            intensity = BitArray(uint=0, length=3)
        toSend.append(frequency)
        toSend.append(intensity)
        toSend.append(state)
    toSend.append('0b000')
    toSend.append('0x00')
    stdscr.addstr(10,10,str(toSend.bin))
    if enable_bluetooth:
        bt.send(toConnect, str(toSend.tobytes()))

def cleanup():
    for i in range(0,5):
        intensities[i] = 0
    update(bt, enable_bluetooth=bluetooth)
    curses.nocbreak(); 
    stdscr.keypad(0); 
    curses.echo()
    curses.endwin()

if __name__ == '__main__':
    try:
        bt = BluetoothService()
        args = parser.parse_args()
        if args.a:
            toConnect = d_addrs[args.a]
        if args.b:
            bluetooth = False
        signal.signal(signal.SIGINT, signal_handler) 

        #toConnect = '20:14:04:24:11:95'
        #toConnect = '20:13:11:29:04:20'
        
    #    bluetooth = False
        if bluetooth:
            bt.connect(toConnect,enablePrint=False)
        os.system('cls' if os.name == 'nt' else 'clear')
        for i in range(0,5):
            if(i == current_motor):
                stdscr.addstr(i+1,8, "*")
            stdscr.addstr(i+1,10,"[" + str(i) +"] "+ str(intensities[i]) + " " +str(frequencies[i]))
        while key != ord('q'):
            key = stdscr.getch()
            stdscr.erase()
            stdscr.refresh()
            if key==curses.KEY_UP:
                intensities[current_motor] = (intensities[current_motor] + 1) % 8
            if key==curses.KEY_DOWN:
                intensities[current_motor] = (intensities[current_motor] - 1) % 8
            if key==curses.KEY_LEFT:
                current_motor = (current_motor - 1) % 5
            if key==curses.KEY_RIGHT:
                current_motor = (current_motor + 1) % 5
            if key==50:#2
                frequencies[current_motor] = ((frequencies[current_motor] - 2) % 5) + 1
            if key==51:#3
                frequencies[current_motor] = ((frequencies[current_motor]) % 5) + 1
            if key==ord('r'):
                #reset
                for i in range(0,5):
                    intensities[i] = 0
            if key==10:
                mode = (mode + 1) % 2
            
            # update bluetooth and display
            update(bt, enable_bluetooth=bluetooth)
            for i in range(0,5):
                if(i == current_motor):
                    stdscr.addstr(i+1,8, "*")
                stdscr.addstr(i+1,10,"[" + str(i) +"] "+ str(intensities[i]) + " " +str(frequencies[i]))
    finally:
        cleanup()
    cleanup()
    print("All done!")
