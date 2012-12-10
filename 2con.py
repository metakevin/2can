#******************************************************************************
# File:              2con.pl
# Author:            Kevin Day
# Date:              December, 2011
# Description:       
#                    2CAN bus analyzer debug console / web server
#                    
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.
#
# Copyright (c) 2011 Kevin Day
# All rights reserved.
#******************************************************************************


import d2xx
import serial
from time import *
import os
import sys
import time
import traceback
import random
import exceptions
import traceback
from struct import *
import array
import threading
from Queue import Queue,Empty
import json
import pickle

# some constants from the C code
TASK_ID_COMMS          = 0xC
TASK_ID_CAN            = 8
TASK_ID_MCP            = 4
MCP_MSG_READ           = 4
MCP_MSG_WRITE          = 5
COMMS_MSG_ECHO_REQUEST = 1
COMMS_MSG_ECHO_REPLY   = 2
COMMS_MSG_STATS        = 3
COMMS_MSG_SET_BAUD     = 4
COMMS_MSG_NAK          = 0xA
COMMS_MSG_BADTASK      = 0xE
COMMS_MSG_CAN_RAW      = 1
CAN_SEND_RAW_MSG       = 2
CAN_STATS              = 4
CAN_SET_BT = 0xB
CAN_SET_ACCEPT_FILTER = 0xA
CAN_SET_SID_FILTER = 0xC

def sendbytes(bytes):
#    print "Sending " + str([hex(x) for x in bytes])
    #usb.write(array.array('B', bytes).tostring())
    ser.write(array.array('B', bytes).tostring())

tofile = None

msg = []
def send(task, code, payload=[]):
    flags = 0    
    sender = 0
    destination = 0
    plen = len(payload)
    if plen > 15:
        print "Payload too long (%d)" % plen
        raise BaseException
    if task > 15:
        print "Invalid task (%d)" % task
        raise BaseException
    if code > 15:
        print "Invalid code (%d)" % code
        raise BaseException
    # note: gcc treats AVR as little endian, so 'flags' are the 4 lower bits
    m = [((plen<<4)|flags), ((sender<<4)|destination), ((task<<4)|code)] + payload
    
    csum = 0
    for b in m:
        csum += b 
    checksum = escape([(csum>>8)&0xFF, csum&0xFF])

    me = escape(m)

    msg = [0x7E] + me + checksum

    sendbytes(msg)

def escape(m):
    me = []
    for b in m:
        if b == 0x7D or b == 0x7E:
            me.append(0x7D)
            me.append(b^0x20)
        else:
            me.append(b)            
    return me    

class TimeoutException(Exception):
    def __init__(self,value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class NakException(Exception):
    def __init__(self,value):
        self.value = value
    def __str__(self):
        return repr(self.value)

# returns one integer representing one byte
def usbread(l):
    tries = 0
    maxtries = 5
    while tries < maxtries:
        s = ser.read(1)
        if len(s) == 1:
            break
        tries = tries + 1
    if tries == maxtries:
        raise TimeoutException(tries)
    #print "usb read returns",s
    bs = [ord(x) for x in s]
#    hs = [hex(x) for x in bs]
#    print "Received",hs
    return bs

def rd():
    bs = usbread(256)
    print [hex(x) for x in bs]

def usbread_esc(n):
    bv = []
    for i in range(0,n):
        b = usbread(1)[0]
        if b == 0x7D:
            b = usbread(1)[0] ^ 0x20
        bv.append(b)
    return bv

def rcv_direct():
    while 1:
        b = usbread_esc(1)
        if b[0] == 0x7E:
            (m1, m2, m3) = usbread_esc(3)
            csum = m1 + m2 + m3;
            plen = (m1 >>4)
            if (m1&1):
                # big backet: plen is the number of octets of message size that
                # preceed the actual payload
                if plen == 1:
                    plen = usbread_esc(1)[0]
                    csum += plen
#                    print "Big packet payload length is %d" % plen
                else:
                    print "FIXME: multi-octet big packet RX not supported (got %d)" % plen
                    raise BaseException                
            payload = usbread_esc(plen)
            fromaddr = (m2&0xF)
            task = (m3>>4)
            code = (m3&0xF)
            csum_v = usbread_esc(2)
            csum_wire = ((csum_v[0]<<8) + csum_v[1])
            for b in payload:
                csum = csum + b
            if csum != csum_wire:
                print "Checksum error: calculated %X got %X" % (csum, csum_wire)
            else:
                return [task, code] + payload
        else:
            print "Junk character:",hex(b[0])

def rcvmsg():
    while 1:
        try:
            m = rcv()
            parse(m)
            return
        except KeyboardInterrupt:
            print "Abort"
            return    
        except TimeoutException as (attempts):
            print "Timed out after %s attempts" % str(attempts)
            return
        except NakException:
            print "Retransmitting"
            sendbytes(msg)
            
def as_u32(bytes):
    return ((bytes[3]<<24) | (bytes[2]<<16) | (bytes[1]<<8) | (bytes[0]))

def as_u16(bytes):
    return ((bytes[1]<<8) | (bytes[0]))

def checkNak(msg):
    task = msg[0]
    code = msg[1]
    if task == TASK_ID_COMMS and code == COMMS_MSG_NAK:
        return 1
    return 0

def parse(msg):
    task = msg[0]
    code = msg[1]
    payload = msg[2:]

    if checkNak(msg):
        raise NakException(0)

    print "Message task/code %X/%x %d bytes" % (task, code, len(payload))

    if task == TASK_ID_COMMS and code == COMMS_MSG_STATS:
        print "Packets RX:          ", as_u32(payload[0:4])
        print "Packets TX:          ", as_u32(payload[4:8])
        print "Phy bytes RX:        ", as_u32(payload[8:12])
        print "Phy bytes TX:        ", as_u32(payload[12:16])
        print "Junk bytes:          ", as_u16(payload[16:18])
        print "Error bytes:         ", as_u16(payload[18:20])
        print "Checksum errors:     ", as_u16(payload[20:22])
        print "Bad code packets:    ", as_u16(payload[22:24])
        print "RX ring overflow:    ", as_u16(payload[24:26])
        print "RX no-buffers:       ", as_u16(payload[26:28])
        print "RX state:            ", payload[28]
        print "In escape:           ", payload[29]
        print "Uptime:              ", as_u32(payload[30:34])
        print "TX no buffers:       ", as_u16(payload[34:36])
        print "Mailbox overflow:    ", as_u16(payload[36:38])
        print "UART RX overrun:     ", as_u16(payload[38:40])
        print "UART RX frame error: ", as_u16(payload[40:42])
        print "UART RX parity error ", as_u16(payload[42:44])
        print "UART RX multi-byte   ", as_u16(payload[44:46])
        print "UART TX drop (ISR)   ", as_u16(payload[46:48])
        print "UART TX ringfull wait", as_u16(payload[48:50])
    	print "Reset cause          ", hex(payload[50]), 
    	rstbits = {'PORF':0x1, 'EXTRF':0x2, 'BORF':0x4, 'WDRF':0x8, 'JTRF':0x10}
        for n,m in rstbits.iteritems():
            if (payload[50]&m):        
                print n,
        print ""

    elif task == TASK_ID_COMMS and code == COMMS_MSG_BADTASK:
        print "Bad task: ", hex(payload[0])
        print "Bad code: ", hex(payload[1])
    elif task == TASK_ID_COMMS and code == COMMS_MSG_ECHO_REPLY:
        print "Echo reply: ", [hex(x) for x in payload]
    elif task == TASK_ID_MCP and code == MCP_MSG_READ:
        print "MCP read:", [hex(x) for x in payload],[bin(x) for x in payload]
    elif task == TASK_ID_MCP and code == COMMS_MSG_CAN_RAW:
        # old format:        parse_mcp_can(payload)
        print_can("mcp", payload)
    elif task == TASK_ID_CAN and code == COMMS_MSG_CAN_RAW:
        print_can("avr", payload)
    elif task == TASK_ID_CAN and code == CAN_STATS:
        print "Bus Error            ", as_u16(payload[0:2])
        print "Over time            ", as_u16(payload[2:4])
        print "Stuff Error          ", as_u16(payload[4:6])
        print "CRC Error            ", as_u16(payload[6:8])
        print "Frame Error          ", as_u16(payload[8:10])
        print "Ack Error            ", as_u16(payload[10:12])
        print "Unhandled Interrupt  ", as_u16(payload[12:14])
        print "MOB RXOK             ", as_u16(payload[14:16])
        print "MOB TXOK             ", as_u16(payload[16:18])
        print "MOB DLCW             ", as_u16(payload[18:20])
        print "MOB Bit Err          ", as_u16(payload[20:22])
        print "MOB Stuff Error      ", as_u16(payload[22:24])
        print "MOB CRC Error        ", as_u16(payload[24:26])
        print "MOB Frame Error      ", as_u16(payload[26:28])
        print "MOB Ack Error        ", as_u16(payload[28:30])
        print "RX filtered          ", as_u16(payload[30:32])
        print "RX relayed           ", as_u16(payload[32:34])
    elif task == TASK_ID_CAN and code == CAN_SET_BT:
        print "AVR CAN bit time set: CANBT[1..3] = ", [hex(x) for x in msg[2:5]];
    else:
        print "Unrecognized message", [hex(x) for x in msg]

def parse_mcp_can(msg):
    #print [hex(x) for x in msg]
    eid = -1
    sid = (msg[0]<<3) | (msg[1]>>5)
    if (msg[1] & (1<<3)):
        eid = (sid<<18) | ((msg[1]&3)<<16) | (msg[2]<<8) | (msg[3])
    plen = (msg[4]&0xF);
    payload = msg[5:(5+plen)]
    print "MCP2515 CAN frame: ",
    if eid == -1:
        print "SID %03X " % sid, bin(sid),
    else:
        print "EID %08X " % eid, bin(eid),   
    print [hex(x) for x in payload]

CAN_FLAG_EID = 1

def parse_can(msg):
    ts      = msg[1]<<8|msg[0]
    flags   = msg[2]
    payload = msg[7:]
    id      = (msg[6]<<24)|(msg[5]<<16)|(msg[4]<<8)|msg[3]
    return (id,flags,payload,ts)

def print_can(src, msg):
#typedef u16 can_20a_addr_t;
#typedef u32 can_20b_addr_t;
#typedef struct {
#	u8 ver_20b : 1; /* 11 bit IDs if zero; 29 bit if one */	
#	u8 rtr     : 1;	
#	union {
#		can_20a_addr_t s;
#		can_20b_addr_t e;
#	} id;
#} can_addr_t;
#define CAN_MAX_PAYLOAD 8    

    (id, flags, payload, ts) = parse_can(msg)

    print src, "CAN frame (flags %s) TS %d " % (bin(flags),ts),
    if (flags & CAN_FLAG_EID):  # first bit in field, i.e. ver_20b
        # 29 bit
#        eid = (msg[1]<<21)|(msg[2]<<13)|(msg[3]<<5)|(msg[4]>>3)
        print "EID %08X " % id, bin(id),
    else:
#        sid = (msg[1]<<3)|(msg[2]>>5)
        print "SID %03X " % id, bin(id),
    print [hex(x) for x in payload]


def avrsend(sid, data, eid=0):
    flags = 0
    if eid:
        flags = (flags | 1)
    msg = [flags, (sid)&0xFF, (sid>>8)&0xFF, (sid>>16)&0xFF, (sid>>24)&0xFF] + data
    send(TASK_ID_CAN, CAN_SEND_RAW_MSG, msg)

def mcpsend(sid, data, eid=0):
    flags = 0
    if eid:
        flags = (flags | 1)
    msg = [flags, (sid)&0xFF, (sid>>8)&0xFF, (sid>>16)&0xFF, (sid>>24)&0xFF] + data
    send(TASK_ID_MCP, CAN_SEND_RAW_MSG, msg)
    
def cansend(interface, sid, data, eid=0):
    if interface == 0:
        avrsend(sid, data, eid)
    else:
        mcpsend(sid, data, eid)

def canrcv():
    while 1:
        m = rcv()    
        if len(m) >= 2:
            task = m[0]
            code = m[1]
            payload = m[2:]    

            if (task == TASK_ID_MCP or task == TASK_ID_CAN) and code == COMMS_MSG_CAN_RAW:
                (id, flags, payload, ts) = parse_can(payload)        
                rxif = 0
                if task == TASK_ID_MCP:
                    rxif = 1
                return (rxif, id, flags, payload, ts)                

        parse(m)            


def canwalk(iter=1,start=0,end=40):
    mcpmode("normal")
    bad = 0
    good = 0
    j = 0
    while j < iter:
        for plen in range(0,9):
            for txif in range(0,2):
                for idbit in range(start,end):
                    if idbit < 11:
                        txeid = 0
                        txsid = (1<<idbit) + j
                    else:
                        txeid = 1
                        txsid = (1<<(idbit-11)) + j
                    txdata = []
                    for i in range(0,plen):
                        txdata.append(int(random.uniform(0,255)))
                    cansend(txif, txsid, txdata, txeid)

                    (rxif, rxsid, rxeid, rxdata) = canrcv()

                    mismatch = 0
                    if len(txdata) != len(rxdata):
                        mismatch = 1
                    else:
                        for td,rd in zip(txdata,rxdata):
                            if td != rd:
                                mismatch = 1

                    if mismatch == 0 and txif != rxif and txeid == rxeid and rxsid == txsid:
                        print "OK %d->%d eid=%d ID %X" % (txif,rxif,txeid,txsid), [hex(x) for x in txdata]
                        good=good+1
                    else:
                        print "BAD %d->%d: sent eid=%d ID %X received eid=%d %X" % (txif,rxif,txeid,txsid,rxeid,rxsid)
                        print "TX data", [hex(x) for x in txdata], "RX data", [hex(x) for x in rxdata]
                        bad=bad+1
        j = j + 1
    print "Good %d bad %d" % (good,bad)



def rcvraw():
    while 1:
        b = usbread_esc(1)[0]
        print "Raw",b

def datatest(pl,c):
    bad=0
    naks=0
    timeouts=0
    j=0
    lastgood=[]
    while j < c:
        p = [(j>>8)&0xFF,j&0xFF]
        for i in range(2,pl):
            p.append(int(random.uniform(0,255)))

        while 1:
            send(TASK_ID_COMMS, COMMS_MSG_ECHO_REQUEST, p)
    #        print "Sent:    ",p
            try:
                r = rcv()
            except TimeoutException:
                print "Timeout in iteration %d" % j
                timeouts = timeouts + 1
                return;
    #        print "Received:",r
            if checkNak(r):
#                print "NAK"
                naks = naks+1
            else:
                break

        if r[0] == TASK_ID_COMMS and r[1] == COMMS_MSG_ECHO_REPLY:
            mismatch = 0
            for sx,rx in zip(p,r[2:]):
                if sx != rx:
                    mismatch = 1
                    break
            if mismatch:
                print "Data mismatch: sent", [hex(x) for x in p]
                print " received          ", [hex(x) for x in r]
                print " last good rx      ", [hex(x) for x in lastgood]
                bad=bad+1
                rcvmsg() # resync
            else:
#                print "Data matched"
                lastgood=r
        else:
            print "Code mismatch: sent %X/%X received %X/%X" % (TASK_ID_COMMS,COMMS_MSG_ECHO_REQUEST,r[0],r[1]) 
            break
#        print "Iteration %d complete" % j
        j=j+1
    print "Received %d bad packets of %d with %d retransmits and %d timeouts" % (bad,j,naks,timeouts)
        
def echo():
    send(TASK_ID_COMMS, COMMS_MSG_ECHO_REQUEST, [])
    parse(rcv())

def stats():
    send(TASK_ID_COMMS, COMMS_MSG_STATS)
    rcvmsg()

def canstats(clear=1):
    send(TASK_ID_CAN, CAN_STATS, [clear])
    rcvmsg()


def setbaud(bps=9600, dblspeed=0):
    clkio = 16000000
    if dblspeed == 0:
        div = 16.0
    else:
        div = 8.0

    ubrr = int((1.0*clkio / (div*bps)) + .5) - 1

    efbps = clkio / (div*(ubrr+1))

    err = -(1 - (1.0*efbps/bps)) * 100

    print "Target BPS = %d Effective BPS = %d Error = %.1f%% UBRR = %02X (%d)" % (bps, efbps, err, ubrr, ubrr)
        
    send(TASK_ID_COMMS, COMMS_MSG_SET_BAUD, [ubrr, dblspeed])
    sleep(1)
    
    ser.baudrate = bps

def modem():
    ms = usb.getModemStatus()

    print "Modem status: %X" % ms

    bits = {'CTS':0x10, 'DSR':0x20, 'RI':0x40, 'DCD':0x80, 'OE':0x200, 'PE':0x400, 'FE':0x800, 'BI':0x1000}

    for n,m in bits.iteritems():
        if (ms&m):        
            print "%04s: SET" % n
        else:
            print "%04s: CLEAR" % n

    (rx, tx, evt) = usb.getStatus()
    print " txQ: %d" % tx
    print " rxQ: %d" % rx
    print " evt: %X" % evt

def mcpread(addr, len):
    send(TASK_ID_MCP, MCP_MSG_READ, [addr, len])
    rcvmsg()

def mcpwrite(addr, buf):
    send(TASK_ID_MCP, MCP_MSG_WRITE, [addr] + buf)

def mcpmode(mode,bits=7):
    if mode == "normal":
        reqop = 0
    elif mode == "sleep":
        reqop = 1
    elif mode == "loopback":
        reqop = 2
    elif mode == "listen":
        reqop = 3
    elif mode == "config":
        reqop = 4
    else:
        print "invalid mode: options are normal,sleep,loopback,listen,config"
        return
    rval = reqop<<5 | bits
    mcpwrite(0xF, [rval])
    mcpread(0xF, 1)

# set baud rate 250kbps
# mcpwrite(0x28,[5,0xb8,1])

def mcpsend_reg(sid, data, eid=0, txbuf=0):
    # registers: 0x30-0x3D + txbuf*0x10
    # TXBnCTRL: set bit 3 to request TX, rest as zero
    # TXBnSIDH: SID10:SID3
    # TXBnSIDL: SID2:SID0-EXIDE-EID17:16 set bit 3 for extended ID
    # TXBnEID8: EID15:EID8
    # TXBnEID0: EID7:EID0
    # TXBnDLC: bit 6 RTR bits 3-0 length
    # TXBnDm: 0-8 bytes

    tx = [1<<3, (sid>>3)&0xFF, (sid<<5)&0xFF, 0, 0, len(data)] + data

    if eid != 0:
        eidx = sid&0x3FFFF
        sidx = sid>>18
        tx = [1<<3, (sidx>>3)&0xFF, ((sidx<<5)&0xFF)| (1<<3) | ((eidx>>16)&3),  ((eidx>>8)&0xFF), (eidx&0xFF), len(data)] + data
    else:
        tx = [1<<3, (sid>>3)&0xFF, (sid<<5)&0xFF, 0, 0, len(data)] + data
        
    print [(hex(x), bin(x)) for x in tx]

    basereg = 0x30 + txbuf*0x10
    # address & data (the EID bytes could be skipped -- not sure if 
    # two writes is cheaper than one with two extra bytes)
    mcpwrite(basereg+1, tx[1:])
    # command (this could be done with the RTS pin)
    mcpwrite(basereg, tx[0:1])

# note: must be run in config mode
MCP_REG_CNF1 = 0x2A
def mcpbaud(bps):
    # This assumes CNF2/3 are programmed at startup to 0xB8 and 0x05 and Fosc is 16M
    if bps == 500000:
        brp = 0
    else:
        brp = ((((1000000000.0/bps)/16.0)/62.5)-2)/2 - 1
    print "BRP for %d BPS is %f" % (bps, brp)
    if int(brp) != brp:
        print "Warning: prescaler is not an integer"
    mcpwrite(MCP_REG_CNF1, [int(brp)])
    mcpread(MCP_REG_CNF1, 3)

def avrbaud(bps):    
    canbt1 = {100000:18, 125000:14, 200000:8, 250000:6, 500000:2, 1000000:0}
    if bps in canbt1:
        send(TASK_ID_CAN, CAN_SET_BT, [canbt1[bps]])
        rcvmsg()
    else:
        print "Invalid AVR baud rate %d" % bps

def canspeed(bps):
    mcpbaud(bps)
    avrbaud(bps)

def resetavr():
    serclose()
    usbopen()
    # toggle CBUS2 (/RST) 
    usb.setBitMode(0x40,0x20)
    usb.setBitMode(0x44,0x20)
    usbclose()
    print "reset opening serial"
    seropen(9600)
    print "serial opened"

def newspeed(bps=9600):
    ser.baudrate = 9600
    resetavr()
    # wait for AVR to boot
    sleep(1)
    # switch to fast mode
    setbaud(bps)
    stats()


# main program
twocan = -1
usb = -1
ser = -1
# list devices by description, returns tuple of attached devices description strings
dl = d2xx.listDevices(d2xx.OPEN_BY_DESCRIPTION)
for num,name in enumerate(dl):
    print "Found device " + str(num) + " named " + name
    if name == "2CAN":
        twocan = num
        break
if twocan < 0:
    print "2CAN not found"
    raise BaseException

def usbopen():
    global usb
    usb = d2xx.open(twocan)
def usbclose():
    global usb
    usb.close()

usbopen()
print usb.getDeviceInfo()
port = usb.getComPortNumber()
print "Virtual COM port is COM%d " % port
usbclose()

rxq = Queue(50)
rxqctrl = Queue(1)


def seropen(bps):
    global ser, port
    timeout = 0.1
    ser = serial.Serial(port-1,bps,serial.EIGHTBITS,serial.PARITY_NONE,serial.STOPBITS_ONE,timeout,False,True,None,False,None)
    rxqctrl.put("go")
def serclose():
    rxqctrl.put("wait")
    rxq.get()
    print "Sleep waiting for rxq stop"
    sleep(1)
    global ser
    ser.close()

seropen(9600)
print "Opened serial port", ser.portstr


def rawread(n):
    bv = []
    for i in range(0,n):
        b = usbread(1)[0]
        bv.append(b)
    return bv


def enter_bootloader():
    serclose()
    usbopen()
    # assert CBUS2 (/RST)
    # set CBUS3 high (enter bootloader mode)
    usb.setBitMode(0xC8,0x20)

    # deassert /RST, leave CBUS3 high
    usb.setBitMode(0xCC,0x20)
    usbclose()
    print "bootloader opening serial"
    seropen(38400)
    print "serial opened, waiting for bootloader..."
    try:
        s = rawread(6);
        print "got '%s'"%(s)
    except TimeoutException:
        print "Timed out waiting for bootloader"

def load_hex(filename):
    import ihex
    f = ihex.IHex();
    f.read_file(filename);
    for addr, data in f.areas.iteritems():
        print "Area at %X for %X"%(addr, len(data))
        
def bootloader():

        
    while 1:
        try:
            input("2CAN bootloader>")
        except exceptions.KeyboardInterrupt:
            break
        except exceptions.EOFError:
            break
        except exceptions.ValueError:
            break
        except:
            (t, v, tb) = sys.exc_info()
            print "Exception:", t, v
            print traceback.print_tb(tb)
    sys.exit(1)


if len(sys.argv) > 1:
    if sys.argv[1] == 'prog':
        print "Entering bootloader mode"
        bootloader()
    else:
        print "unrecognized argument"


def rxhandler():
    cbs = []
    while 1:
        try:
            m = rcv_direct()
            havecb = 0
            for c in filter(lambda x: x[0:2] == m[0:2], cbs):
#                print "Callback for %X/%X" % (c[0], c[1])
                havecb = 1
                global rxtime
                rxtime = time.time()
                c[2](m)
            if havecb == 0:
                rxq.put(m)
        except TimeoutException:
            pass
        if not rxqctrl.empty():
            what = rxqctrl.get()
            if what == "stop":
                print "RX handler stopped"
                break
            elif what == "wait":
                print "RX handler waiting"
                rxq.put([])
                block = rxqctrl.get()
            elif what == "go":
                print "RX handler: go"
                pass
            elif isinstance(what, (list,tuple)):
                if what[0] == "addcb":
#                    print "Add callback for %X/%X" % (what[1],what[2])
                    cbs.append([what[1], what[2], what[3]])
                elif what[0] == "delcb":
#                    for c in filter(lambda x: x[0:3] == what[1:4], cbs):
#                        print "Delete callback for %X/%X" % (what[1],what[2])
                    cbs.remove([what[1], what[2], what[3]])
                elif what[0] == "resetcb":
                    cbs = []
                else:
                    print "RX handler: unknown list command"
            else:
                print "RX handler: unknown command " + what


rxthread = threading.Thread(target=rxhandler)
rxthread.start()

def rcv():
    try :
        m = rxq.get(True,1)
        #print "rxq.get: ", m
        return m
    except Empty:
        raise TimeoutException(1)

def add_rcv_cb(task, code, func):
    rxqctrl.put(["addcb", task, code, func])

def del_rcv_cb(task, code, func):
    rxqctrl.put(["delcb", task, code, func])

tracing = 0

def tracemsg(prefix,ts,id,flags,data):
    global tracing
    if not tracing:
        return

    if flags&CAN_FLAG_EID:
        type = "EID"
    else:
        type = "SID"

    h = ""
    for x in data:
        h += "%02X " % (x)

    print "%2d.%03d %s (%02X) %s %03X " % (ts/1000, ts%1000, prefix, flags, type, id), h

paused = 0

def canframecb(msg):
    task = msg[0]
    code = msg[1]
    payload = msg[2:]

    if checkNak(msg):
        raise NakException(0)
    
    rxif = -1
    txif = -1

    if task == TASK_ID_MCP and code == COMMS_MSG_CAN_RAW:
        rxif = 1
        txif = 0
    elif task == TASK_ID_CAN and code == COMMS_MSG_CAN_RAW:
        rxif = 0
        txif = 1
    else:
        print "Unknown message received in canframecb: ", msg
        return
 
    (canid, flags, data, ts) = parse_can(payload)

    if do_relay(canid, len(data)):
        cansend(txif, canid, data)

    canframe = {'canid': canid, 
                'flags': flags, 
                'timestamp':ts, 
                'interface':rxif, 
                'dlc': len(data), 
                'annotation': get_annotation(id, len(data)),
                'data':data}

    frameout(canframe)

def frameout(canframe):
    global tofile
    if tofile:
        global rxtime
        ts = strftime("%%H:%%M:%%S.%03d"%((rxtime-int(rxtime))*1000),time.localtime(rxtime))
        log = "%s %03X %s\n"%(ts, canframe['canid'], ' '.join(["%02X"%(i) for i in canframe['data']]))
        tofile.write(log)
        #print log
        return

    logframe(canframe)    
    if not do_filter(canframe['canid'], canframe['dlc']):
        tracemsg("RX%d"%(canframe['interface']), canframe['timestamp'],canframe['canid'],canframe['flags'],canframe['data'])
        global paused
        if not paused:
            webout(json.dumps({'canframe':canframe}));

def webstatus(status):
    webout(json.dumps({'statustext':status}));

logfilename = strftime("%Y-%M-%d-%H-%M.plog");
logging = 0
logpickles = []

def logframe(frame):
    global logging    
    if logging:
        global logpickles
        logpickles.append(frame)

def unlog(filename):
    f = open(filename, "rb")
    log = pickle.load(f);
    for frame in log:
        frameout(frame)

def tofileonly(filename):
    global tofile
    tofile = open(filename, "w+")

def closetofile():
    global tofile
    tofile.close()
    tofile = None

def statscb(msg):
    task = msg[0]
    code = msg[1]
    payload = msg[2:]
    js = {}

    if task == TASK_ID_CAN and code == CAN_STATS:        
        js["Bus_Error"]             = as_u16(payload[0:2])
#       js["Over time"]             = as_u16(payload[2:4])
        js["Stuff_Error"]           = as_u16(payload[4:6])
        js["CRC_Error"]             = as_u16(payload[6:8])
        js["Frame_Error"]           = as_u16(payload[8:10])
        js["Ack_Error" ]            = as_u16(payload[10:12])
        js["Unhandled_Interrupt"]   = as_u16(payload[12:14])
        js["MOB_RXOK"]              = as_u16(payload[14:16])
        js["MOB_TXOK"]              = as_u16(payload[16:18])
#        js["MOB DLCW"]              = as_u16(payload[18:20])
        js["MOB_Bit_Err"]           = as_u16(payload[20:22])
        js["MOB_Stuff_Error"]       = as_u16(payload[22:24])
        js["MOB_CRC_Error"]         = as_u16(payload[24:26])
        js["MOB_Frame_Error"]       = as_u16(payload[26:28])
        js["MOB_Ack_Error"]         = as_u16(payload[28:30])
        js["RX_Filtered"]           = as_u16(payload[30:32])
        js["RX_Relayed"]            = as_u16(payload[32:34])

    if task == TASK_ID_COMMS and code == COMMS_MSG_STATS:
        js["Junk_bytes"] = as_u16(payload[16:18])
        js["Error_bytes"] = as_u16(payload[18:20])
        js["Checksum_errors"] = as_u16(payload[20:22])
        js["Bad_code_packets"] = as_u16(payload[22:24])
        js["RX_ring_overflow"] = as_u16(payload[24:26])
        js["RX_no-buffers"] = as_u16(payload[26:28])
        js["Uptime"] = as_u32(payload[30:34])
        js["TX_no_buffers"] = as_u16(payload[34:36])
        js["Mailbox_overflow"] = as_u16(payload[36:38])
        js["UART_RX_overrun"] = as_u16(payload[38:40])
        js["UART_RX_frame error"] = as_u16(payload[40:42])
        js["UART_RX_parity error"] = as_u16(payload[42:44])
        js["UART_TX_drop_ISR"] = as_u16(payload[46:48])

    webout(json.dumps({'stats':js}))

timerstop = None

def initbus():
    webstatus("Resetting 2CAN")
    newspeed(500000)
    webstatus("Setting CAN speed to 500kbps")
    canspeed(500000)
    mcpmode("normal")
    
    rxqctrl.put(["resetcb"]);

    add_rcv_cb(TASK_ID_MCP, COMMS_MSG_CAN_RAW, canframecb)
    add_rcv_cb(TASK_ID_CAN, COMMS_MSG_CAN_RAW, canframecb)

#    add_rcv_cb(TASK_ID_CAN, CAN_STATS, statscb)
#    add_rcv_cb(TASK_ID_COMMS, COMMS_MSG_STATS, statscb)
    
    update_hwfilters()

    webstatus("Monitoring both CAN busses @ 500kbps")

    def stats():
        #send(TASK_ID_CAN, CAN_STATS, [0])
        #send(TASK_ID_COMMS, COMMS_MSG_STATS)
        update_summary()
        if timerstop == None:
            threading.Timer(1, stats).start()

    stats()

def acceptfilter(sid, mask):
    # program hardware acceptance filter to only accept the given SID
    idtidm = range(0,8)
    idtidm[0] = (mask&0x7FF)>>3
    idtidm[1] = (mask&7)<<5
    idtidm[2] = 0
    idtidm[3] = 0 
    idtidm[4] = (sid&0x7FF)>>3
    idtidm[5] = (sid&7)<<5
    idtidm[6] = 0
    idtidm[7] = 0 
    send(TASK_ID_CAN, CAN_SET_ACCEPT_FILTER, idtidm)
        

def logone(sid):
    webstatus("Resetting 2CAN")
    newspeed(500000)
    webstatus("Setting CAN speed")
    canspeed(500000)
    mcpmode("normal")
    
    rxqctrl.put(["resetcb"]);
    add_rcv_cb(TASK_ID_CAN, COMMS_MSG_CAN_RAW, canframecb)

    acceptfilter(sid, 0x7FF)
    filename = strftime("%m-%d_%H_%M")+"_%03X"%(sid)+".log"
    print "Logging to %s"%(filename)
    tofileonly(filename)


pcancelf = 0
def periodic(ms, iface, sid, data):
    global pcancelf
    pcancelf = 0
    def psend():
	cansend(iface, sid, data)
	print "sent %03X to if %d"%(sid, iface)
	if not pcancelf:
	    threading.Timer(.5, psend).start()
    psend()

def pcancel():
   global pcancelf
   pcancelf = 1


def logtoggle():
    global logging, logfilename
    logging = not logging
    webstatus("logging %s: %s" % (logging, logfilename))



def msgrelay(msg):
    (id, flags, data) = parse_can(msg[2:])
    # this was closing over dstif below but we can't remove the callback
    # by matching closures...
    if msg[0] == TASK_ID_MCP:
        dstif=0
    else:
        dstif=1
    tracemsg("TX%d"%(dstif), id,flags,data)
    cansend(dstif, id, data, 0) # always send in 11 bit mode

def relay(srcif, on=1):
    task = 0
    dstif=-1
    if srcif == 0:
        task = TASK_ID_CAN
        dstif = 1
    else:
        task = TASK_ID_MCP
        dstif = 0

    if on:
        add_rcv_cb(task, COMMS_MSG_CAN_RAW, msgrelay)
    else:
        del_rcv_cb(task, COMMS_MSG_CAN_RAW, msgrelay)

def scopetest():
    while 1:
        cansend(0,0x55,[0xAA])
        sleep(.025)
        cansend(1,0x55,[0xAA])
        sleep(.025)

annotations = {}
relay = {}
msgfilter = {}
pickles = {}
try:
    f = open("anno.p", "rb")
    pickles = pickle.load(f);
    print "Read pickles: ",pickles
    annotations = pickles['annotations']
    relay = pickles['relay']
    msgfilter = pickles['filter']
    f.close()
except:
    print "Failed to unpickle"


import tornado.ioloop
import tornado.web
import sockjs.tornado  

listeners = set()

# would be nice to replace this with http://www.tornadoweb.org/documentation/_modules/tornado/web.html#StaticFileHandler or inherit that somehow
class IndexHandler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html')

class CanConnection(sockjs.tornado.SockJSConnection):
    
    def on_open(self, info):
        listeners.add(self)

    def on_message(self, message):
#        print "websocket message", message;
        m = json.loads(message);
        try:
            for k in m:
                if k == 'eval':
                    eval(m['eval']);
                if k == 'annotate':
                    set_annotation(m['annotate'])
                    r = json.dumps(m);
#                    print "reply: ", r
                    webout(r)
                if k == 'relay':
                    relay[annotation_key(m[k]['id'], m[k]['dlc'])] = m[k]['relay']
                    r = json.dumps(m);
                    webout(r)
                if k == 'filter':
                    add_filter(m[k])
                    r = json.dumps(m);
                    webout(r)
                if k == 'globalinvert':
                    v = 0
                    v |= (m[k]['hwf0']<<0)
                    v |= (m[k]['hwr0']<<1)
                    v |= (m[k]['hwf1']<<2)
                    v |= (m[k]['hwr1']<<3)
                    send(TASK_ID_CAN, CAN_SET_SID_FILTER, [v])
                    print "Global invert set to %X"%(v)
                    webout(json.dumps(m))
                    
        except:
            traceback.print_exc();
            webstatus("onmessage exception")

        

    def on_close(self):
        listeners.remove(self)

router = sockjs.tornado.SockJSRouter(CanConnection, '/can')
app = tornado.web.Application(
        [(r"/", IndexHandler), 
         (r"/s/images/(.*)", tornado.web.StaticFileHandler, {"path":os.path.dirname(__file__)}),
         (r"/s/(.*)", tornado.web.StaticFileHandler, {"path":os.path.dirname(__file__)}),
        ] + router.urls
)

def webhandler():
    app.listen(8888)
    tornado.ioloop.IOLoop.instance().start()

webthread = threading.Thread(target=webhandler)
webthread.start()

def webout(s):
    router.broadcast(listeners,s)

def filter_debug(id, dlc, swf, swr, hwf, hwr, count):
    m = {}
    m['id'] = id;
    m['dlc'] = dlc;
    m['swf'] = swf;
    m['swr'] = swr;
    m['hwf'] = hwf;
    m['hwr'] = hwr;
    m['count'] = count;
    s = json.dumps({'filter':m});
    print "Sending ", s
    webout(s)

hwfilter_cache = []

# the hardware filters are 14 16 bit values divided into two blocks which can be updated separately
# and a single 8 bit value that can invert the sense of each bit
# each word:
#  15: Relay if received on interface 1
#  14: Don't send to host if received on interface 1
#  13: Relay if received on interface 0
#  12: Don't send to host if received on interface 0
#  11: entry is valid (search stops on first entry with this bit clear)
#  [10:0] : SID of CAN frame to match

def update_hwfilters():
    proposed = sorted(filter(lambda x: msgfilter[x]['hwf'], msgfilter) + filter(lambda x: msgfilter[x]['hwr'], msgfilter))
    print "proposed: ", proposed
#    proposed = filter(lambda x: msgfilter[x]['hwf'] or msgfilter[x]['hwr'], msgfilter).sort()
    global hwfilter_cache
    if proposed == hwfilter_cache:
        return

    banks = 2
    entries = 7
    data = []

    for pk in set(proposed):
        v = (msgfilter[pk]['canid'] | (1<<11))
        if msgfilter[pk]['hwf']:
            v = (v | (1<<12) | (1<<14))  # both interfaces
        if msgfilter[pk]['hwr']:
            v = (v | (1<<13) | (1<<15))  # both interfaces
        data.append(v)

    if len(data) > banks*entries:
        print "Too many filters!  Have ", len(data)
    
    while len(data) < banks*entries:
        data.append(0)

    hwfilter_cache = proposed

    for bank in range(0,banks):
        t = [(x&0xFF, x>>8) for x in data[bank*entries:(bank+1)*entries]]
        p = [bank] + [x for y in t for x in y]
        print "Sending filters", [hex(x) for x in p]
        send(TASK_ID_CAN, CAN_SET_SID_FILTER, p)
        



def add_filter(m):
    keys = ['swf', 'hwf', 'hwr', 'swr', 'canid', 'dlc'];
    fk = annotation_key(m['canid'], m['dlc'])
    if not fk in msgfilter:
        msgfilter[fk] = {}
    for k in keys:
        print "Filter for ID %03X DLC %d: %s = %d"%(m['canid'], m['dlc'], k, m[k])
        msgfilter[fk][k] = m[k]

    update_hwfilters()

def do_relay(canid, dlc):
    fk = annotation_key(canid, dlc)
    if fk in msgfilter:
        if msgfilter[fk]['swr']:
            return 1
    return 0

def do_filter(canid, dlc):
    fk = annotation_key(canid, dlc)
    if fk in msgfilter:
        msgfilter[fk]['count'] += 1
        if msgfilter[fk]['swf']:
            return 1
            
    else:
        # first occurrance
        msgfilter[fk] = {}
        msgfilter[fk]['canid'] = canid
        msgfilter[fk]['dlc'] = dlc
        msgfilter[fk]['count'] = 1
        msgfilter[fk]['swf'] = 0
        msgfilter[fk]['hwf'] = 0
        msgfilter[fk]['hwr'] = 0
        msgfilter[fk]['swr'] = 0
        
    return 0

def update_summary():
    for fk in msgfilter:
        a = {'filter':msgfilter[fk]};
        a['filter']['anno'] = ""
        if fk in annotations:
            a['filter']['anno'] = annotations[fk]
        x = json.dumps(a)
        webout(x)

def annotation_key(canid, dlc):
    return str(canid) + "_" + str(dlc)

def get_annotation(canid, dlc):
    a = ""
    try:
        a = annotations[annotation_key(canid, dlc)]
    except:
        pass
#    print "Annotation for %02X:%d is '%s'"%(id,dlc,a)
    return a

def set_annotation(m):
    annotations[annotation_key(m['id'], m['dlc'])] = m['value']

def pause(p):
    global paused
    if p == 1:
        print "PAUSED"
        paused = 1
    else:
        print "RESUMED"
        paused = 0

def auxdata(kv):
    logframe(kv)
    global paused
    if not paused:
        webout(json.dumps({'auxdata':kv}));

while 1:
    try:
        input("2CAN py>")
    except exceptions.KeyboardInterrupt:
        break
    except exceptions.EOFError:
        break
    except exceptions.ValueError:
        break
    except:
        (t, v, tb) = sys.exc_info()
        print "Exception:", t, v
        print traceback.print_tb(tb)

rxqctrl.put("stop")
timerstop = 1
print "Stopping tornado"
tornado.ioloop.IOLoop.instance().stop()
print "Ready to exit"

f = open("anno.p", "wb")
pickles['annotations'] = annotations
pickles['relay'] = relay
pickles['filter'] = msgfilter
pickle.dump(pickles, f)
print "Wrote pickles: ",pickles
f.close()

if len(logpickles) > 0:
    f = open(logfilename, "wb")
    pickle.dump(logpickles, f)
    f.close()


