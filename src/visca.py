#! /usr/bin/env python
# -*- coding: utf8 -*-
#
#    PyVisca - Implementation of the Visca serial protocol in python
#    Copyright (C) 2013  Florian Streibelt pyvisca@f-streibelt.de
#
#    This program is free software; you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, version 2 only.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program; if not, write to the Free Software
#    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
#    USA

'''
PyVisca by Florian Streibelt <pyvisca@f-streibelt.de>
https://github.com/mutax/PyVisca/blob/master/pyviscalib/visca.py

Adopted for python3/mypy by Alex Sokolsky <asokolsky@gmail.com>
TODO: better error handling - exit(1) is not good enough
'''

from serial import Serial, SerialException
import sys
from threading import Lock
from typing import Optional

class ViscaError(IOError):
    """Base class for Visca errors."""
    pass

class Visca():

    def __init__(self, port:str = "/dev/ttyUSB0") -> None:
        # guards access to self.serialport
        self.lock = Lock()
        #
        self.portname = port
        self.serialport: Optional[Serial] = None
        self.open_port()
        return

    def open_port(self) -> None:
        '''
        Open the port named self.portname and set self.serialport
        '''

        if self.serialport is not None:
            return
        try:
            self.lock.acquire()
            self.serialport = Serial(
                self.portname, 9600, timeout=2, stopbits=1, bytesize=8,
                rtscts=False, dsrdtr=False)
            assert self.serialport is not None
            #self.serialport.flushInput()

        except SerialException as e:
            print(f"SerialException opening port '{self.portname}':", e)
            #raise e
            raise ViscaError(e.strerror)

        except Exception as e:
            print(f"Exception opening port '{self.portname}':", e)
            #raise e

        finally:
            self.lock.release()
        return


    def dump(self, packet:bytes, title:Optional[str] = None) -> None:
        '''
        Dump the packet
        '''
        if not packet or len(packet) == 0:
            return

        header = packet[0]
        term = packet[-1:]
        qq = packet[1]

        sender: int = (header & 0b01110000) >> 4
        broadcast: int = (header & 0b1000) >> 3
        recipient: int = (header & 0b0111)

        if broadcast:
            recipient_s = "*"
        else:
            recipient_s = str(recipient)

        print("-----")
        if title:
            print(f"packet ({title}) [{sender} => {recipient_s}] len={len(packet)}: {packet.hex()}")
        else:
            print(f"packet [{sender} => {recipient_s}] len={len(packet)}: {packet.hex()}")

        line = f" QQ.........: {qq:02x}"
        if qq == 0x01:
            line += " (Command)"
        if qq == 0x09:
            line += " (Inquiry)"
        print(line)

        if len(packet) > 3:
            rr = packet[2]
            line = f" RR.........: {rr:02x}"
            if rr == 0x00:
                line += " (Interface)"
            if rr == 0x04:
                line += " (Camera [1])"
            if rr == 0x06:
                line += " (Pan/Tilter)"
            print(line)

        if len(packet) > 4:
            data = packet[3:-1]
            print(f" Data.......: {data.hex()}")

        if term != b'\xff':
            print("ERROR: Packet not terminated correctly")
            return

        if len(packet) == 3 and ((qq & 0b11110000) >> 4) == 4:
            socketno = (qq & 0b1111)
            print(" packet: ACK for socket %02x" % socketno)

        if len(packet) == 3 and ((qq & 0b11110000) >> 4) == 5:
            socketno = (qq & 0b1111)
            print(" packet: COMPLETION for socket %02x" % socketno)

        if len(packet) > 3 and ((qq & 0b11110000) >> 4) == 5:
            socketno = (qq & 0b1111)
            ret = packet[2:-1].hex()
            print(f" packet: COMPLETION for socket {socketno}, data={ret}")

        if len(packet) == 4 and ((qq & 0b11110000) >> 4) == 6:
            print(" packet: ERROR!")

            socketno = (qq & 0b00001111)
            errcode = packet[2]

            #these two are special, socket is zero and has no meaning:
            if errcode == 0x02 and socketno == 0:
                print("        : Syntax Error")
            if errcode == 0x03 and socketno == 0:
                print("        : Command Buffer Full")

            if errcode == 0x04:
                print(f"        : Socket {socketno}: Command canceled")

            if errcode == 0x05:
                print(f"        : Socket {socketno}: Invalid socket selected")

            if errcode == 0x41:
                print(f"        : Socket {socketno}: Command not executable")

        if len(packet) == 3 and qq == 0x38:
            print("Network Change - we should immediately issue a renumbering!")
        return

    def recv_packet(self, extra_title: Optional[str] = None) -> bytes:
        '''
        Receive a packet - up to 16 bytes until 0xff
        '''
        assert self.serialport is not None
        packet = bytearray()
        count = 0
        while count < 16:
            s = self.serialport.read(1)
            if s:
                packet.extend(s)
                count += 1
            else:
                print("ERROR: Timeout waiting for reply")
                break
            if s == b'\xff':
                break

        if extra_title:
            self.dump(packet, f"recv: {extra_title}")
        else:
            self.dump(packet, "recv")
        return packet

    def _write_packet(self, packet:bytes) -> None:
        '''
        Write packet to self.serialport
        '''
        assert self.serialport is not None
        if not self.serialport.is_open:
            sys.exit(1)

        # lets see if a completion message or something
        # else waits in the buffer. If yes dump it.
        if self.serialport.in_waiting:
            self.recv_packet("ignored")

        self.serialport.write(packet)
        self.dump(packet,"sent")
        return

    def send_packet(self, recipient:int, data:bytes) -> Optional[bytes]:
        """
        according to the documentation:

        |------packet (3-16 bytes)---------|
         header     message      terminator
         (1 byte)  (1-14 bytes)  (1 byte)

        | X | X . . . . .  . . . . . X | X |

        header:                  terminator:
        1 s2 s1 s0 0 r2 r1 r0     0xff

        with r,s = recipient, sender msb first
        for broadcast the header is 0x88!
        we use -1 as recipient to send a broadcast!
        """
        # we are the controller with id=0
        sender = 0
        if recipient == -1:
            #broadcast:
            rbits = 0x8
        else:
            # the recipient (address = 3 bits)
            rbits = recipient & 0b111

        sbits = (sender & 0b111) << 4
        header = 0b10000000 | sbits | rbits
        terminator = b'\xff'
        packet = bytes([header]) + data + terminator

        try:
            self.lock.acquire()
            self._write_packet(packet)
            reply = self.recv_packet()

        except SerialException as e:
            print(f"SerialException opening port '{self.portname}':", e)

        finally:
            self.lock.release()

        if not reply:
            return None

        if reply[-1:] != b'\xff':
            print(f"received packet not terminated correctly: {reply.hex()}")
            return None

        return reply

    def send_broadcast(self, data: bytes) -> Optional[bytes]:
        # shortcut
        return self.send_packet(-1, data)

    def i2v(self, value:int) -> bytes:
        """
        return value as 4-byte dword in Visca format
        packets are not allowed to be 0xff
        so for numbers the first nibble is 0000
        and 0xfd gets encoded into 0x0f 0x0xd
        """
        ms = (value & 0b1111111100000000) >> 8
        ls = (value & 0b0000000011111111)
        p = (ms & 0b11110000) >> 4
        r = (ls & 0b11110000) >> 4
        q = ms & 0b1111
        s = ls & 0b1111
        return bytes([p, q, r, s])

    def cmd_adress_set(self) -> None:
        """
        starts enumerating devices, sends the first address to use on the bus
        reply is the same packet with the next free address to use
        """

        #address of first device. should be 1:
        first = 1
        # set address
        reply = self.send_broadcast(bytes([0x30, first]))
        if not reply:
            print("No reply from the bus.")
            sys.exit(1)

        if len(reply) != 4 or reply[-1:] != b'\xff':
            print("ERROR enumerating devices")
            sys.exit(1)

        if reply[0] != 0x88:
            print("ERROR: expecting broadcast answer to an enumeration request")
            sys.exit(1)

        address = reply[2]
        d = address - first
        print(f"debug: found {d} devices on the bus")
        if d == 0:
            sys.exit(1)
        return

    def cmd_if_clear_all(self) -> None:
        '''
        interface clear all
        '''
        reply = self.send_broadcast(b'\x01\x00\x01')
        if reply is None:
            print("ERROR sending broadcast")
            sys.exit(1)

        if not reply[1:] == b'\x01\x00\x01\xff':
            print("ERROR clearing all interfaces on the bus!")
            sys.exit(1)

        print("debug: all interfaces clear")
        return

    def cmd_cam(self, device:int, subcmd:bytes) -> Optional[bytes]:
        '''
        Send the command.  Returns reply
        '''
        reply = self.send_packet(device, b'\x01\x04' + subcmd)
        #FIXME: check returned data here and retransmit?
        return reply

    def cmd_pt(self, device:int, subcmd:bytes) -> Optional[bytes]:
        reply = self.send_packet(device, b'\x01\x06' + subcmd)
        #FIXME: check returned data here and retransmit?
        return reply

    #
    # POWER control
    #
    def cmd_cam_power(self, device:int, onoff:bool) -> Optional[bytes]:
        if onoff:
            pwr = b'\x00\x02'
        else:
            pwr = b'\x00\x03'
        return self.cmd_cam(device, pwr)

    def cmd_cam_power_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_power(device, True)

    def cmd_cam_power_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_power(device, False)

    def cmd_cam_auto_power_off(self, device:int, time:int = 0) -> Optional[bytes]:
        """
        time = minutes without command until standby
        0: disable
        0xffff: 65535 minutes
        """
        return self.cmd_cam(device, b'\x40' + self.i2v(time))

    #
    # ZOOM control
    #
    def cmd_cam_zoom_stop(self, device:int) -> Optional[bytes]:
        return self.cmd_cam(device, b'\x07\x00')

    def cmd_cam_zoom_tele(self, device:int) -> Optional[bytes]:
        return self.cmd_cam(device, b'\x07\x02')

    def cmd_cam_zoom_wide(self, device:int) -> Optional[bytes]:
        return self.cmd_cam(device, b'\x07\x03')

    def cmd_cam_zoom_tele_speed(self, device:int, speed:int) -> Optional[bytes]:
        """
        zoom in with speed = 0..7
        """
        sbyte = 0x20 + (speed & 0b111)
        subcmd = bytes([0x07, sbyte])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_zoom_wide_speed(self, device:int, speed:int) -> Optional[bytes]:
        """
        zoom in with speed = 0..7
        """
        sbyte = 0x30+(speed & 0b111)
        subcmd = bytes([0x07, sbyte])
        return self.cmd_cam(device,subcmd)

    def cmd_cam_zoom_direct(self, device:int, zoom:int) -> Optional[bytes]:
        """
        zoom to value
        optical: 0..4000
        digital: 4000..7000 (1x - 4x)
        """
        return self.cmd_cam(device, b'\x47' + self.i2v(zoom))

    def cmd_cam_dzoom(self, device:int, state:bool) -> Optional[bytes]:
        '''
        Digital Zoom control on/off
        '''
        if state:
            subcmd = b'\x06\x02'
        else:
            subcmd = b'\x06\x03'
        return self.cmd_cam(device, subcmd)

    def cmd_cam_dzoom_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_dzoom(device, True)

    def cmd_cam_dzoom_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_dzoom(device, False)

#FIXME: CAM_FOCUS COMMANDS
#FIXME: CAM_WB
#FIXME: CAM_?GAIN
#FIXME: CAM_AE
#FIXME: CAM_SlowShutter
#FIXME: CAM_Shutter
#FIXME: CAM_Iris
#FIXME: CAM_Gain
#FIXME: CAM_Bright
#FIXME: CAM_ExpComp
#FIXME: CAM_BackLight
#FIXME: CAM_Aperature

    def cmd_cam_wide(self, device:int, mode:int) -> Optional[bytes]:
        '''
        16:9 / Wide format:
        '''
        subcmd = bytes([0x60, mode])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_wide_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x00)

    def cmd_cam_wide_cinema(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x01)

    def cmd_cam_wide_full(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x02)

    # mirror
    def cmd_cam_lr_reverse(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x61, mode])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_lr_reverse_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_lr_reverse(device, 0x02)

    def cmd_cam_lr_reverse_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_lr_reverse(device, 0x03)

    # freeze
    def cmd_cam_freeze(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x62, mode])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_freeze_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_freeze(device, 0x02)

    def cmd_cam_freeze_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_freeze(device, 0x03)

    # Picture Effects
    def cmd_cam_picture_effect(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x63, mode])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_picture_effect_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x00)

    def cmd_cam_picture_effect_pastel(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x01)

    def cmd_cam_picture_effect_negart(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x02)

    def cmd_cam_picture_effect_sepa(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x03)

    def cmd_cam_picture_effect_bw(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x04)

    def cmd_cam_picture_effect_solarize(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x05)

    def cmd_cam_picture_effect_mosaic(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x06)

    def cmd_cam_picture_effect_slim(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x07)

    def cmd_cam_picture_effect_stretch(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_picture_effect(device, 0x08)

    def cmd_cam_digital_effect(self, device:int, mode:int) -> Optional[bytes]:
        '''
        Apply Digital Effect
        '''
        subcmd = bytes([0x64, mode])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_digital_effect_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_digital_effect(device, 0x00)

    def cmd_cam_digital_effect_still(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_digital_effect(device, 0x01)

    def cmd_cam_digital_effect_flash(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_digital_effect(device, 0x02)

    def cmd_cam_digital_effect_lumi(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_digital_effect(device, 0x03)

    def cmd_cam_digital_effect_trail(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_digital_effect(device, 0x04)

    def cmd_cam_digital_effect_level(self, device:int, level:int) -> Optional[bytes]:
        subcmd = bytes([0x65, 0b00111111 & level])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_memory(self, device:int, func:int, num:int) -> Optional[bytes]:
        '''
        memory of settings including position
        '''
        if num > 5:
            num = 5
        if func < 0 or func > 2:
            print(f"DEBUG: cmd_cam_memory bad func={func}")
            return None

        print("DEBUG: cam_memory command")
        subcmd = bytes([0x3f, func, (0b0111 & num)])
        return self.cmd_cam(device, subcmd)

    def cmd_cam_memory_reset(self, device:int, num:int) -> Optional[bytes]:
        #FIXME: Can only be executed when motion has stopped!!!
        return self.cmd_cam_memory(device, 0x00, num)

    def cmd_cam_memory_set(self, device:int, num:int) -> Optional[bytes]:
        return self.cmd_cam_memory(device,0x01,num)

    def cmd_cam_memory_recall(self, device:int, num:int) -> Optional[bytes]:
        return self.cmd_cam_memory(device,0x02,num)

    def cmd_datascreen(self, device:int, func:int) -> Optional[bytes]:
        '''
        Datascreen control
        '''
        subcmd = bytes([0x06, func])
        return self.cmd_pt(device, subcmd)

    def cmd_datascreen_on(self, device:int) -> Optional[bytes]:
        return self.cmd_datascreen(device, 0x02)

    def cmd_datascreen_off(self, device:int) -> Optional[bytes]:
        return self.cmd_datascreen(device, 0x03)

    def cmd_datascreen_toggle(self, device:int) -> Optional[bytes]:
        return self.cmd_datascreen(device, 0x10)

#FIXME: IR_Receive
#FIXME: IR_Receive_Return

    def cmd_ptd(self, device:int, ps:int, ts:int, lr:int, ud:int) -> Optional[bytes]:
        '''
        Pan and Tilt Drive - pan speed, tilt speed, left/right, up/down
        '''
        subcmd = bytes([0x01, ps, ts, lr, ud])
        return self.cmd_pt(device, subcmd)

    def cmd_ptd_up(self, device:int, ts:int = 0x14) -> Optional[bytes]:
        return self.cmd_ptd(device, 0, ts, 0x03, 0x01)

    def cmd_ptd_down(self, device:int,ts:int = 0x14) -> Optional[bytes]:
        return self.cmd_ptd(device, 0, ts, 0x03, 0x02)

    def cmd_ptd_left(self, device:int, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device, ps, 0, 0x01, 0x03)

    def cmd_ptd_right(self, device:int, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device, ps, 0, 0x02, 0x03)

    def cmd_ptd_upleft(self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device,ps,ts,0x01,0x01)

    def cmd_ptd_upright(self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device,ps,ts,0x02,0x01)

    def cmd_ptd_downleft(self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device,ps,ts,0x01,0x02)

    def cmd_ptd_downright(self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        return self.cmd_ptd(device,ps,ts,0x02,0x02)

    def cmd_ptd_stop(self, device:int) -> Optional[bytes]:
        return self.cmd_ptd(device, 0, 0, 0x03, 0x03)

    def cmd_ptd_abs(self, device:int, ts:int = 0x14, ps:int = 0x18, pp:int = 0,
                    tp:int = 0) -> Optional[bytes]:
        '''
        Pan and Tilt Drive - tilt speed, pan speed, pan-pos, tilt-pos
        '''
        print(f"DEBUG: ABS POS TO {pp}/{tp}")

        # pp: range: -1440 - 1440
        if pp < 0:
            p = (((pp*-1)-1) ^ 0xffff)
        else:
            p = pp
        #tp: range -360 - 360
        if tp < 0:
            t = (((tp*-1)-1) ^ 0xffff)
        else:
            t = tp
        subcmd = bytes([0x02, ts, ps]) + self.i2v(p) + self.i2v(t)
        return self.cmd_pt(device, subcmd)

    def cmd_ptd_home(self, device:int) -> Optional[bytes]:
        return self.cmd_pt(device, b'\x04')

    def cmd_ptd_reset(self, device:int) -> Optional[bytes]:
        return self.cmd_pt(device, b'\x05')

#FIXME: Pan-tiltLimitSet
