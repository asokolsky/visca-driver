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
'''

from logging import Logger
from serial import Serial, SerialException
from threading import Lock
from time import sleep, time
from typing import Optional, Tuple

class ViscaError(IOError):
    """Base class for Visca errors."""
    pass

class ViscaTimeout(ViscaError):
    """Timeout for wait_xxx function"""
    pass

class Visca():

    @staticmethod
    def i2v(value:int) -> bytes:
        '''
        return value as 4-byte dword in Visca format
        packets are not allowed to be 0xff
        so for numbers the first nibble is 0000
        and 0xfd gets encoded into 0x0f 0x0xd
        '''
        ms = (value & 0b1111111100000000) >> 8
        ls = (value & 0b0000000011111111)
        p = (ms & 0b11110000) >> 4
        r = (ls & 0b11110000) >> 4
        q = ms & 0b1111
        s = ls & 0b1111
        return bytes([p, q, r, s])

    @staticmethod
    def v2i(value:bytes, start:int) -> int:
        '''
        0p 0p 0p 0p
        '''
        a = value[start+0]
        b = value[start+1]
        c = value[start+2]
        d = value[start+3]
        return (a*0x1000) + (b*0x100) + (c*0x10) + d


    def __init__(self, log:Logger, port:str = "/dev/ttyUSB0",
                 bVerbose:bool = True) -> None:
        # guards access to self.serialport
        self.lock = Lock()
        self.log = log
        self.bVerbose = bVerbose
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
            self.log.error("SerialException opening port '%s': %s",
                self.portname, str(e))
            #raise e
            raise ViscaError(e.strerror)

        except Exception as e:
            self.log.error("Exception opening port '%s': %s",
                self.portname, str(e))
            #raise e

        finally:
            self.lock.release()
        return


    def dump(self, packet:bytes, title:Optional[str] = None) -> None:
        '''
        Dump the packet into the log
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

        if title is not None:
            line = f"{title} "
        else:
            line = "packet "
        line += f"[{sender} => {recipient_s}] len={len(packet)}: {packet.hex(' ')}"
        self.log.debug(line)

        line = f" QQ.........: {qq:02x}"
        if qq == 0x01:
            line += " (Command)"
        if qq == 0x09:
            line += " (Inquiry)"
        self.log.debug(line)

        if len(packet) > 3:
            rr = packet[2]
            line = f" RR.........: {rr:02x}"
            if rr == 0x00:
                line += " (Interface)"
            if rr == 0x04:
                line += " (Camera [1])"
            if rr == 0x06:
                line += " (Pan/Tilt)"
            self.log.debug(line)

        if len(packet) > 4:
            data = packet[3:-1]
            self.log.debug(f" Data.......: {data.hex(' ')}")

        if term != b'\xff':
            self.log.error("Packet not terminated correctly")
            return

        if len(packet) == 3 and ((qq & 0b11110000) >> 4) == 4:
            socketno = (qq & 0b1111)
            self.log.debug(" ACK for socket %02x", socketno)

        if len(packet) == 3 and ((qq & 0b11110000) >> 4) == 5:
            # 90-5Y-FF Returned by the camera when execution of commands and
            # inquiries are completed.
            socketno = (qq & 0b1111)
            self.log.debug(" COMPLETION for socket %02x", socketno)

        if len(packet) > 3 and ((qq & 0b11110000) >> 4) == 5:
            socketno = (qq & 0b1111)
            ret = packet[2:-1].hex(' ')
            self.log.debug(" COMPLETION for socket %02x, data=%s",
                           socketno, ret)

        if len(packet) == 3 and qq == 0x38:
            self.log.debug(
                "Network Change - we should immediately issue a renumbering!")

        if len(packet) == 4 and ((qq & 0b11110000) >> 4) == 6:
            # 90-6Y-..FF Returned by camera instead of a completion message
            # when command or inquiry failed to be executed.

            self.log.error(" ERROR!")

            socketno = (qq & 0b00001111)
            errcode = packet[2]

            # 90-6Y-01-FF Message length error (>14 bytes)
            if errcode == 0x01:
                self.log.error("        : message length error")

            #these two are special, socket is zero and has no meaning:
            if errcode == 0x02 and socketno == 0:
                self.log.error("        : syntax error")

            if errcode == 0x03 and socketno == 0:
                self.log.error("        : command buffer full")

            if errcode == 0x04:
                self.log.info(
                    f"        : Socket {socketno}: command canceled")

            if errcode == 0x05:
                self.log.debug(
                    f"        : Socket {socketno}: invalid socket selected")

            if errcode == 0x41:
                self.log.debug(
                    f"        : Socket {socketno}: command not executable")

        return

    def _recv_packet(self, extra_title: Optional[str] = None) -> bytes:
        '''
        Receive a packet - 3 to 16 bytes until 0xff
        will block
        '''
        assert self.serialport is not None
        packet = bytearray()
        count = 0
        # packet can be anywhere between 3 and 16 bytes
        while count < 16:
            s = self.serialport.read(1)
            if s:
                packet.extend(s)
                count += 1
            else:
                self.log.warning("Timeout waiting for a reply")
                break
            if s == b'\xff':
                break

        if self.bVerbose:
            if extra_title:
                self.dump(packet, f"Rx: {extra_title}")
            else:
                self.dump(packet, "Rx")
        return packet

    def wait_for_completion(self, secs:float) -> Optional[bytes]:
        '''
        Wait for the completion message for upto secs.
        Returns a packet or None in case of timeout
        '''
        if self.serialport is None:
            return None
        try:
            self.lock.acquire()

            start = time()
            done = start + secs
            while time() < done:
                if self.serialport.in_waiting:
                    ms = int((time() - start)*1000)
                    return self._recv_packet(f"after {ms}ms")
                sleep(1)

        except SerialException as e:
            self.log.error("SerialException reading from the port '%s': %s",
                self.portname, e)

        finally:
            self.lock.release()

        raise ViscaTimeout(f"Reached wait_for_completion timeout of {secs}")

    def _write_packet(self, packet:bytes) -> None:
        '''
        Write packet to self.serialport
        Will pick the message from the port if the one is available
        '''
        assert self.serialport is not None
        if not self.serialport.is_open:
            msg = "trying to write to a closed port"
            self.log.error(msg)
            raise ViscaError(msg)

        # lets see if a completion message or something
        # else waits in the buffer. If yes dump it.
        if self.serialport.in_waiting:
            self._recv_packet("ignored")

        self.serialport.write(packet)
        self.dump(packet, "Tx")
        return

    def send_packet(self, recipient:int, data:bytes) -> Optional[bytes]:
        """
        Will add terminator '\xff' to the data

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
            reply = self._recv_packet()

        except SerialException as e:
            self.log.error("SerialException writing to the port '%s': %s",
                self.portname, e)

        finally:
            self.lock.release()

        if not reply:
            return None

        if reply[-1:] != b'\xff':
            self.log.error(
                f"received packet not terminated correctly: {reply.hex(' ')}")
            return None

        return reply

    def send_broadcast(self, data: bytes) -> Optional[bytes]:
        # shortcut
        return self.send_packet(-1, data)

    def cmd_address_set(self) -> None:
        '''
        starts enumerating devices, sends the first address to use on the bus
        reply is the same packet with the next free address to use
        '''

        #address of first device. should be 1:
        first = 1
        # set address
        reply = self.send_broadcast(bytes([0x30, first]))
        if not reply:
            msg = "No reply from the bus."
            self.log.error(msg)
            raise ViscaError(msg)

        if len(reply) != 4 or reply[-1:] != b'\xff':
            msg = "ERROR enumerating devices"
            self.log.error(msg)
            raise ViscaError(msg)

        if reply[0] != 0x88:
            msg = "ERROR: expecting broadcast answer to an enumeration request"
            self.log.error(msg)
            raise ViscaError(msg)

        address = reply[2]
        d = address - first
        msg = f"Found {d} devices on the bus"
        if d == 0:
            self.log.error(msg)
            raise ViscaError(msg)
        else:
            self.log.debug(msg)
        return

    def cmd_if_clear_all(self) -> None:
        '''
        Clears the command buffer in the unit. When cleared, the operation
        currently being executed is not guaranteed.
        '''
        reply = self.send_broadcast(b'\x01\x00\x01')
        if reply is None:
            msg = "ERROR sending broadcast"
            self.log.error(msg)
            raise ViscaError(msg)

        if not reply[1:] == b'\x01\x00\x01\xff':
            msg = "ERROR clearing all interfaces on the bus!"
            self.log.error(msg)
            raise ViscaError(msg)

        self.log.debug("all interfaces clear")
        return

    def _cmd_cam(self, device:int, data:bytes) -> Optional[bytes]:
        '''
        Send the command - starts with 8x 01 04
        Will terminate with FF
        Returns reply
        '''
        reply = self.send_packet(device, b'\x01\x04' + data)
        #FIXME: check returned data here and retransmit?
        return reply

    def _inq_cam(self, device:int, data:bytes) -> Optional[bytes]:
        '''
        Send the command - starts with 8x 09
        Will terminate with FF
        Returns reply
        '''
        reply = self.send_packet(device, b'\x09' + data)
        #FIXME: check returned data here and retransmit?
        return reply

    #
    # POWER control.
    #
    def cmd_cam_power(self, device:int, onoff:bool) -> Optional[bytes]:
        '''
        Send 8x 01 04 00 0p FF
        where p: 2=On, 3=Standby
        '''
        if onoff:
            pwr = b'\x00\x02'
        else:
            pwr = b'\x00\x03'
        return self._cmd_cam(device, pwr)

    def cmd_cam_power_on(self, device:int) -> Optional[bytes]:
        '''
        Power on the motors
        '''
        return self.cmd_cam_power(device, True)

    def cmd_cam_power_off(self, device:int) -> Optional[bytes]:
        '''
        This command stores the zoom and focus value and resets the motors.
        These commands do not power the camera on or off.
        '''
        return self.cmd_cam_power(device, False)

    def cmd_cam_auto_power_off(
            self, device:int, time:int = 0) -> Optional[bytes]:
        '''
        time = minutes without command until standby
        0: disable
        0xffff: 65535 minutes
        '''
        return self._cmd_cam(device, b'\x40' + self.i2v(time))

    #
    # ZOOM control
    #
    def cmd_cam_zoom_stop(self, device:int) -> Optional[bytes]:
        '''
        Zoom_Stop 8x 01 04 07 00 ff
        '''
        return self._cmd_cam(device, b'\x07\x00')

    def cmd_cam_zoom_tele(self, device:int) -> Optional[bytes]:
        return self._cmd_cam(device, b'\x07\x02')

    def cmd_cam_zoom_wide(self, device:int) -> Optional[bytes]:
        return self._cmd_cam(device, b'\x07\x03')

    def cmd_cam_zoom_tele_speed(self, device:int, speed:int) -> Optional[bytes]:
        '''
        Zoom_Tele 8x 01 04 07 2p ff, p = speed parameter, a (low) to b (high)
        zoom in with speed = 0..7
        '''
        sbyte = 0x20 + (speed & 0b111)
        return self._cmd_cam(device, bytes([0x07, sbyte]))

    def cmd_cam_zoom_wide_speed(self, device:int, speed:int) -> Optional[bytes]:
        '''
        Zoom_Wide 8x 01 04 07 3p ff, p = speed parameter, a (low) to b (high)
        zoom in with speed = 0..7
        '''
        sbyte = 0x30+(speed & 0b111)
        return self._cmd_cam(device, bytes([0x07, sbyte]))

    def cmd_cam_zoom_direct(self, device:int, zoom:int) -> Optional[bytes]:
        '''
        Zoom_Direct 8x 01 04 47 0p 0q 0r 0s ff pqrs: zoom position
        optical: 0..4000
        digital: 4000..7000 (1x - 4x)
        '''
        return self._cmd_cam(device, b'\x47' + self.i2v(zoom))

    def cmd_cam_dzoom(self, device:int, state:bool) -> Optional[bytes]:
        '''
        Digital Zoom control on/off
        '''
        if state:
            subcmd = b'\x06\x02'
        else:
            subcmd = b'\x06\x03'
        return self._cmd_cam(device, subcmd)

    def cmd_cam_dzoom_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_dzoom(device, True)

    def cmd_cam_dzoom_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_dzoom(device, False)
# FIXME:
# ZoomFocus_Direct 8x 01 04 47 0p 0q 0r 0s 0t 0u 0v 0w ff
#   pqrs: zoom position
#   tuvw: focus position
#
# FIXME: CAM_FOCUS COMMANDS
# Focus_Stop 8x 01 04 08 00 ff
# Focus_Far  8x 01 04 08 2p ff   p = speed parameter, a (low) to b (high)
# Focus_Near 8x 01 04 08 3p ff   p = speed parameter, a (low) to b (high)
# Focus_Direct 8x 01 04 48 0p 0q 0r 0s ff pqrs: focus position
# Focus_Auto 8x 01 04 38 02 ff              Autofocus mode on/off.
# NOTE: If the mode is on auto, camera may disable autofocus when focus is ok.
# Autofocus is turned back on when camera is moved using Zoom_Tele/Wide,
# PT_Up/Down/Left/Right.  also applies for IR_CameraControl movement.
# Focus_Manual 8x 01 04 38 03 ff

#
# FIXME: CAM_WB
# WB_Auto           8x 01 04 35 00 ff WB: White Balance
# WB_Table_Manual   8x 01 04 35 06 ff
# WB_Table_Direct   8x 01 04 75 0p 0q 0r 0s ff
#                                ^pqrs = wb table.
# Used if Wbmode=Table manual.
# If Wbmode is not Table manual, the table index is stored and used next time
# Table manual mode is entered.
#
# FIXME: CAM_?GAIN
# FIXME: CAM_AE
# AE_Auto 8x 01 04 39 00 ff AE: Automatic Exposure.
# AE_Manual 8x 01 04 39 03 ff
#
# FIXME: CAM_SlowShutter
# FIXME: CAM_Shutter
# FIXME: CAM_Iris
# Iris_Direct 8x 01 04 4B 0p 0q 0r 0s ff Used if AE mode = Manual.
# pqrs: Iris position, range 0..50
#
# FIXME: CAM_Gain
# Gain_Direct 8x 01 04 4c 0p 0q 0r 0s ff Used if AE mode = Manual.
#  pqrs: Gain position, values: 12-21dB
#
# FIXME: CAM_Bright
# FIXME: CAM_ExpComp
#
# FIXME: CAM_BackLight
# Backlight_On 8x 01 04 33 02 ff BacklightCompensation mode
# Backlight_Off 8x 01 04 33 03 ff
#
# FIXME: CAM_Aperature

    def cmd_cam_wide(self, device:int, mode:int) -> Optional[bytes]:
        '''
        16:9 / Wide format:
        '''
        return self._cmd_cam(device, bytes([0x60, mode]))

    def cmd_cam_wide_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x00)

    def cmd_cam_wide_cinema(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x01)

    def cmd_cam_wide_full(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_wide(device, 0x02)

    #
    # mirror
    #
    def _cmd_cam_lr_reverse(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x61, mode])
        return self._cmd_cam(device, subcmd)

    def cmd_cam_lr_reverse_on(self, device:int) -> Optional[bytes]:
        '''
        Mirror ON 8x 01 04 61 02 ff
        '''
        return self._cmd_cam_lr_reverse(device, 0x02)

    def cmd_cam_lr_reverse_off(self, device:int) -> Optional[bytes]:
        '''
        Mirror OFF 8x 01 04 61 03 ff
        '''
        return self._cmd_cam_lr_reverse(device, 0x03)

    #
    # FIXME:
    # Flip_On 8x 01 04 66 02 ff
    # Flip_Off 8x 01 04 66 03 ff
    # Sony calls this CAM_ImgFlip.
    # The “xConfiguration Cameras Camera [1..n] Flip: Auto
    #
    # FIXME:
    # Gamma_Auto 8x 01 04 51 02 ff  Gamma mode. Default uses gamma table 4
    # Gamma_Manual 8x 01 04 51 03 ff
    # Gamma_Direct 8x 01 04 52 0p 0q 0r 0s ff
    #                       pqrs: Gamma table to use in manual mode. Range 0-7
    #

    # freeze
    def cmd_cam_freeze(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x62, mode])
        return self._cmd_cam(device, subcmd)

    def cmd_cam_freeze_on(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_freeze(device, 0x02)

    def cmd_cam_freeze_off(self, device:int) -> Optional[bytes]:
        return self.cmd_cam_freeze(device, 0x03)

    # Picture Effects
    def cmd_cam_picture_effect(self, device:int, mode:int) -> Optional[bytes]:
        subcmd = bytes([0x63, mode])
        return self._cmd_cam(device, subcmd)

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
        return self._cmd_cam(device, bytes([0x64, mode]))

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

    def cmd_cam_digital_effect_level(
            self, device:int, level:int) -> Optional[bytes]:
        subcmd = bytes([0x65, 0b00111111 & level])
        return self._cmd_cam(device, subcmd)

    def cmd_cam_memory(self, device:int, func:int, num:int) -> Optional[bytes]:
        '''
        memory of settings including position
        '''
        if num > 5:
            num = 5
        if func < 0 or func > 2:
            self.log.warning(f"cmd_cam_memory bad func={func}")
            return None

        self.log.debug("cam_memory command")
        subcmd = bytes([0x3f, func, (0b0111 & num)])
        return self._cmd_cam(device, subcmd)

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
        return self._cmd_pt(device, bytes([0x06, func]))

    def cmd_datascreen_on(self, device:int) -> Optional[bytes]:
        '''
        '''
        return self.cmd_datascreen(device, 0x02)

    def cmd_datascreen_off(self, device:int) -> Optional[bytes]:
        return self.cmd_datascreen(device, 0x03)

    def cmd_datascreen_toggle(self, device:int) -> Optional[bytes]:
        return self.cmd_datascreen(device, 0x10)

#FIXME: IR_Receive
#FIXME: IR_Receive_Return
# FIXME
# IR_Output_On  8x 01 06 08 02 ff
# IR_Output_Off 8x 01 06 08 03 ff
# see IR_push message
#
# IR_CameraControl_On 8x 01 06 09 02 ff
# IR_CameraControl_Off 8x 01 06 09 03 ff
# Lets the up/down/left/right/zoom+/- on the IR remote control the camera
# directly. Those keycodes are sent to the controller if the IR Output is on.
#

    def _cmd_pt(self, device:int, subcmd:bytes) -> Optional[bytes]:
        '''
        Pan/Tilt command starting with 01 06
        '''
        reply = self.send_packet(device, b'\x01\x06' + subcmd)
        #FIXME: check returned data here and retransmit?
        return reply

    def cmd_ptd_home(self, device:int) -> Optional[bytes]:
        return self._cmd_pt(device, b'\x04')

    def cmd_ptd_reset(self, device:int) -> Optional[bytes]:
        '''
        PT_Reset 8x 01 06 05 ff Reset pan/tilt to center position.
        This also re–synchronizes the motors
        '''
        return self._cmd_pt(device, b'\x05')

    def _cmd_ptd(
            self, device:int, ps:int, ts:int, lr:int, ud:int) -> Optional[bytes]:
        '''
        Pan and Tilt Drive - pan speed, tilt speed, left/right, up/down
        '''
        return self._cmd_pt(device, bytes([0x01, ps, ts, lr, ud]))

    def cmd_ptd_up(self, device:int, ts:int = 0x14) -> Optional[bytes]:
        '''
        PT_Up 8x 01 06 01 0p 0t 03 01 ff  p-pan speed, t-tilt speed
        Up -> increment tilt
        Down -> decrement tilt
        '''
        return self._cmd_ptd(device, 0, ts, 0x03, 0x01)

    def cmd_ptd_down(self, device:int, ts:int = 0x14) -> Optional[bytes]:
        '''
        PT_Down 8x 01 06 01 0p 0t 03 02 ff  p-pan speed, t-tilt speed
        Up -> increment tilt
        Down -> decrement tilt
        '''
        return self._cmd_ptd(device, 0, ts, 0x03, 0x02)

    def cmd_ptd_left(self, device:int, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_Left 8x 01 06 01 0p 0t 01 03 ff  p-pan speed, t-tilt speed
        Right -> increment pan
        Left -> decrement pan
        '''
        return self._cmd_ptd(device, ps, 0, 0x01, 0x03)

    def cmd_ptd_right(self, device:int, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_Right 8x 01 06 01 0p 0t 02 03 ff  p-pan speed, t-tilt speed
        Right -> increment pan
        Left -> decrement pan
        '''
        return self._cmd_ptd(device, ps, 0, 0x02, 0x03)

    def cmd_ptd_upleft(
            self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_UpLeft 8x 01 06 01 0p 0t 01 01 ff  p-pan speed, t-tilt speed
        '''
        return self._cmd_ptd(device, ps, ts, 0x01, 0x01)

    def cmd_ptd_upright(
            self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_UpRight 8x 01 06 01 0p 0t 02 01 ff  p-pan speed, t-tilt speed
        '''
        return self._cmd_ptd(device,ps,ts,0x02,0x01)

    def cmd_ptd_downleft(
            self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_DownLeft 8x 01 06 01 0p 0t 01 02 ff  p-pan speed, t-tilt speed
        '''
        return self._cmd_ptd(device,ps,ts,0x01,0x02)

    def cmd_ptd_downright(
            self, device:int, ts:int = 0x14, ps:int = 0x18) -> Optional[bytes]:
        '''
        PT_DownRight 8x 01 06 01 0p 0t 02 02 ff  p-pan speed, t-tilt speed
        '''
        return self._cmd_ptd(device, ps, ts, 0x02, 0x02)

    def cmd_ptd_stop(self, device:int) -> Optional[bytes]:
        '''
        PT_Stop 8x 01 06 01 03 03 03 03 ff
        '''
        return self._cmd_ptd(device, 0x03, 0x03, 0x03, 0x03)

    def cmd_ptd_abs(self, device:int, ts:int = 0x14, ps:int = 0x18, pp:int = 0,
                    tp:int = 0) -> Optional[bytes]:
        '''
        Pan and Tilt Drive - tilt speed, pan speed, pan-pos, tilt-pos
        PT_Direct 8x 01 06 02 0p 0t 0q 0r 0s 0u 0v 0w 0x 0y ff
            p: max pan speed
            t: max tilt speed
            qrsu: pan position
            vwxy: tilt position
            Attempts to linearize movement
        '''
        self.log.debug(f"DEBUG: ABS POS TO {pp}/{tp}")

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
        return self._cmd_pt(device, subcmd)

    # FIXME
    # PTZF_Direct 8x 01 06 20 0p 0q 0r 0s 0t 0u 0v 0w 0x 0y 0z 0g 0h 0i 0j 0k ff
    # Sets all motors in one operation.
    # pqrs: pan
    # tuvw: tilt
    # xyzg: zoom
    # hijk: focus
    # Attempts to linearize movement for pan and tilt.
    # The focus value will not be used if the camera is in continuous autofocus
    # mode.
    # NOTE: Never route this message through Sony cameras.

    # FIXME:
    # PT_Limit_Set * 8x 01 06 07 00 0x 0p 0q 0r 0s 0t 0u 0v 0w ff
    # x=1: Up/Right
    # x=0: Down/Left
    # pqrs: Pan limit
    # tuvx: Tilt limit.
    # This command is valid until the next time the camera boots.

    # FIXME
    # PT_Limit_Clear * 8x 01 06 07 01 0x [...] ff x=1: Up/Right
    # x=0: Down/Left
    # Sony specifies several filler bytes after 0x. These can be ignored

    #
    # FIXME:
    # MM_Detect_On 8x 01 50 30 01 ff
    # Turn on the Motor Moved Detection.
    # The camera recalibrates if touched.
    # MM_Detect_Off 8x 01 50 30 00 ff
    # Turn off the Motor Moved Detection.
    # The camera does not recalibrate if touched.
    #
    # FIXME
    # Call_LED_On    8x 01 33 01 01 ff
    # Call_LED_Off   8x 01 33 01 00 ff
    # Call_LED_Blink 8x 01 33 01 02 ff
    # Refers to the LED on top of the camera. It is always off on startup.
    #
    # FIXME
    # Power_LED_On 8x 01 33 02 01 ff
    # Power_LED_Off 8x 01 33 02 00 ff
    # Green power LED. If switched to off and stored to startup profile,
    # it is always off.

    def inq_version(self, device:int) -> Optional[bytes]:
        '''
        CAM_VersionInq
        Returns information on the VISCA interface.
        Inquiry Packet: CAM_VersionInq: 8X 09 00 02 FF
        Reply Packet: Y0 50 GG GG HH HH JJ JJ KK FF
        X = 1 to 7: Address of the unit (Locked to “X = 1” for VISCA over IP)
        Y = 9 to F: Address of the unit +8 (Locked to “Y = 9” for VISCA over IP)
        GGGG = Vender ID
           0001: Sony
        HHHH = Model ID
           051C:BRC-X400
           051D:BRC-X401
           0617:SRG-X400
           0618:SRG-X120
           061A:SRG-201M2
           061B:SRG-HD1M2
        JJJJ = ROM revision
        KK = Maximum socket # (02)
        '''
        res = self._inq_cam(device, b'\x00\x02')

        return res

    def inq_pt_position(self, device:int) -> Optional[Tuple[int, int]]:
        '''
        8X 09 06 12 FF
        Y0 50 0p 0p 0p 0p 0t 0t 0t 0t FF
        Returns (pan, tilt)
        Supported by Monoprice-39512!
        '''
        # mitigate f/w bug
        max_retries = 3
        res = self._inq_cam(device, b'\x06\x12')
        for _ in range(max_retries):
            if res is None:
                return None
            if len(res) > 3:
                break
            # COMPLETION is NOT a valid response to Inquiry
            # Monoprice-39512 f/w bug
            res = self._inq_cam(device, b'\x06\x12')

        assert res is not None
        pan = self.v2i(res, 2)
        tilt = self.v2i(res, 6)
        self.log.debug(f'inq_pt_position({device:02x}) => {pan:04x}, {tilt:04x}')
        return (pan, tilt)

    def wait_for_pt_completion(self, device:int, secs:float) -> Tuple[
            Optional[Tuple[int, int]], Optional[Tuple[int, int]]]:
        '''
        Wait until the Pan/Tilt completes or timeout expires.
        Waits at least 1s.
        Returns a tuple ((pan0,tilt0), (pan1,tilt1))
        '''
        res = self.inq_pt_position(device)
        if res is None:
            return (None, None)
        pan0, tilt0 = res
        pan1, tilt1 = res
        done = time() + secs
        while time() < done:
            sleep(1)
            res = self.inq_pt_position(device)
            if res is None:
                fres1 = (pan0, tilt0), None
                self.log.debug("wait_for_pt_completion(%d) => %s", device, fres1)
                return fres1
            pan, tilt = res
            if pan != pan1 or tilt != tilt1:
                pan1,tilt1 = pan,tilt
            else:
                fres2 = (pan0, tilt0), (pan1, tilt1)
                self.log.debug("wait_for_pt_completion(%d) => %s", device, fres2)
                return fres2

        # timeout
        raise ViscaTimeout(f"Reached wait_for_pt_completion timeout of {secs}")
        #fres3 = (pan0, tilt0), (pan1, tilt1)
        #self.log.debug(
        #    "wait_for_pt_completion(%d) => %s, timeout", device, fres3)
        #return fres3

    def inq_pt_status(self, device:int) -> Optional[bytes]:
        '''
        8X 09 06 10 FF
        Y0 50 pp pp FF
        Ignored by Monoprice-39512
        '''
        self._inq_cam(device, b'\x06\x10')
        # res is None because ACK is not returned
        return self.wait_for_completion(5)
