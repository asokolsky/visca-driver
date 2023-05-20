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

import logging
from multiprocessing import Process, current_process
from time import sleep
from typing import List, Tuple

from  visca import Visca, ViscaError
from  logger import create_logger

log = create_logger(
    None,  # log_file_path,
    logging.DEBUG)

def visca_initialize(v: Visca) -> None:
    print("v.cmd_address_set()")
    v.cmd_address_set()
    #sleep(3)
    print("v.cmd_if_clear_all()")
    v.cmd_if_clear_all()
    #sleep(3)
    return

def visca_memory_set(v: Visca, cam: int, postns: List[Tuple[int, int]]) -> None:
    '''
    Set memory 0..N according to element in pstns
    '''
    mem = 0
    for pp, tp in postns:
        v.cmd_ptd_abs(cam, pp=pp, tp=tp)
        sleep(3)
        v.cmd_cam_memory_set(cam, mem)
        sleep(3)
        mem += 1

    return

def main() -> None:
    p = current_process()
    p.name = 'visca_test'

    v = Visca(log, "/dev/ttyUSB0", True)
    visca_initialize(v)

    # device # of the webcam
    CAM = 1
    print("v.cmd_cam_power_on(CAM)")
    v.cmd_cam_power_on(CAM)
    v.wait_for_completion(30)

    #v.cmd_cam_auto_power_off(CAM, 2)
    #v.cmd_datascreen_on(CAM)
    #sleep(3)

    #print("v.inq_version(CAM)")
    #v.inq_version(CAM) - on my camera leads to a bad syntax response

    print("v.cmd_ptd_reset(CAM)")
    v.cmd_ptd_reset(CAM)

    print("v.cmd_ptd_home(CAM)")
    v.cmd_ptd_home(CAM)
    v.wait_for_completion(30)
    res = v.wait_for_pt_completion(CAM, 600)


    #v.wait_for_completion(30)

    #visca_memory_set(v, CAM, [
    #    (-1440, -360),
    #    (1440, 360),
    #    (0, 0),
    #])
    #sleep(5)
    #for mem in (0, 1, 2):
    #    v.cmd_cam_memory_recall(CAM, mem)
    #    sleep(3)


#	sleep(1)
#	v.cmd_cam_zoom_tele(CAM)
#	sleep(2)
#	v.cmd_cam_zoom_stop(CAM)
#	sleep(3)
#
#	v.cmd_cam_zoom_wide(CAM)
#	sleep(7)

#	v.cmd_cam_zoom_tele_speed(CAM, 0)
#	sleep(7)
#	v.cmd_cam_zoom_wide_speed(CAM, 0)
#	sleep(7)
#	v.cmd_cam_zoom_tele_speed(CAM, 7)
#	sleep(7)
#	v.cmd_cam_zoom_wide_speed(CAM, 7)
#	sleep(7)

# max optical zoom
#	v.cmd_cam_zoom_direct(CAM, 0x4000)
#	sleep(7)
# max digital zoom:
#	v.cmd_cam_zoom_direct(CAM, 0x7000)
#	sleep(7)
# no zoom
#	v.cmd_cam_zoom_direct(CAM, 0x0)

#	v.cmd_cam_dzoom_off(CAM)
#	v.cmd_cam_zoom_direct(CAM,0x7000)
#	sleep(7)
#	v.cmd_cam_wide_cinema(CAM)
#	sleep(7)
#	v.cmd_cam_wide_full(CAM)
#	sleep(7)
#	v.cmd_cam_wide_off(CAM)
#	sleep(7)

#	v.cmd_cam_lr_reverse_on(CAM)
#	sleep(2)
#	v.cmd_cam_lr_reverse_off(CAM)

#	v.cmd_cam_zoom_direct(CAM, 0x7000)
#	sleep(3)
#	v.cmd_cam_freeze_on(CAM)
#	sleep(2)

#	v.cmd_cam_zoom_direct(CAM, 0x0)
#	sleep(4)
#	v.cmd_cam_freeze_off(CAM)

#	v.cmd_cam_picture_effect_off(CAM)

    #
    # Try all the picture effects
    #
    #for i in range(0, 9):
    #    v.cmd_cam_picture_effect(CAM,i)
    #    sleep(3)
    #
    # Try all the digital effects for all the ranges
    #
    #for i in range(0,5):
    #    v.cmd_cam_digital_effect(CAM,i)
    #    for level in range(0,0x21):
    #        v.cmd_cam_digital_effect_level(CAM,level)
    #        sleep(2)

    #for ts in (0x14, 2):
    #    v.cmd_ptd_up(CAM, ts)
    #    sleep(3)
    #    v.cmd_ptd_down(CAM, ts)
    #    sleep(3)


    #print('inq_pt_status')
    #v.inq_pt_status(CAM)

    print('inq_pt_position')
    pos = v.inq_pt_position(CAM)
    #
    # swing from left to right
    #
    for ps in (0x2, 0x18):
        print('go left from', pos)
        v.cmd_ptd_left(CAM, ps)
        v.wait_for_completion(3)
        res = v.wait_for_pt_completion(CAM, 600)
        print('go left => ', res)
        if res is None:
            break
        _,pos = res
        print('go right from', pos)
        v.cmd_ptd_right(CAM, ps)
        v.wait_for_completion(30)
        res = v.wait_for_pt_completion(CAM, 600)
        print('go right => ', res)
        if res is None:
            break
        _,pos = res

    #
    # swing from up-left to down-right
    #
    #for ts, ps in ((0x14, 0x18), (2, 2)):
    #    print(f"v.cmd_ptd_upleft(CAM, {ts}, {ps})")
    #    v.cmd_ptd_upleft(CAM, ts, ps)
    #    sleep(2)
    #    print(f"v.cmd_ptd_downright(CAM, {ts}, {ps})")
    #    v.cmd_ptd_downright(CAM,  ts, ps)
    #    sleep(2)
    #
    # swing from up-right to down-left
    #
    #for ts, ps in ((0x14, 0x18), (2, 2)):
    #    print(f"v.cmd_ptd_upright(CAM, {ts}, {ps})")
    #    v.cmd_ptd_upright(CAM, ts, ps)
    #    sleep(2)
    #    print(f"v.cmd_ptd_downleft(CAM, {ts}, {ps})")
    #    v.cmd_ptd_downleft(CAM, ts, ps)
    #    sleep(2)

    #print("v.cmd_ptd_home(CAM)")
    #v.cmd_ptd_home(CAM)
    #v.wait_for_completion(30)

    print("v.cmd_ptd_reset(CAM)")
    v.cmd_ptd_reset(CAM)
    v.wait_for_completion(30)
    v.wait_for_pt_completion(CAM, 600)

    #v.cmd_cam_power_off(CAM)
    return


if __name__ == '__main__':
    try:
        main()

    except ViscaError as err:
        print('caught ViscaError:', err)

    except KeyboardInterrupt:
        print('caught KeyboardInterrupt')
