from argparse import ArgumentParser, Namespace
from multiprocessing import Process, Queue, current_process, freeze_support
import sys
from typing import Any, Dict

#import .driver

default_device = '/dev/ttyUSB0'

def driver_main(args:Namespace, req_queue:"Queue") -> int:
    pid = current_process()
    print("driver_main: current_process() => ", pid)
    while True:
        msg:Dict[str, Any] = req_queue.get()
    return 0

def main() -> int:
    #
    # create argparser to handle command line
    #
    description = 'Creates processes for controlling VISCA camera'
    ap = ArgumentParser('visca-start', description=description)
    ap.add_argument(
        '-v', '--verbose', action='store_true', default=False,
        help='Tell more about what is going on')
    ap.add_argument(
        '-d', '--device', type=str, default=default_device,
        help=f"Serial VISCA device, defaults to '{default_device}'")

    args = ap.parse_args()
    req_queue = Queue()

    driver = Process(target=driver_main, args=(args, req_queue, ))
    driver.start()

    try:
        while True:
            sys.sleep(1)

    except KeyboardInterrupt:
        pass

    # shutdown the child processes
    driver.join()

    return 0


if __name__ == '__main__':
    sys.exit(main())
