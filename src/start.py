from argparse import ArgumentParser, Namespace
from logging import Logger
from multiprocessing import Process, current_process
#from multiprocessing.queue import Queue
from multiprocessing import Queue
import sys
from time import sleep, time
from typing import Any, Dict
Command = Dict[str, Any]

#import .driver

default_device = '/dev/ttyUSB0'

def driver_main(args:Namespace, log: Logger, req_queue:"Queue") -> int:
    '''
    Driver process main
    '''
    pid = current_process().pid
    log.info("pid=%d", pid)

    def dispatch(msg: Command) -> bool:
        '''
        Process a message received from the client
        '''
        log.debug("driver_dispatch: pid=%d, msg=%s", pid, str(msg))
        cmd:str = msg['cmd']

        return cmd != 'exit'

    try:
        while True:
            msg:Dict[str, Any] = req_queue.get(block=True, timeout=None)
            if dispatch(msg):
                continue
            log.info("pid=%d, main loop break", pid)
            break

    except ValueError as err:
        #queue is closed!
        log.info("pid=%d, caught ValueError: %s", pid, str(err))

    except KeyboardInterrupt:
        log.info("pid=%d, caught KeyboardInterrupt", pid)

    return 0

def client_main(args:Namespace, log: Logger, req_queue:"Queue") -> int:
    '''
    Client process main
    '''
    pid = current_process().pid
    pname = current_process().name
    log.info("pid=%d", pid)

    try:
        while True:
            sleep(3)
            msg = {
                'cmd':'ping',
                'src': pname,
            }
            req_queue.put(msg)
            log.info("pid=%d, main loop break", pid)

    except ValueError as err:
        #queue is closed!
        log.info("pid=%d, caught ValueError: %s", pid, str(err))

    except KeyboardInterrupt:
        log.info("pid=%d, caught KeyboardInterrupt", pid)

    return 0

def start_driver(args:Namespace, log: Logger, req_queue:"Queue") -> Process:
    driver = Process(
        target=driver_main, name='visca-driver', args=(args, log, req_queue, ))
    driver.start()
    log.info("started visca-driver, pid=%d", driver.pid)
    return driver

def start_client(name:str, args:Namespace, log: Logger, req_queue:"Queue") -> Process:
    client = Process(
        target=client_main, name=name, args=(args, log, req_queue, ))
    client.start()
    log.info("started %s, pid=%d", name, client.pid)
    return client

def main() -> int:
    '''
    Main entry
    '''
    #
    # create parser to handle command line
    #
    pid = current_process().pid
    current_process().name = 'visca-start'

    description = 'Creates processes for controlling VISCA camera'
    ap = ArgumentParser('visca-start', description=description)
    ap.add_argument(
        '-v', '--verbose', action='store_true', default=False,
        help='Tell more about what is going on')
    ap.add_argument(
        '-d', '--device', type=str, default=default_device,
        help=f"Serial VISCA device, defaults to '{default_device}'")
    #
    # parse the command line
    #
    args = ap.parse_args()
    from logger import get_logger
    log = get_logger()
    log.info("Start process pid=%d", pid)
    #
    # launch driver process
    #
    req_queue = Queue()
    driver = start_driver(args, log, req_queue)
    #
    # launch the clients
    #
    keys = start_client('visca-keyboard', args, log, req_queue)
    web = start_client('visca-web', args, log, req_queue)
    #
    # main loop
    #
    try:
        time_start = time()
        msg = {
            'cmd':'ping',
            'src': 'start',
        }
        while msg['cmd'] != 'exit':
            sleep(3)
            if (time() - time_start) > 20:
                msg['cmd'] = 'exit'
            req_queue.put(msg)
            log.info("pid=%d, req_queue.put(%s)", pid, str(msg))

    except ValueError as err:
        #queue is closed!
        log.info("pid=%d, caught ValueError: %s", pid, str(err))

    except KeyboardInterrupt:
        msg = {
            'cmd':'exit',
            'src': 'start',
        }
        req_queue.put(msg)
        log.info("pid=%d, caught KeyboardInterrupt", pid)

    #
    # shutdown the child processes
    #
    req_queue.close()
    driver.join()
    keys.join()
    web.join()

    return 0


if __name__ == '__main__':
    sys.exit(main())
