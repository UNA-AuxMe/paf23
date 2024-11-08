#!/usr/bin/env python3
"""
This is a dedicated debug logger node

Without a fully initialized node, log messages only show up in the logfile.
But not in the /rosout topic and thus not in the rqt_console

This is why this node is used by the debug_wrapper for logging
before it has initialized its own node
"""

import time
import sys
from multiprocessing.connection import Listener, Client
import threading
import signal

import rospy

ADDRESS = ("localhost", 52999)
AUTHKEY = b"debug_logger"
CLOSE_MSG = "close"
LISTENER_THREAD = None
NODE_RUNNING = True

MESSAGES_MUTEX = threading.Lock()
MESSAGES = []


def eprint(msg: str):
    print(f"[debug_logger_node]: {msg}", file=sys.stderr)


def log(name: str, msg: str, level: str):
    msg = f"[debug_logger for {name}]: {msg}"

    level = level.lower()
    if level == "debug":
        rospy.logdebug(msg)
    elif level == "info":
        rospy.loginfo(msg)
    elif level == "warn":
        rospy.logwarn(msg)
    elif level == "error":
        rospy.logerr(msg)
    else:
        rospy.logfatal(msg)


def run_listener(listener: Listener):
    running = True
    while running:
        conn = listener.accept()
        print(f"[debug_logger]: connection accepted from {listener.last_accepted}")
        msg = None
        if conn.poll(timeout=2.0):
            msg = conn.recv()
            if isinstance(msg, str):
                if msg.lower() == CLOSE_MSG:
                    running = False
                    msg = None
        conn.close()
        if msg is not None:
            with MESSAGES_MUTEX:
                MESSAGES.append(msg)
    listener.close()


def close_listener():
    try:
        conn = Client(ADDRESS, authkey=AUTHKEY)
        conn.send(CLOSE_MSG)
        conn.close()
    except BaseException:
        pass


def exit_cleanup(signum=None, frame=None):
    close_listener()
    if LISTENER_THREAD is not None:
        LISTENER_THREAD.join()
    global NODE_RUNNING
    NODE_RUNNING = False


def main():
    try:
        # Based on
        # https://stackoverflow.com/questions/6920858/interprocess-communication-in-python
        # Only one listener can be active at once
        listener = Listener(ADDRESS, authkey=AUTHKEY)
        eprint("Debug logger node started")
    except OSError as error:
        eprint(f"Failed to run listener: {error}. Exiting...")
        exit(0)
    signal.signal(signal.SIGINT, exit_cleanup)
    signal.signal(signal.SIGTERM, exit_cleanup)
    global LISTENER_THREAD
    LISTENER_THREAD = threading.Thread(target=run_listener, args=(listener,))
    LISTENER_THREAD.start()

    rospy.init_node(name="debug_logger")
    # We need to wait a bit until the node is fully initialized to send log messages
    time.sleep(5.0)
    rospy.on_shutdown(exit_cleanup)

    while NODE_RUNNING:
        with MESSAGES_MUTEX:
            while len(MESSAGES) > 0:
                msg = MESSAGES.pop()
                log_msg = "Wrong log message format"
                log_name = "NAMERR"
                log_level = "fatal"
                if isinstance(msg, dict):
                    if "name" in msg:
                        log_name = msg["name"]
                    if "msg" in msg:
                        log_msg = msg["msg"]
                    if "level" in msg:
                        log_level = msg["level"]
                log(log_name, log_msg, log_level)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
