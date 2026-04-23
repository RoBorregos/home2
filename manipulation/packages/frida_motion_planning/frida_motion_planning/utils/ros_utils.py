import time


def wait_for_future(future, timeout=60):
    # time.sleep yields so other executor threads can make progress; a bare
    # busy-loop used to peg an executor worker and starve the DDS event loop
    # while the node was waiting on service/action responses.
    start_time = time.time()
    while future is None and (time.time() - start_time) < timeout:
        time.sleep(0.001)
    if future is None:
        print("timeout reached")
        return False
    while not future.done() and (time.time() - start_time) < timeout:
        time.sleep(0.001)
    if not future.done():
        print("timeout reached")
        return False
    return future
