import time


def wait_for_future(future, timeout=60):
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
