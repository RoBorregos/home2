import time


def wait_for_future(future, timeout=60):
    start_time = time.time()
    while future is None and (time.time() - start_time) < timeout:
        pass
    if future is None:
        print("timeout reached")
        return False
    while not future.done() and (time.time() - start_time) < timeout:
        pass
    if not future.done():
        print("timeout reached")
        return False
    return future
