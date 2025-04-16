import time


def wait_for_future(future, timeout=1):
    start_time = time.time()
    print("waiting for future not none")
    while future is None and (time.time() - start_time) < timeout:
        pass
    if future is None:
        print("timeout reached")
        return False
    while not future.done():
        pass
    return future
