import time


def wait_for_future(future, timeout=5):
    """
    Wait for a future to complete with a timeout.
    """
    start_time = time.time()
    while future is None and (time.time() - start_time) < timeout:
        time.sleep(0.01)
    if future is None:
        return False
    while not future.done() and (time.time() - start_time) < timeout:
        time.sleep(0.01)

    return future
