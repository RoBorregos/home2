import time


def wait_for_future(future, timeout=5):
    """
    Wait for a future to complete with a timeout.
    """
    start_time = time.time()
    while future is None and (time.time() - start_time) < timeout:
        pass
    if future is None:
        return False
    while not future.done() and (time.time() - start_time) < timeout:
        # print("Waiting for future to complete...")
        pass

    return future
