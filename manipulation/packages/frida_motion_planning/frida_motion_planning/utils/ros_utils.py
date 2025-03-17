def wait_for_future(future):
    if future is None:
        return False
    while not future.done():
        pass
    return future
