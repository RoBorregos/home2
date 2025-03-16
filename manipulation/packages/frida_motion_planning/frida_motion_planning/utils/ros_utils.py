def wait_for_future(self, future):
    if future is None:
        return False
    while not future.done():
        pass
    return future
