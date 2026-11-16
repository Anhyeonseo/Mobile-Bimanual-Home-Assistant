"""Keep the sole serial owner running while independent file work completes."""
import time


def await_serviced(future, service, *, sleep=time.sleep, interval_s=.01):
    while not future.done():
        service()
        sleep(interval_s)
    # Recheck real measurements after I/O, including on an already-done future.
    service()
    return future.result()
