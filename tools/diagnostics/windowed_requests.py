"""Bounded asynchronous read-only requests; drain before a caller changes scene."""
import math
from time import monotonic


def windowed_requests(items, submit, pump, *, window=8, timeout_s=12., clock=monotonic):
    """Return all results in input order, regardless of response completion order.

    Each request has its own deadline. On any transport error, cancel remaining
    futures and raise: callers must abort the session, not mutate/reuse its scene.
    Cancellation of a local future does not cancel the remote service callback.
    """
    if type(window) is not int or not 1 <= window <= 32:
        raise ValueError('window must be an integer in 1..32')
    if not math.isfinite(timeout_s) or timeout_s <= 0:
        raise ValueError('positive finite timeout required')
    items = list(items)
    results = [None]*len(items)
    pending = {}
    next_index = 0
    try:
        while next_index < len(items) or pending:
            while next_index < len(items) and len(pending) < window:
                pending[next_index] = (submit(items[next_index]), clock()+timeout_s)
                next_index += 1
            completed = []
            for index, (future, deadline) in pending.items():
                if future.done():
                    if future.cancelled():
                        raise RuntimeError('state check request cancelled')
                    result = future.result()
                    if result is None:
                        raise RuntimeError('state check returned no response')
                    results[index] = result
                    completed.append(index)
                elif clock() >= deadline:
                    raise TimeoutError('state check timed out at index '+str(index))
            for index in completed:
                del pending[index]
            if pending and not completed:
                pump()
    except BaseException:
        for future, _ in pending.values():
            if not future.done():
                future.cancel()
        raise
    return results
