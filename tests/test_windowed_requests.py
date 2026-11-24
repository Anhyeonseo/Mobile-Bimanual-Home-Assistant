from concurrent.futures import Future
import pytest

from tools.diagnostics.windowed_requests import windowed_requests


class FakeTransport:
    def __init__(self, delays, fail=None):
        self.delays, self.fail = delays, fail
        self.now = 0.
        self.pending = []
        self.futures = []
        self.maximum = 0
        self.completed = []

    def submit(self, index):
        f = Future()
        self.pending.append((index, f, self.now+self.delays[index]))
        self.futures.append(f)
        self.maximum = max(self.maximum, sum(not x.done() for x in self.futures))
        return f

    def pump(self):
        self.now += .1
        for i, f, deadline in self.pending:
            if not f.done() and self.now >= deadline:
                if i == self.fail: f.set_exception(RuntimeError('transport failure'))
                else: f.set_result({'index': i, 'valid': i != 2})
                self.completed.append(i)


def test_bounded_out_of_order_responses_are_reordered_and_drained():
    transport = FakeTransport([.5, .1, .1, .1, .1])
    result = windowed_requests(range(5), transport.submit, transport.pump,
                               window=2, clock=lambda: transport.now)
    assert transport.maximum == 2
    assert transport.completed[0] == 1
    assert [r['index'] for r in result] == list(range(5))
    assert next(i for i, r in enumerate(result) if not r['valid']) == 2
    assert all(f.done() for f in transport.futures)


def test_window_overlaps_waits_without_changing_results():
    timings = []
    results = []
    for window in (1, 4):
        t = FakeTransport([.3]*8)
        results.append(windowed_requests(range(8), t.submit, t.pump,
            window=window, clock=lambda: t.now))
        timings.append(t.now)
    assert results[0] == results[1]
    assert timings[1] < timings[0]  # Simulated latency only, not ROS performance.


def test_timeout_cancels_local_futures_and_never_returns_partial_success():
    t = FakeTransport([100.]*5)
    with pytest.raises(TimeoutError):
        windowed_requests(range(5), t.submit, t.pump, window=2, timeout_s=.3,
                          clock=lambda: t.now)
    assert len(t.futures) == 2
    assert all(f.cancelled() for f in t.futures)


def test_transport_error_is_not_classified_as_collision():
    t = FakeTransport([.1, 100., 100.], fail=0)
    with pytest.raises(RuntimeError, match='transport failure'):
        windowed_requests(range(3), t.submit, t.pump, window=3, clock=lambda: t.now)
    assert all(f.done() for f in t.futures)
    assert t.futures[1].cancelled()


def test_missing_response_rejected():
    f = Future(); f.set_result(None)
    with pytest.raises(RuntimeError, match='no response'):
        windowed_requests([0], lambda _: f, lambda: None)


@pytest.mark.parametrize('window', [0, 33, True, 1.5])
def test_invalid_window_rejected(window):
    with pytest.raises(ValueError):
        windowed_requests([], None, None, window=window)


def test_empty_input_submits_nothing():
    assert windowed_requests([], lambda _: pytest.fail('unexpected request'), lambda: None) == []
