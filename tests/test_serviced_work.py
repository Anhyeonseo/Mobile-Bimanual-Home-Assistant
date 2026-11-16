from concurrent.futures import ThreadPoolExecutor
import threading
import time
import pytest
from home_robot_tasks.serviced_work import await_serviced


def test_slow_capture_storage_cannot_block_controller_service():
    serviced=[]
    def storage():time.sleep(.35);return 'saved'
    with ThreadPoolExecutor(1) as worker:
        result=await_serviced(worker.submit(storage),lambda:serviced.append(time.monotonic()))
    assert result=='saved' and len(serviced)>10
    assert max(b-a for a,b in zip(serviced,serviced[1:]))<.1
    assert serviced[-1]-serviced[0]>=.3


def test_storage_exception_is_reported_after_live_service():
    observed=[]
    def fail():raise OSError('disk full')
    with ThreadPoolExecutor(1) as worker:
        with pytest.raises(OSError,match='disk full'):
            await_serviced(worker.submit(fail),lambda:observed.append(True))
    assert observed


def test_controller_fault_interrupts_wait_without_hiding_it():
    ready=threading.Event()
    def fault():raise RuntimeError('controller fault')
    with ThreadPoolExecutor(1) as worker:
        try:
            with pytest.raises(RuntimeError,match='controller fault'):
                await_serviced(worker.submit(ready.wait),fault)
        finally:ready.set()
