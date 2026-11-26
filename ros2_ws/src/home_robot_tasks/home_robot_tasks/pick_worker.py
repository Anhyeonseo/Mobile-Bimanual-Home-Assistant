"""Bounded spawned compute worker. It never receives a serial/camera handle."""
import multiprocessing
import os
import signal
import traceback
import uuid


def _serve(incoming, outgoing, factory, options):
    os.setsid()  # Own only this worker and its MoveIt subprocesses.
    runtime = None
    try:
        from .pick_process_budget import background_budget
        options=dict(options,compute_scheduling=background_budget())
        runtime = factory(options)
        while True:
            identifier, method, values = incoming.recv()
            if method == '_close':
                break
            try:
                result = getattr(runtime, method)(**values)
                outgoing.send((identifier, True, result))
            except Exception as exc:
                outgoing.send((identifier, False, (str(exc), traceback.format_exc())))
    except EOFError:
        pass
    finally:
        if runtime is not None:
            runtime.close()
        incoming.close()
        outgoing.close()


class Job:
    def __init__(self, worker, identifier, future):
        self.worker, self.identifier, self.future = worker, identifier, future

    def done(self):
        if self.future.done():
            if self.worker.pending is self:
                self.worker.pending = None
            return True
        return False

    def result(self):
        if not self.done():
            raise RuntimeError('compute result is not ready')
        ok, value = self.future.result()
        if not ok:
            raise RuntimeError(value[0]+'\n'+value[1])
        return value


class ComputeWorker:
    def __init__(self, factory, options):
        context = multiprocessing.get_context('spawn')
        receive_command, self.sender = context.Pipe(duplex=False)
        self.receiver, send_answer = context.Pipe(duplex=False)
        self.process = context.Process(target=_serve, args=(receive_command,send_answer,factory,options),
                                       name='pick-compute', daemon=True)
        from concurrent.futures import ThreadPoolExecutor
        self.io = ThreadPoolExecutor(max_workers=1, thread_name_prefix="pick-worker-pipe")
        self.pending = None
        self.closed = False
        try:
            self.process.start()
        except BaseException:
            for connection in (receive_command,self.sender,self.receiver,send_answer):
                connection.close()
            self.io.shutdown(wait=True,cancel_futures=True)
            raise
        receive_command.close()
        send_answer.close()

    def submit(self, method, **values):
        if self.closed or not self.process.is_alive():
            raise RuntimeError('compute worker unavailable')
        if self.pending is not None:
            raise RuntimeError('compute worker already has a request')
        identifier = uuid.uuid4().hex
        def exchange():
            self.sender.send((identifier,method,values))
            received, ok, value = self.receiver.recv()
            if received != identifier:
                raise RuntimeError('compute result belongs to another request')
            return ok, value
        job = Job(self,identifier,self.io.submit(exchange))
        self.pending = job
        return job

    def close(self):
        if self.closed:
            return
        self.closed = True
        if self.pending is None and self.process.is_alive():
            try:self.sender.send((None,'_close',{}))
            except (OSError,EOFError):pass
            self.process.join(2)
        # Also remove planner subprocesses if inference or ROS is stuck. These
        # processes never own the robot; hardware STOP belongs to the parent.
        pid = self.process.pid
        try:
            if os.getpgid(pid) == pid:
                os.killpg(pid,signal.SIGTERM)
            elif self.process.is_alive():
                self.process.terminate()
        except ProcessLookupError:
            pass
        self.process.join(2)
        try:
            os.killpg(pid,signal.SIGKILL)
        except ProcessLookupError:
            pass
        if self.process.is_alive():
            self.process.kill()
            self.process.join(2)
        self.sender.close()
        self.receiver.close()
        self.io.shutdown(wait=True, cancel_futures=True)
