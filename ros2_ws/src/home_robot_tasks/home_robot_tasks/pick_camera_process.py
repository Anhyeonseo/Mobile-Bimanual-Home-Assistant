"""Isolate SDK/GIL work from the one serial owner; preserve exposure timestamps."""
import multiprocessing
import os
import threading


def _capture(factory, sender, stop):
    from .pick_process_budget import background_budget
    try:
        budget=background_budget()
        with factory() as camera:
            sender.send(('ready',dict(pid=os.getpid(),scheduling=budget)))
            while not stop.is_set():
                record,images=camera.next_frame()
                if not stop.is_set():sender.send(('frame',(record,images)))
        sender.send(('stopped',None))
    except BaseException as error:
        try:sender.send(('error',type(error).__name__+': '+str(error)))
        except (OSError,EOFError):pass
    finally:sender.close()


class PickCameraProcess:
    """A pipe applies backpressure; no unbounded queue or restamped old frames.

    Only the spawned child constructs the camera. Only the parent receiver
    calls consume(), so live arm inputs and the serial handle never cross IPC.
    """
    def __init__(self, factory, consume):
        context=multiprocessing.get_context('spawn')
        self.receiver,self.sender=context.Pipe(duplex=False)
        self.stop=context.Event()
        self.process=context.Process(target=_capture,args=(factory,self.sender,self.stop),
                                     name='pick-rgbd',daemon=True)
        self.consume=consume
        self.ready=threading.Event();self.error=None;self.frames=0;self.runtime_info=None
        self.closed=False
        self.thread=threading.Thread(target=self._receive,name='pick-rgbd-pipe',daemon=True)

    def _receive(self):
        try:
            while not self.closed:
                if not self.receiver.poll(.1):
                    if not self.process.is_alive():
                        if self.stop.is_set():break
                        raise RuntimeError('camera process exited without completion')
                    continue
                kind,value=self.receiver.recv()
                if kind=='ready':self.runtime_info=value;self.ready.set()
                elif kind=='frame':self.consume(*value);self.frames+=1
                elif kind=='stopped':break
                elif kind=='error':raise RuntimeError(value)
                else:raise RuntimeError('invalid camera process message')
        except BaseException as error:
            if not self.closed:self.error=error
            self.stop.set()
        finally:self.ready.set()

    def __enter__(self):
        try:
            self.process.start();self.sender.close();self.thread.start()
            if not self.ready.wait(12):raise TimeoutError('D415 process startup timeout')
            self.check()
            if self.runtime_info is None:raise RuntimeError('camera process did not initialize')
            return self
        except BaseException:
            self.__exit__(None,None,None)
            raise

    def check(self):
        if self.error:raise RuntimeError('camera worker: '+str(self.error))
        if not self.closed and self.process.pid and not self.process.is_alive():
            raise RuntimeError('camera process stopped unexpectedly')

    def __exit__(self,*_):
        if self.closed:return
        self.stop.set()
        if self.process.pid:
            self.process.join(3)
            if self.process.is_alive():self.process.terminate();self.process.join(1)
            if self.process.is_alive():self.process.kill();self.process.join(1)
        self.closed=True
        if self.thread.ident:self.thread.join(1)
        self.receiver.close();self.sender.close()
