import signal
import sys

class ShutdownTool:
    def __init__(self):
        self.callbacks = []
        self.if_capture = False
        # 捕获 Ctrl+C 信号
        signal.signal(signal.SIGINT, self._handle)

    def on_shutdown(self, func):
        """注册退出时要执行的函数"""
        self.callbacks.append(func)

    def is_shutdown(self):
        return self.if_capture

    def _handle(self, signum, frame):
        self.if_capture = True
        print("\n[ShutdownTool] Caught Ctrl+C, running cleanup...")
        for cb in self.callbacks:
            try:
                cb()
            except Exception as e:
                print(f"Error in shutdown callback: {e}")
        sys.exit(0)