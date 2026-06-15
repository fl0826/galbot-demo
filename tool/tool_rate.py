import time
import math
from typing import Callable, Optional

class ToolRate:
    """
    A simple implementation of ROS-like Rate.

    Usage:
        r = Rate(10)         # 10 Hz
        while running:
            do_work()
            r.sleep()        # block to keep 10 Hz

    Arguments:
        hz: frequency in Hz (must be > 0)
        time_fn: function returning monotonic time in seconds (default time.monotonic)
        sleep_fn: function to sleep for seconds (default time.sleep)
    """

    def __init__(self,
                 hz: float,
                 time_fn: Callable[[], float] = time.monotonic,
                 sleep_fn: Callable[[float], None] = time.sleep):
        if hz <= 0:
            raise ValueError("hz must be > 0")
        self._hz = float(hz)
        self._period = 1.0 / self._hz
        self._time = time_fn
        self._sleep = sleep_fn

        # schedule next wakeup relative to current monotonic time
        now = self._time()
        self._next_time = now + self._period

    def sleep(self) -> None:
        """
        Sleep until the next scheduled cycle. If we're behind schedule,
        don't sleep and advance the internal schedule so next call
        aims for the next period (skips missed cycles).
        """
        now = self._time()
        to_sleep = self._next_time - now

        if to_sleep > 0:
            # on-time: sleep exactly the remaining time
            self._sleep(to_sleep)
            # advance schedule by exactly one period
            self._next_time += self._period
            return
        else:
            # behind schedule: compute how many periods we missed
            # N = floor((now - next_time) / period) + 1
            missed = int(math.floor((now - self._next_time) / self._period)) + 1
            self._next_time += missed * self._period
            # do not sleep (we're late)
            return

    def remaining(self) -> float:
        """
        Return seconds remaining until next scheduled wakeup (can be negative if we're late).
        """
        return self._next_time - self._time()

    def reset(self) -> None:
        """
        Reset schedule: next wakeup will be one period after current time.
        """
        now = self._time()
        self._next_time = now + self._period

    def set_hz(self, hz: float) -> None:
        """
        Change frequency (affects future periods). Keeps the schedule anchored:
        next wakeup remains at the same absolute time, but the period used after that changes.
        If you prefer to re-anchor relative to now, call reset() after set_hz().
        """
        if hz <= 0:
            raise ValueError("hz must be > 0")
        self._hz = float(hz)
        self._period = 1.0 / self._hz

    @property
    def hz(self) -> float:
        return self._hz

    @property
    def period(self) -> float:
        return self._period
    
if __name__ == "__main__":

    r = ToolRate(2)  # 2 Hz -> 0.5s period

    for i in range(6):
        # print(f"{i}  time={time.monotonic():.6f}  remaining={r.remaining():.6f}")
        # 模拟工作耗时

        print(time.perf_counter())
        if i == 2:
            # 第3次做得慢一点，模拟超时
            time.sleep(0.7)
        r.sleep()