import numpy as np
import ctypes
 
class HighPrecisionTimer:
    """High precision timing utilities using Linux clock_nanosleep"""
    
    def __init__(self):
        self.CLOCK_REALTIME = 0
        self.TIMER_ABSTIME = 1
        self._setup_timing()
    
    def _setup_timing(self):
        """Setup high-precision timing functions"""
        class timespec(ctypes.Structure):
            _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]
        
        self.timespec = timespec
        self.libc = ctypes.CDLL("libc.so.6")
        
        # Define function prototypes
        self.libc.clock_nanosleep.argtypes = [
            ctypes.c_int, ctypes.c_int, 
            ctypes.POINTER(timespec), ctypes.POINTER(timespec)
        ]
        self.libc.clock_nanosleep.restype = ctypes.c_int
        
        self.libc.clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(timespec)]
        self.libc.clock_gettime.restype = ctypes.c_int
    
    def timespec_add(self, a, b):
        """Add two timespec structures"""
        result = self.timespec()
        result.tv_sec = a.tv_sec + b.tv_sec
        result.tv_nsec = a.tv_nsec + b.tv_nsec
        
        if result.tv_nsec >= 1000000000:
            result.tv_sec += 1
            result.tv_nsec -= 1000000000
        
        return result
    
    def clock_nanosleep(self, clock_id: int, flags: int, requested_time):
        """High precision sleep (requested_time: timespec instance)."""
        remaining = self.timespec()
        result = self.libc.clock_nanosleep(
            clock_id, flags, ctypes.byref(requested_time), ctypes.byref(remaining)
        )
        if result != 0:
            raise OSError(f"clock_nanosleep failed with error code: {result}")
    
    def clock_gettime(self, clock_id: int):
        """Get current time"""
        ts = self.timespec()
        result = self.libc.clock_gettime(clock_id, ctypes.byref(ts))
        if result != 0:
            raise OSError(f"clock_gettime failed with error code: {result}")
        return ts
    
    def sleep_milliseconds(self, ms: float):
        """Sleep for specified milliseconds"""
        dt = self.timespec()
        dt.tv_sec = int(ms // 1000)
        dt.tv_nsec = int((ms % 1000) * 1000000)
        
        current_time = self.clock_gettime(self.CLOCK_REALTIME)
        target_time = self.timespec_add(current_time, dt)
        self.clock_nanosleep(self.CLOCK_REALTIME, self.TIMER_ABSTIME, target_time)

if __name__ == "__main__":
    timer = HighPrecisionTimer()
    print("Testing high precision sleep...")

    start = timer.clock_gettime(timer.CLOCK_REALTIME)
    print(f"Start time: {start.tv_sec}.{start.tv_nsec:09d}")

    timer.sleep_milliseconds(1500)  # 睡眠1.5秒

    end = timer.clock_gettime(timer.CLOCK_REALTIME)
    print(f"End time:   {end.tv_sec}.{end.tv_nsec:09d}")

    elapsed_sec = end.tv_sec - start.tv_sec
    elapsed_nsec = end.tv_nsec - start.tv_nsec
    if elapsed_nsec < 0:
        elapsed_sec -= 1
        elapsed_nsec += 1000000000
    print(f"Elapsed: {elapsed_sec}.{elapsed_nsec:09d} seconds")