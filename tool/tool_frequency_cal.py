import time 

class ToolFrequencyCal:
    def __init__(self, total_count):
        self.count = 0 
        self.total_count = total_count
        self.last_timestamp = 0
        self.frequency = 0

    def update(self):
        self.count = self.count + 1
        if self.count == self.total_count:
            timestamp = time.perf_counter()
            self.frequency = self.total_count/(timestamp - self.last_timestamp)
            self.last_timestamp = timestamp
            self.count = 0