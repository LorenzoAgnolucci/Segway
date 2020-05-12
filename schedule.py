from datetime import datetime, timedelta
from time import sleep


class SampleData:
    last_cycle_end: datetime

    def __init__(self, last_cycle_end):
        self.last_cycle_end = last_cycle_end


def every(interval):
    def wrapped(func):
        def f():
            last_cycle_end = datetime.now()
            while True:
                last_timestamp = datetime.now()
                sampling_data = SampleData(last_cycle_end)
                func(sampling_data=sampling_data)
                now = datetime.now()
                elapsed = now - last_timestamp

                if elapsed < interval:
                    seconds_to_sleep = ((interval - elapsed) / timedelta(milliseconds=1)) / 1000
                    sleep(seconds_to_sleep)
        return f
    return wrapped
