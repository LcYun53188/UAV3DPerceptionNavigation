"""One bounded process for speculative frontier searches, with no ROS ownership."""
from concurrent.futures import ProcessPoolExecutor
import multiprocessing

from .exploration import choose_subgoal


def select_target(grid, start, goal, radius, settings, visited, rejected, best_distance):
    return choose_subgoal(grid, start, goal, radius, settings, visited, rejected,
                          explore=True, best_distance=best_distance, continuous=True)


class BackgroundSelector:
    def __init__(self):
        self.pool = None
        self.future = None
        self.context = None

    def submit(self, context, *args):
        # Never accumulate a queue of obsolete maps behind a running search.
        if self.future is not None:
            return False
        if self.pool is None:
            self.pool = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self.context = context
        try:
            self.future = self.pool.submit(select_target, *args)
        except Exception:
            self.pool.shutdown(wait=False, cancel_futures=True)
            self.pool = self.context = None
            raise
        return True

    def poll(self):
        if self.future is None or not self.future.done():
            return None
        context, future = self.context, self.future
        self.future = self.context = None
        try:
            return context, future.result()
        except Exception:
            self.pool.shutdown(wait=False, cancel_futures=True)
            self.pool = None
            raise

    def invalidate(self):
        # Running work cannot always be cancelled. Keep its slot until it ends,
        # but retire its identity so a late result cannot initiate a request.
        self.context = None
        if self.future is not None and self.future.cancel():
            self.future = None

    def close(self):
        self.invalidate()
        if self.pool is not None:
            self.pool.shutdown(wait=False, cancel_futures=True)
