"""Shutdown ordering for our synchronous ROS callbacks on Jazzy."""

import logging


def drain_executor(executor):
    """Call only after the spin loop has stopped, never from a callback."""
    # Jazzy destroys its guard before joining the worker pool. Queued handlers
    # still trigger that guard, so drain submissions while ROS entities exist.
    pool = getattr(executor, '_executor', None)
    try:
        if pool is not None:
            pool.shutdown(wait=True)
        for future in getattr(executor, '_futures', ()):
            if future.done() and not future.cancelled():
                error = future.exception()
                if error is not None:
                    logging.getLogger(__name__).error('ROS callback failed during shutdown: %s', error)
    finally:
        executor.shutdown(wait_for_threads=True)
