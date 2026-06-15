from typing import Dict
import numpy as np

class ActionChunkBroker():
    """Wraps a policy to return action chunks one-at-a-time.

    Assumes that the first dimension of all action fields is the chunk size.

    A new inference call to the inner policy is only made when the current
    list of chunks is exhausted.
    """

    def __init__(self, policy , action_horizon: int):
        self._policy = policy

        self._action_horizon = action_horizon
        self._cur_step: int = 0
        self.results = dict()

        self._last_results: Dict[str, np.ndarray] | None = None
    
    def infer(self, obs: Dict) -> Dict:  # noqa: UP006
        return self._policy.infer(obs)
    
        # if self._last_results is None:
        #     self._last_results = self._policy.infer(obs)
        #     # elf._cur_step = 0

        # # results = tree.map_structure(lambda x: x[self._cur_step, ...], self._last_results)
        # # results = dict()
        # self.results["actions"] = tree.map_structure(lambda x: x[self._cur_step, ...], self._last_results["actions"])
        # if "bbox" in self._last_results:
        #     self.results["bbox"] = self._last_results["bbox"]
        # self._cur_step += 1

        # if self._cur_step >= self._action_horizon:
        #     self._last_results = None
        #     self._cur_step = 0

        # return self.results

    def reset(self) -> None:
        self._policy.reset()
        self._last_results = None
        self._cur_step = 0
