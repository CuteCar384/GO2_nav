import json
from typing import Optional

from unitree_api.msg import Request


API_ID_STOP_MOVE = 1003
API_ID_MOVE = 1008


class SportRequestBuilder:
    """Build stateless Unitree sport requests with monotonically increasing IDs."""

    def __init__(self) -> None:
        self._request_id = 0

    def _next_request_id(self) -> int:
        self._request_id += 1
        return self._request_id

    def build_request(
        self,
        api_id: int,
        parameter: Optional[dict] = None,
        *,
        noreply: bool = True,
        priority: int = 0,
    ) -> Request:
        request = Request()
        request.header.identity.id = self._next_request_id()
        request.header.identity.api_id = api_id
        request.header.policy.priority = priority
        request.header.policy.noreply = noreply
        request.parameter = json.dumps(parameter or {})
        return request

    def build_move_request(self, vx: float, vy: float, vyaw: float) -> Request:
        return self.build_request(API_ID_MOVE, {"x": vx, "y": vy, "z": vyaw})

    def build_stop_request(self) -> Request:
        return self.build_request(API_ID_STOP_MOVE)
