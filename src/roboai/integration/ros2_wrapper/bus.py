from __future__ import annotations

from collections import defaultdict
from typing import Callable


class TopicBus:
    def __init__(self) -> None:
        self._subscribers: dict[str, list[Callable]] = defaultdict(list)

    def publish(self, topic: str, message) -> None:
        for callback in self._subscribers.get(topic, []):
            callback(message)

    def subscribe(self, topic: str, callback: Callable) -> None:
        self._subscribers[topic].append(callback)
