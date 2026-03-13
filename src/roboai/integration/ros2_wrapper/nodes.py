from __future__ import annotations

from dataclasses import dataclass

from roboai.core.frontier import rank_frontier_targets


@dataclass(slots=True)
class TopicNames:
    scan: str = "/scan"
    occupancy: str = "/map"
    frontier_targets: str = "/frontiers"
    waypoint_path: str = "/plan"
    velocity_cmd: str = "/cmd_vel"


class MappingNode:
    def __init__(self, bus, topics: TopicNames, callback):
        self.callback = callback
        bus.subscribe(topics.scan, self._on_scan)
        self.bus = bus
        self.topics = topics

    def _on_scan(self, message) -> None:
        self.bus.publish(self.topics.occupancy, self.callback(message))


class FrontierNode:
    def __init__(self, bus, topics: TopicNames, callback):
        self.callback = callback
        bus.subscribe(topics.occupancy, self._on_map)
        self.bus = bus
        self.topics = topics

    def _on_map(self, message) -> None:
        self.bus.publish(self.topics.frontier_targets, self.callback(message))


class PlannerNode:
    def __init__(self, bus, topics: TopicNames, callback):
        self.callback = callback
        bus.subscribe(topics.frontier_targets, self._on_frontiers)
        self.bus = bus
        self.topics = topics

    def _on_frontiers(self, message) -> None:
        self.bus.publish(self.topics.waypoint_path, self.callback(message))


class ControllerNode:
    def __init__(self, bus, topics: TopicNames, callback):
        self.callback = callback
        bus.subscribe(topics.waypoint_path, self._on_plan)
        self.bus = bus
        self.topics = topics

    def _on_plan(self, message) -> None:
        self.bus.publish(self.topics.velocity_cmd, self.callback(message))


def default_frontier_callback(payload):
    return rank_frontier_targets(**payload)
