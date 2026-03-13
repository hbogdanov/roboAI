from __future__ import annotations

import argparse
import json
from dataclasses import asdict

from roboai.app.run_demo import run_demo
from roboai.integration.ros2_wrapper.bus import TopicBus
from roboai.integration.ros2_wrapper.nodes import ControllerNode, FrontierNode, MappingNode, PlannerNode, TopicNames


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", required=True)
    parser.add_argument("--planner", default="hybrid")
    parser.add_argument("--seed", type=int, default=7)
    args = parser.parse_args()

    bus = TopicBus()
    topics = TopicNames()
    trace: list[str] = []
    MappingNode(bus, topics, lambda message: trace.append("map") or message)
    FrontierNode(bus, topics, lambda message: trace.append("frontier") or message)
    PlannerNode(bus, topics, lambda message: trace.append("planner") or message)
    ControllerNode(bus, topics, lambda message: trace.append("controller") or message)
    bus.publish(topics.scan, {"event": "bootstrap"})

    metrics = run_demo(
        map_name=args.map,
        planner_name=args.planner,
        seed=args.seed,
        frontier_policy="semantic_information_gain",
        semantic_mode="enabled",
    )
    payload = {
        "ros2_wrapper": "in_process_topic_graph",
        "topics": asdict(topics),
        "node_trace": trace,
        "metrics": asdict(metrics),
    }
    print(json.dumps(payload, indent=2, default=str))


if __name__ == "__main__":
    main()
