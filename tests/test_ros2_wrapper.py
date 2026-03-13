from roboai.integration.ros2_wrapper.bus import TopicBus
from roboai.integration.ros2_wrapper.nodes import ControllerNode, FrontierNode, MappingNode, PlannerNode, TopicNames


def test_topic_bus_executes_ros_style_node_chain():
    bus = TopicBus()
    topics = TopicNames()
    trace = []

    MappingNode(bus, topics, lambda message: trace.append("map") or message)
    FrontierNode(bus, topics, lambda message: trace.append("frontier") or message)
    PlannerNode(bus, topics, lambda message: trace.append("planner") or message)
    ControllerNode(bus, topics, lambda message: trace.append("controller") or message)

    bus.publish(topics.scan, {"event": "bootstrap"})

    assert trace == ["map", "frontier", "planner", "controller"]
