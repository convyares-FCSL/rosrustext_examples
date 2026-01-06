def from_params(node, topic_name: str, default_value: str) -> str:
    return node.declare_parameter(f"topics.{topic_name}", default_value).value
