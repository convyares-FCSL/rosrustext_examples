# Internal helper
def _from_params(node, topic_key: str, default_value: str) -> str:
    """Load a topic name from parameters (e.g. topics.chatter), with a default."""
    param_name = f"topics.{topic_key}"
    if node.has_parameter(param_name):
        return node.get_parameter(param_name).value
    return node.declare_parameter(param_name, default_value).value


# Explicit Topic Accessors
# These functions ensure consistency across the codebase.

def chatter(node) -> str:
    """
    Get the configured topic name for 'chatter'.
    Default: 'chatter' (which often remaps to /tutorial/chatter via global config)
    """
    return _from_params(node, "chatter", "chatter")

def robot_news(node) -> str:
    """Get the configured topic name for 'robot_news'"""
    return _from_params(node, "robot_news", "robot_news")

def number(node) -> str:
    """Get the configured topic name for 'number'"""
    return _from_params(node, "number", "number")

def number_count(node) -> str:
    """Get the configured topic name for 'number_count'"""
    return _from_params(node, "number_count", "number_count")
