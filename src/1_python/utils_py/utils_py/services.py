def from_params(node, service_name: str, default_value: str) -> str:
    return node.declare_parameter(f"services.{service_name}", default_value).value


TODO = "todo"

