def from_params(node, service_name: str, default_value: str) -> str:
    return node.declare_parameter(f"services.{service_name}", default_value).value


ADD_TWO_INTS = "add_two_ints"
ADD_TWO_INTS_SERVICE = "add_two_ints"
