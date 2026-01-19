# Import the existing helper
from .utils import _get_or_declare
# Import the new Lifecycle Shim
from .lifecycle import LifecycleNode

# Export both so they can be imported directly from 'utils_py'
__all__ = [
    "_get_or_declare",
    "LifecycleNode",
]