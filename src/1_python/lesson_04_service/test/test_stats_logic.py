import pytest
from lesson_04_service_py.stats_logic import StatsLogic

def test_compute_normal_values():
    """Verify standard positive integers/floats."""
    data = [10.0, 20.0, 30.0]
    total, avg, status = StatsLogic.compute(data)
    
    assert total == 60.0
    assert avg == 20.0
    assert status == "Success"

def test_compute_empty_list():
    """Verify safe handling of division-by-zero risk."""
    data = []
    total, avg, status = StatsLogic.compute(data)
    
    assert total == 0.0
    assert avg == 0.0
    assert "Warning" in status

def test_compute_floats():
    """Verify floating point arithmetic."""
    data = [1.5, 2.5]
    total, avg, status = StatsLogic.compute(data)
    
    assert total == 4.0
    assert avg == 2.0

def test_compute_single_element():
    """Verify list with one item."""
    data = [42.0]
    total, avg, status = StatsLogic.compute(data)
    
    assert total == 42.0
    assert avg == 42.0
    assert status == "Success"