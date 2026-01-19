"""
Unit tests for Lesson 07 logic.
"""
from lesson_07_actions.logic import FibonacciGenerator

def test_fibonacci_generator_logic():
    """Verify Fibonacci logic sequence accumulation."""
    gen = FibonacciGenerator()
    seq = []
    
    # Order 0
    # Step 1
    val = gen.step(seq)
    seq.append(val)
    assert val == 0
    assert seq == [0]
    
    # Step 2
    val = gen.step(seq)
    seq.append(val)
    assert val == 1
    assert seq == [0, 1]
    
    # Step 3
    val = gen.step(seq)
    seq.append(val)
    assert val == 1
    assert seq == [0, 1, 1]

    # Step 4
    val = gen.step(seq)
    seq.append(val)
    assert val == 2
    assert seq == [0, 1, 1, 2]

    # Step 5
    val = gen.step(seq)
    seq.append(val)
    assert val == 3
    assert seq == [0, 1, 1, 2, 3]

    # Confirm statelessness helper (if logic was pure function it would be easier, 
    # but generator class implies some state management, which we're testing here)
