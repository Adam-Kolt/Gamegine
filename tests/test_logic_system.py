import pytest
from unittest.mock import MagicMock
from gamegine.simulation.logic import LogicRule, TriggerType
from gamegine.simulation.conditions import TimeGreaterThan, RobotInZone
from gamegine.simulation.state import ValueChange, ValueIncrease, ValueEntry, StateSpace
from gamegine.simulation.game import GameState
from gamegine.utils.NCIM.Dimensions.spatial import Meter
from gamegine.representation.bounds import Boundary, Rectangle

# Mock GameState helper
def create_mock_gamestate(time=0.0):
    gs = GameState()
    gs.current_time.set(time)
    gs.score.set(0)
    return gs

def test_on_true_trigger():
    # Setup
    gs = create_mock_gamestate(0.0)
    action_mock = MagicMock(return_value=[ValueIncrease(gs.score, 10)])
    
    rule = LogicRule(
        name="TestRule",
        condition=TimeGreaterThan(5.0),
        trigger_type=TriggerType.ON_TRUE,
        action=action_mock
    )
    
    # Time = 0: Condition False
    changes = rule.update(1.0, gs)
    assert len(changes) == 0
    assert not rule._has_triggered

    # Time = 4: Condition False
    gs.current_time.set(4.0)
    changes = rule.update(1.0, gs)
    assert len(changes) == 0

    # Time = 6: Condition True -> Trigger
    gs.current_time.set(6.0)
    changes = rule.update(1.0, gs)
    assert len(changes) == 1
    assert changes[0].value == 10
    assert rule._has_triggered
    
    # Time = 7: Condition True -> Already Triggered
    gs.current_time.set(7.0)
    changes = rule.update(1.0, gs)
    assert len(changes) == 0

    # Reset: Condition False
    gs.current_time.set(0.0)
    changes = rule.update(1.0, gs)
    assert not rule._has_triggered

def test_while_true_trigger_with_delay():
    gs = create_mock_gamestate(0.0)
    action_mock = MagicMock(return_value=[ValueIncrease(gs.score, 1)])
    
    # Condition: Always True for testing delay and interval
    rule = LogicRule(
        name="DelayRule",
        condition=lambda g: True,
        trigger_type=TriggerType.WHILE_TRUE,
        action=action_mock,
        delay=2.0,
        interval=1.0
    )
    
    # Update 1.0s -> Total 1.0s < 2.0s -> No Trigger
    changes = rule.update(1.0, gs)
    assert len(changes) == 0
    
    # Update 1.0s -> Total 2.0s >= 2.0s -> Delay Met
    # Interval timer starts at `interval` (1.0), so it should trigger immediately
    changes = rule.update(1.0, gs)
    assert len(changes) == 1
    
    # Update 0.5s -> Interval timer = 0.5 < 1.0 -> No Trigger
    changes = rule.update(0.5, gs)
    assert len(changes) == 0
    
    # Update 0.5s -> Interval timer = 1.0 >= 1.0 -> Trigger
    changes = rule.update(0.5, gs)
    assert len(changes) == 1

def test_robot_in_zone_condition():
    gs = create_mock_gamestate(0.0)
    
    # Setup Robot State
    robot_space = gs.createSpace("robots")
    r_state = robot_space.createSpace("TestRobot")
    r_state.setValue("x", Meter(0))
    r_state.setValue("y", Meter(0))
    
    # Setup Zone: Rectangle 10x10 centered at 10,10
    # Boundary(Rectangle(10, 10), transform=Translated(10, 10))
    from gamegine.representation.bounds import Transform2D, Transform3D
    zone_shape = Rectangle(Meter(10), Meter(10)) # -5 to 5
    # Move zone to 10, 10 (range 5 to 15)
    zone_transform = Transform3D(position=(Meter(10), Meter(10), Meter(0)))
    zone = Boundary(zone_shape, transform=zone_transform)
    
    condition = RobotInZone("TestRobot", zone)
    
    # 0,0 is outside 5-15 range
    assert condition(gs) == False
    
    # Move robot to 10, 10
    r_state.setValue("x", Meter(10))
    r_state.setValue("y", Meter(10))
    assert condition(gs) == True
    
    # Move robot to 16, 10 (outside)
    r_state.setValue("x", Meter(16))
    assert condition(gs) == False
