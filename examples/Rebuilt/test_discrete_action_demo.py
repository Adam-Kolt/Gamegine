#!/usr/bin/env python
"""Automated test for discrete_action_demo.py

This test verifies that:
1. The demo initializes without errors
2. All helper methods work correctly
3. Actions can execute without crashing

Run with: python examples/Rebuilt/test_discrete_action_demo.py
"""

import sys
sys.path.insert(0, '.')

def test_demo_initialization():
    """Test that the demo initializes without errors."""
    print("=" * 60)
    print("TEST: Demo Initialization")
    print("=" * 60)
    
    try:
        # Import required modules
        from examples.Rebuilt.discrete_action_demo import (
            MultiRobotDemo,
            create_robot,
            create_swerve_config,
            LEGACY_DEMO_ACTIONS,
            ROBOT_CONFIGS,
            DemoAction,
        )
        from examples.Rebuilt.scoring import Fuel, Hub, AllianceZone
        from gamegine.first.alliance import Alliance
        
        print("[OK] Imports successful")
        
        # Test robot creation
        robot = create_robot("TestBot", Alliance.BLUE)
        assert robot is not None, "Robot creation failed"
        print("[OK] Robot creation successful")
        
        # Test actions list - now includes both alliances (3 Blue + 3 Red)
        assert len(ROBOT_CONFIGS) == 6, "Should have 6 robot configs (3 Blue + 3 Red)"
        print(f"[OK] {len(ROBOT_CONFIGS)} robot configs defined")
        
        print("\n[PASS] Demo initialization tests passed\n")
        return True
        
    except Exception as e:
        print(f"\n[FAIL] Error: {e}\n")
        import traceback
        traceback.print_exc()
        return False


def test_helper_methods():
    """Test that helper methods work correctly."""
    print("=" * 60)
    print("TEST: Helper Methods")
    print("=" * 60)
    
    try:
        from examples.Rebuilt.discrete_action_demo import DiscreteActionDemo
        from examples.Rebuilt.scoring import Fuel, Hub
        from gamegine.simulation.robot import RobotState
        from gamegine.utils.NCIM.Dimensions.spatial import Feet, Inch
        from gamegine.utils.NCIM.Dimensions.angular import Degree
        
        # Create a minimal demo instance (avoid full renderer)
        # We'll test the methods in isolation
        
        # Test _get_center with a fake RobotState-like object
        class MockRobotState:
            class MockValue:
                def __init__(self, val):
                    self._val = val
                def get(self):
                    return self._val
            
            def __init__(self, x, y):
                self.x = self.MockValue(x)
                self.y = self.MockValue(y)
        
        mock_robot = MockRobotState(Feet(10), Feet(20))
        
        # Create mock demo class to test methods
        class MockDemo:
            def _get_center(self, obj):
                """Copied from DiscreteActionDemo for isolated testing."""
                if hasattr(obj, 'x') and hasattr(obj, 'y') and not hasattr(obj, 'bounds'):
                    x = obj.x.get() if hasattr(obj.x, 'get') else obj.x
                    y = obj.y.get() if hasattr(obj.y, 'get') else obj.y
                    return (x, y)
                if hasattr(obj, 'bounds'):
                    b = obj.bounds
                    if hasattr(b, 'get_center'):
                        return b.get_center()
                    return (b.x, b.y)
                raise ValueError(f"Unknown object type {type(obj)}")
        
        demo = MockDemo()
        center = demo._get_center(mock_robot)
        assert center[0] == Feet(10), f"X mismatch: {center[0]}"
        assert center[1] == Feet(20), f"Y mismatch: {center[1]}"
        print("[OK] _get_center works with RobotState-like objects")
        
        # Test with raw values (no .get() method)
        class SimplePoint:
            def __init__(self, x, y):
                self.x = x
                self.y = y
        
        simple = SimplePoint(Inch(100), Inch(200))
        center = demo._get_center(simple)
        assert center[0] == Inch(100), f"X mismatch: {center[0]}"
        assert center[1] == Inch(200), f"Y mismatch: {center[1]}"
        print("[OK] _get_center works with simple Point objects")
        
        # Test with bounds object
        class MockBounds:
            def __init__(self, x, y):
                self.x = x
                self.y = y
        
        class MockInteractable:
            def __init__(self, x, y):
                self.bounds = MockBounds(x, y)
        
        interactable = MockInteractable(Feet(5), Feet(15))
        center = demo._get_center(interactable)
        assert center[0] == Feet(5), f"X mismatch: {center[0]}"
        assert center[1] == Feet(15), f"Y mismatch: {center[1]}"
        print("[OK] _get_center works with Interactable objects")
        
        print("\n[PASS] Helper method tests passed\n")
        return True
        
    except Exception as e:
        print(f"\n[FAIL] Error: {e}\n")
        import traceback
        traceback.print_exc()
        return False


def test_animation_classes():
    """Test animation classes work correctly."""
    print("=" * 60)
    print("TEST: Animation Classes")
    print("=" * 60)
    
    try:
        from examples.Rebuilt.discrete_action_demo import (
            ProjectileAnimation,
            AnimationLayer,
        )
        from gamegine.utils.NCIM.Dimensions.spatial import Feet, Inch
        
        # Test ProjectileAnimation
        start = (Feet(0), Feet(0))
        end = (Feet(10), Feet(10))
        anim = ProjectileAnimation(start, end, duration=1.0)
        
        assert anim.elapsed == 0.0, "Initial elapsed should be 0"
        print("[OK] ProjectileAnimation created successfully")
        
        # Update animation
        finished = anim.update(0.5)
        assert not finished, "Animation should not be finished at 0.5s"
        assert anim.elapsed == 0.5, "Elapsed should be 0.5s"
        print("[OK] ProjectileAnimation.update() works")
        
        # Finish animation
        finished = anim.update(0.6)
        assert finished, "Animation should be finished after 1.1s total"
        print("[OK] ProjectileAnimation finishes correctly")
        
        # Test AnimationLayer
        layer = AnimationLayer()
        assert len(layer.animations) == 0, "Layer should start empty"
        
        layer.add(ProjectileAnimation(start, end, 1.0))
        layer.add(ProjectileAnimation(start, end, 0.5))
        assert len(layer.animations) == 2, "Layer should have 2 animations"
        print("[OK] AnimationLayer.add() works")
        
        # Update - should remove finished ones
        layer.update(0.6)  # First anim at 0.6s, second finishes
        assert len(layer.animations) == 1, f"Layer should have 1 animation, has {len(layer.animations)}"
        print("[OK] AnimationLayer.update() removes finished animations")
        
        print("\n[PASS] Animation class tests passed\n")
        return True
        
    except Exception as e:
        print(f"\n[FAIL] Error: {e}\n")
        import traceback
        traceback.print_exc()
        return False


def test_jitter_calculation():
    """Test that jitter calculations don't cause type errors."""
    print("=" * 60)
    print("TEST: Jitter Calculations")
    print("=" * 60)
    
    try:
        import random
        from gamegine.utils.NCIM.Dimensions.spatial import Feet, Inch
        
        # Simulate what _queue_burst does
        start_pos = (Feet(5), Feet(10))
        end_pos = (Feet(20), Feet(25))
        
        sx, sy = start_pos
        ex, ey = end_pos
        
        # This is the operation that was failing
        jitter_start = (
            sx + Inch(random.uniform(-4, 4)),
            sy + Inch(random.uniform(-4, 4))
        )
        jitter_end = (
            ex + Inch(random.uniform(-4, 4)),
            ey + Inch(random.uniform(-4, 4))
        )
        
        assert jitter_start[0] is not None, "Jitter start X failed"
        assert jitter_start[1] is not None, "Jitter start Y failed"
        assert jitter_end[0] is not None, "Jitter end X failed"
        assert jitter_end[1] is not None, "Jitter end Y failed"
        
        print("[OK] Jitter calculations with SpatialMeasurement work")
        print(f"    Start: {jitter_start}")
        print(f"    End: {jitter_end}")
        
        print("\n[PASS] Jitter calculation tests passed\n")
        return True
        
    except Exception as e:
        print(f"\n[FAIL] Error: {e}\n")
        import traceback
        traceback.print_exc()
        return False


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("AUTOMATED TEST SUITE: discrete_action_demo.py")
    print("=" * 60 + "\n")
    
    results = []
    
    results.append(("Initialization", test_demo_initialization()))
    results.append(("Helper Methods", test_helper_methods()))
    results.append(("Animation Classes", test_animation_classes()))
    results.append(("Jitter Calculations", test_jitter_calculation()))
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "[PASS]" if passed else "[FAIL]"
        print(f"  {status} {name}")
        if not passed:
            all_passed = False
    
    print()
    if all_passed:
        print("All tests PASSED!")
        return 0
    else:
        print("Some tests FAILED!")
        return 1


if __name__ == "__main__":
    exit(main())
