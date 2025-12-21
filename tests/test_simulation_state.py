
import pytest
from gamegine.simulation.state import StateSpace, ValueEntry, ValueIncrease, ValueDecrease

class TestValueEntry:
    def test_get_set(self):
        entry = ValueEntry(10, "health")
        assert entry.get() == 10
        assert entry.get_name() == "health"
        
        entry.set(20)
        assert entry.get() == 20

class TestStateSpace:
    def test_create_and_access(self):
        space = StateSpace()
        
        # Test setValue and getValue
        entry = space.setValue("score", 0)
        assert entry.get() == 0
        assert space.getValue("score").get() == 0
        
        # Test item access
        space["score"] = 100
        assert space["score"].get() == 100
        
    def test_namespaces(self):
        root = StateSpace()
        sub = root.createSpace("player")
        
        assert root.get("player") == sub
        
        sub.setValue("hp", 100)
        assert root.get("player").getValue("hp").get() == 100

    def test_register_space(self):
        root = StateSpace()
        sub = StateSpace()
        root.registerSpace("level", sub)
        
        assert root.get("level") == sub

class TestValueChanges:
    def test_increase(self):
        entry = ValueEntry(10, "mana")
        inc = ValueIncrease(entry, 5)
        
        assert inc.current() == 10
        assert inc.requested() == 15
        
        new_val = inc.apply()
        assert new_val == 15
        assert entry.get() == 15

    def test_decrease(self):
        entry = ValueEntry(10, "mana")
        dec = ValueDecrease(entry, 3)
        
        assert dec.current() == 10
        assert dec.requested() == 7
        
        new_val = dec.apply()
        assert new_val == 7
        assert entry.get() == 7
