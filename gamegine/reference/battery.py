"""
Battery Model for Trajectory Planning
======================================

Provides a simple battery model that accounts for:
- Voltage sag under load (V = V_oc - I*R)
- RoboRIO brownout protection (limits current to maintain safe voltage)
- Optional state-of-charge tracking (dynamic mode)

This model is designed for trajectory acceleration limiting, not time-stepped simulation.
For high-fidelity electrical simulation, see gamegine.hifi_sim.
"""

from dataclasses import dataclass, field
from typing import Optional
from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt, ElectricPot
from gamegine.utils.NCIM.Dimensions.current import Ampere, CurrentMeasurement


@dataclass
class BatteryModel:
    """
    Simple battery model for trajectory planning.
    
    Models voltage sag under load and enforces RoboRIO brownout limits.
    
    :param open_circuit_voltage: Battery voltage at rest (full charge ~13V, depleted ~11.7V)
    :param internal_resistance: Battery internal resistance in Ohms (typical FRC: 0.012-0.020Ω)
    :param brownout_voltage: Minimum voltage before RoboRIO brownout protection activates
    :param blackout_voltage: Voltage at which RoboRIO shuts down completely
    :param state_of_charge: Current SoC (1.0 = full, 0.0 = empty). Only used in dynamic mode.
    :param capacity_ah: Battery capacity in Amp-hours (typical FRC: 18Ah)
    """
    
    # Electrical parameters
    open_circuit_voltage: ElectricPot = field(default_factory=lambda: Volt(12.8))
    internal_resistance: float = 0.015  # Ohms
    
    # RoboRIO protection limits
    brownout_voltage: ElectricPot = field(default_factory=lambda: Volt(6.3))
    blackout_voltage: ElectricPot = field(default_factory=lambda: Volt(4.5))
    
    # State tracking (for dynamic mode)
    state_of_charge: float = 1.0  # 1.0 = full, 0.0 = empty
    capacity_ah: float = 18.0  # Amp-hours
    
    # Peukert constant for capacity derating at high current
    peukert_constant: float = 1.1
    
    @classmethod
    def frc_standard(cls, initial_soc: float = 1.0) -> "BatteryModel":
        """Create a standard FRC battery model (18Ah, ~0.015Ω)."""
        ocv = cls._ocv_from_soc(initial_soc)
        return cls(
            open_circuit_voltage=Volt(ocv),
            internal_resistance=0.015,
            state_of_charge=initial_soc,
            capacity_ah=18.0,
        )
    
    @classmethod
    def worst_case(cls) -> "BatteryModel":
        """
        Create a conservative worst-case battery model.
        
        Assumes battery is partially depleted and warm (higher resistance).
        Good for trajectory planning to ensure realistic performance.
        """
        return cls(
            open_circuit_voltage=Volt(11.5),  # Partially depleted
            internal_resistance=0.020,  # Warm battery, higher R
            state_of_charge=0.5,
        )
    
    @staticmethod
    def _ocv_from_soc(soc: float) -> float:
        """
        Calculate open-circuit voltage from state of charge.
        
        Uses Hermite cubic interpolation (matching HiFi sim battery.rs).
        """
        s = max(0.0, min(1.0, soc))
        
        # Anchor points
        X0, X1, X2 = 0.0, 0.5, 1.0
        Y0, Y1, Y2 = 11.77, 12.20, 13.03
        
        # Monotone slopes at anchors
        M0, M1, M2 = 0.46, 1.26, 2.06
        
        if s <= X1:
            # Interval [0.0, 0.5]
            h = X1 - X0
            t = (s - X0) / h
            t2, t3 = t * t, t * t * t
            
            h00 = 2*t3 - 3*t2 + 1
            h10 = t3 - 2*t2 + t
            h01 = -2*t3 + 3*t2
            h11 = t3 - t2
            
            return h00*Y0 + h10*h*M0 + h01*Y1 + h11*h*M1
        else:
            # Interval [0.5, 1.0]
            h = X2 - X1
            t = (s - X1) / h
            t2, t3 = t * t, t * t * t
            
            h00 = 2*t3 - 3*t2 + 1
            h10 = t3 - 2*t2 + t
            h01 = -2*t3 + 3*t2
            h11 = t3 - t2
            
            return h00*Y1 + h10*h*M1 + h01*Y2 + h11*h*M2
    
    def get_terminal_voltage(self, current_draw: CurrentMeasurement) -> ElectricPot:
        """
        Calculate terminal voltage under load.
        
        V_terminal = V_oc - I * R_internal
        
        :param current_draw: Total current being drawn from battery
        :return: Terminal voltage at the battery terminals
        """
        current_a = current_draw.to(Ampere) if hasattr(current_draw, 'to') else float(current_draw)
        v_oc = self.open_circuit_voltage.to(Volt) if hasattr(self.open_circuit_voltage, 'to') else float(self.open_circuit_voltage)
        
        voltage_drop = abs(current_a) * self.internal_resistance
        terminal_v = v_oc - voltage_drop
        
        return Volt(max(terminal_v, 0))
    
    def get_max_current_before_brownout(self) -> CurrentMeasurement:
        """
        Calculate maximum current draw before triggering RoboRIO brownout.
        
        I_max = (V_oc - V_brownout) / R_internal
        
        This is the critical limit - the trajectory planner should never
        request acceleration that would exceed this current.
        """
        v_oc = self.open_circuit_voltage.to(Volt) if hasattr(self.open_circuit_voltage, 'to') else float(self.open_circuit_voltage)
        v_brownout = self.brownout_voltage.to(Volt) if hasattr(self.brownout_voltage, 'to') else float(self.brownout_voltage)
        
        max_current = (v_oc - v_brownout) / self.internal_resistance
        return Ampere(max(max_current, 0))
    
    def get_max_current_for_voltage(self, min_voltage: ElectricPot) -> CurrentMeasurement:
        """
        Calculate maximum current that maintains a minimum voltage.
        
        :param min_voltage: Minimum acceptable terminal voltage
        :return: Maximum current that keeps voltage above this threshold
        """
        v_oc = self.open_circuit_voltage.to(Volt) if hasattr(self.open_circuit_voltage, 'to') else float(self.open_circuit_voltage)
        v_min = min_voltage.to(Volt) if hasattr(min_voltage, 'to') else float(min_voltage)
        
        max_current = (v_oc - v_min) / self.internal_resistance
        return Ampere(max(max_current, 0))
    
    def update_soc(self, current_draw: CurrentMeasurement, duration_seconds: float):
        """
        Update state of charge after current draw (dynamic mode).
        
        Uses Peukert's law for capacity derating at high currents.
        
        :param current_draw: Average current during period
        :param duration_seconds: Duration of current draw
        """
        current_a = abs(current_draw.to(Ampere) if hasattr(current_draw, 'to') else float(current_draw))
        
        if current_a < 0.1:
            return  # Negligible current
        
        # Peukert's law: effective capacity decreases at high currents
        # C_effective = C_rated * (I_ref / I)^(k-1)  where k is Peukert constant
        reference_current = 0.9  # 1C rate for 18Ah battery ≈ 0.9A
        peukert_factor = (reference_current / current_a) ** (self.peukert_constant - 1)
        effective_capacity_as = self.capacity_ah * 3600 * peukert_factor  # Amp-seconds
        
        # Charge consumed
        charge_consumed = current_a * duration_seconds
        soc_drop = charge_consumed / effective_capacity_as
        
        self.state_of_charge = max(0.0, self.state_of_charge - soc_drop)
        
        # Update OCV based on new SoC
        self.open_circuit_voltage = Volt(self._ocv_from_soc(self.state_of_charge))
    
    def is_brownout(self, current_draw: CurrentMeasurement) -> bool:
        """Check if current draw would cause brownout."""
        terminal_v = self.get_terminal_voltage(current_draw)
        return terminal_v.to(Volt) < self.brownout_voltage.to(Volt)
    
    def is_blackout(self, current_draw: CurrentMeasurement) -> bool:
        """Check if current draw would cause blackout."""
        terminal_v = self.get_terminal_voltage(current_draw)
        return terminal_v.to(Volt) < self.blackout_voltage.to(Volt)
