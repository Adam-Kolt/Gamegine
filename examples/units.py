from gamegine.utils.NCIM.ComplexDimensions.electricpot import Volt
from gamegine.utils.NCIM.ComplexDimensions.omega import RotationsPerSecond
from gamegine.utils.NCIM.ComplexDimensions.velocity import MetersPerSecond
from gamegine.utils.NCIM.basic import ComplexDimension
from gamegine.utils.logging import Debug


Debug(f"Voltage: {Volt(1)}")
Debug(f"Rotational Speed: {RotationsPerSecond(502.1)}")

Debug
Debug(f"Rotatational Speed per Volt: {RotationsPerSecond(502.1) / Volt(1)}")


Debug(f"MetersPerSecond: {MetersPerSecond(1)}")

Debug(f"Get Units: {MetersPerSecond.dimension}")
print(ComplexDimension([23, 123, 5]))
