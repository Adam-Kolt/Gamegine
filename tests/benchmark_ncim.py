
import time
import numpy as np
from gamegine.utils.NCIM.core import Measurement, Unit, Dimension

def benchmark():
    # Setup
    dims_m = np.zeros(len(Dimension))
    dims_m[Dimension.Spatial.value] = 1
    meter = Unit(dims_m, 1.0, "m")
    
    m1 = Measurement(10.0, meter)
    m2 = Measurement(20.0, meter)
    
    # 1. Simple Addition (Fast Path)
    start = time.time()
    for _ in range(100000):
        _ = m1 + m2
    end = time.time()
    print(f"Simple Addition (100k ops): {end - start:.4f}s")
    
    # 2. Multiplication (New Unit Creation)
    start = time.time()
    for _ in range(100000):
        _ = m1 * m2
    end = time.time()
    print(f"Multiplication (100k ops): {end - start:.4f}s")

    # 3. Float baseline
    f1 = 10.0
    f2 = 20.0
    start = time.time()
    for _ in range(100000):
        _ = f1 + f2
    end = time.time()
    print(f"Float Baseline (100k ops): {end - start:.4f}s")

if __name__ == "__main__":
    benchmark()
