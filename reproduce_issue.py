
from gamegine.utils.NCIM.ncim import Feet, Yard, Inch

def debug_units():
    f1 = Feet(1)
    f2 = Feet(2)
    y1 = Yard(1)
    
    sum_ft = f1 + f2
    
    print(f"Feet(1): {f1} (float: {float(f1)})")
    print(f"Feet(2): {f2} (float: {float(f2)})")
    print(f"Sum: {sum_ft} (float: {float(sum_ft)})")
    print(f"Yard(1): {y1} (float: {float(y1)})")
    
    print(f"Sum == Yard(1): {sum_ft == y1}")
    print(f"float(Sum) == float(Yard(1)): {float(sum_ft) == float(y1)}")
    
    diff = float(sum_ft) - float(y1)
    print(f"Diff: {diff}")
    
    # Division
    div_res = Feet(1) / Inch(1)
    print(f"Feet(1) / Inch(1): {div_res} (float: {float(div_res)})")
    print(f"Result == 12: {div_res == 12}")
    
if __name__ == "__main__":
    debug_units()
