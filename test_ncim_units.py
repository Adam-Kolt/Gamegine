from gamegine.utils.NCIM.Dimensions.spatial import Meter, Inch

try:
    val = Meter(Inch(13))
    print(f"Meter(Inch(13)) object: {val}")
    print(f"Meter(Inch(13)) value in Meters: {val.to(Meter)}")
    
    val2 = Inch(13).to(Meter)
    print(f"Inch(13).to(Meter): {val2}")

    if abs(val.to(Meter) - 13.0) < 0.001:
        print("FAIL: Meter(Inch(13)) preserves value 13.0 (Huge Robot Bug confirmed)")
    elif abs(val.to(Meter) - val2) < 0.001:
        print("PASS: Meter(Inch(13)) converts correctly")
    else:
        print(f"UNKNOWN: Meter(Inch(13)) = {val.to(Meter)}")

except Exception as e:
    print(f"Exception: {e}")
