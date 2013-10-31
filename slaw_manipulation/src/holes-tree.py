    if axis1 <= 39.051248:
        if axis2 <= 43.416587:
            return "M20"
        if axis2 > 43.416587:
            if axis1 <= 28.017851:
                return "F20_20"
            if axis1 > 28.017851:
                return "RV20"
    if axis1 > 39.051248:
        if axis1 <= 46.043458:
            return "M20_100"
        if axis1 > 46.043458:
            if axis1 <= 51.009803:
                return "S40_40"
