def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def map_range(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)