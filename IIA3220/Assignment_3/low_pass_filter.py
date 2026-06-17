"""Module containing a low pass function filter function"""

def low_pass_filter(signal: float, last_filtered_signal: float, a: float = 0.1) -> float:
    """Low pass filter function filtering signals"""
    nr_digits = len(str(signal).rsplit(".", maxsplit=-1)[-1])
    return round((1 - a) * last_filtered_signal + a * signal, nr_digits)
