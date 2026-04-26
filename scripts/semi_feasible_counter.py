"""
This program computes the number of semi-feasible solutions for a
given number of requests (n) and vehicles (m) in a vehicle routing
problem. The function `compute_S` calculates the number of ways to
distribute n requests among m vehicles, considering the constraints
of the problem.
"""

import math
from functools import lru_cache

def compute_S(n, m):
    # Precompute factorials up to 2n
    fact = [1] * (2*n + 1)
    for i in range(1, 2*n + 1):
        fact[i] = fact[i-1] * i

    # Recursive generation of compositions
    @lru_cache(None)
    def helper(remaining, parts):
        if parts == 1:
            k = remaining
            return fact[n] // fact[k] * (fact[2*k] // (2**k))
        
        total = 0
        for k in range(remaining + 1):
            term = (fact[2*k] // (2**k)) / fact[k]
            total += term * helper(remaining - k, parts - 1)
        return total

    return int(helper(n, m))

def main():
    n = int(input("Enter number of requests (n): "))
    m = int(input("Enter number of vehicles (m): "))

    result = compute_S(n, m)
    print(f"S = {result:.6e}")

if __name__ == "__main__":
    main()