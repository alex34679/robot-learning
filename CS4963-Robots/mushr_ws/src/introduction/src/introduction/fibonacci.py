def compute_fibonacci(n):
    """Return the nth Fibonacci number.

    >>> compute_fibonacci(0)
    0
    >>> compute_fibonacci(1)
    1
    >>> compute_fibonacci(2)  # 0 + 1
    1
    >>> compute_fibonacci(3)  # 1 + 1
    2
    >>> compute_fibonacci(4)  # 1 + 2
    3
    """
    # BEGIN QUESTION 1.1
    if n == 0:
        return 0
    exPrevNum: int = 0
    prevNum: int = 1
    for x in range(1, n):
        tmp: int = exPrevNum + prevNum
        exPrevNum = prevNum
        prevNum = tmp
    return prevNum
    # END QUESTION 1.1

print(compute_fibonacci(10))