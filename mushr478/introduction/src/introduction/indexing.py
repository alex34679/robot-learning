# from introduction.fibonacci import compute_fibonacci


def extract_fibonacci_rows(data):
    """Extract the rows of a numpy array that correspond to the Fibonacci numbers,
    using integer array indexing.

    >>> data = np.array([[ 4,  7,  8],
    ...                  [14, 17, 18],
    ...                  [24, 27, 28],
    ...                  [34, 37, 38]])
    >>> extract_fibonacci_rows(data)
    array([[ 4,  7,  8],
           [14, 17, 18],
           [14, 17, 18],
           [24, 27, 28],
           [34, 37, 38]])
    """
    # BEGIN QUESTION 3.1
    "*** REPLACE THIS LINE ***"
    # END QUESTION 3.1


def increment_rows_with_odd_first_element(data):
    """Add one to rows of a numpy array where the 0th column element is odd,
    using Boolean array indexing.

    NOTE: This function does not have a return value: your implementation
    should modify the data argument in-place.

    >>> data = np.array([[ 0,  1,  2],
    ...                  [ 3,  4,  5],
    ...                  [ 6,  7,  8],
    ...                  [ 9, 10, 11],
    ...                  [12, 13, 14],
    ...                  [15, 16, 17]])
    >>> increment_rows_with_odd_first_element(data)
    >>> data
    array([[ 0,  1,  2],
           [ 4,  5,  6],
           [ 6,  7,  8],
           [10, 11, 12],
           [12, 13, 14],
           [16, 17, 18]])
    """
    # BEGIN QUESTION 3.2
    "*** REPLACE THIS LINE ***"
    # END QUESTION 3.2
