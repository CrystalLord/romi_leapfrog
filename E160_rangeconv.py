
def range2m(r):
    return -r/1357 + 1027/1357
    #return -r/372.5 + 515.5/372.5
    #return -r/14.2 + 1033/14.2

def m2range(x, scale=False):
    if not scale:
        return max(0, -1357*x + 1027)
    return 1357 * x
    # return max(515.5 - 372.5 * x, 0)
    #return 1033 - 14.2 * x