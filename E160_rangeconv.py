
def range2m(r, robot_id):
    maps = [range2m_r0(r)]
    return maps[robot_id]
    #return -r/1357 + 1027/1357
    #return -r/372.5 + 515.5/372.5
    #return -r/14.2 + 1033/14.2


def range2m_r0(r):
    return -0.000823444108115*r + 0.906382259505875


def m2range_r0(x, scale=False):
    if scale:
        return -1.144881889763779*x
    return max(0, -1.144881889763779*x + 1.0654)


def m2range(x, robot_id, scale=False):
    maps = [m2range_r0(x, scale)]
    return maps[robot_id]

    #if not scale:
    #    return max(0, -1357*x + 1027)
    #return 1357 * x
    # return max(515.5 - 372.5 * x, 0)
    #return 1033 - 14.2 * x