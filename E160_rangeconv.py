
    #return -r/1357 + 1027/1357
    #return -r/372.5 + 515.5/372.5
    #return -r/14.2 + 1033/14.2


def range2m_r0(r):
    return -0.000823444108115*r + 0.906382259505875


def range2m_r1(r):
    return -0.000880571715617*r + 0.945203856803847


def m2range_r0(x, scale=False):
    if scale:
        return 1.144881889763779*x*1000
    return max(0, -1.144881889763779*x*1000 + 1.0654*1000)


def m2range_r1(x, scale=False):
    if scale:
        return 1.103149606299212*x*1000
    return max(0, -1.103149606299212*x*1000 + 1.0569*1000)


def range2m(r, robot_id):
    maps = [range2m_r0(r), range2m_r0(r)]
    return maps[robot_id]


def m2range(x, robot_id, scale=False):
    maps = [m2range_r0(x, scale), m2range_r0(x, scale)]
    return maps[robot_id]
