PORT_NAMES = [
    "GCC_RP2040",
    "GCC_POSIX",
]

PORT_PATHS = {
    "GCC_RP2040": "portable/ThirdParty/GCC/RP2040",
    "GCC_POSIX": "portable/ThirdParty/GCC/Posix",
}

def join_lists(xs):
    ret = []
    for x in xs:
        ret.extend(x)
    return ret
