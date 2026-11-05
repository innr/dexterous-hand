from dataclasses import replace
from simulation.s6_command_contract import JointCommandData, validate_command

def rejected(fn):
    try:
        fn()
    except ValueError:
        return
    raise AssertionError("unsafe command accepted")

def main():
    names = [f"joint_{i}" for i in range(16)]
    cmd = JointCommandData(100000, 2, 8, tuple(names[::-1]),
                           tuple(float(i) / 100 for i in range(16)))
    def check(c, now=100000, last=7):
        return validate_command(c, names=names, lower=[-0.5]*16,
                                upper=[0.5]*16, epoch=2,
                                last_sequence=last, sim_time_us=now)
    assert check(cmd) == [float(i)/100 for i in range(15,-1,-1)]
    rejected(lambda: check(replace(cmd, epoch=1)))
    rejected(lambda: check(replace(cmd, sequence=7)))
    rejected(lambda: check(replace(cmd, sequence=6)))
    rejected(lambda: check(replace(cmd, stamp_us=0), now=100001))
    rejected(lambda: check(replace(cmd, stamp_us=110001)))
    rejected(lambda: check(replace(cmd, names=tuple(names[:15]+names[:1]))))
    rejected(lambda: check(replace(cmd, positions=tuple([float("nan")]*16))))
    rejected(lambda: check(replace(cmd, positions=tuple([1.0]*16))))
    print("S6-03 command validation passed")

if __name__ == "__main__":
    main()
