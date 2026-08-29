from integration.full_stack import JOINT_COUNT, VirtualTransport, replay, run_episode

def landmarks():
    return {"synthetic": True}

def retarget(_landmarks):
    return [0.01 * i for i in range(JOINT_COUNT)]

def test_full_stack_command_state_and_replay():
    bus = VirtualTransport()
    episode = run_episode(landmarks(), retarget, bus)
    assert len(episode.commands[0]) == JOINT_COUNT
    assert episode.states[0].positions == tuple(0.01 * i for i in range(JOINT_COUNT))
    replayed = replay(episode, VirtualTransport())
    assert replayed.commands == episode.commands
    assert replayed.states == episode.states

def test_timeout_disables_torque():
    bus = VirtualTransport(timeout_after=0)
    try:
        run_episode(landmarks(), retarget, bus)
    except TimeoutError:
        pass
    else:
        raise AssertionError("timeout was expected")
    assert bus.torque_enabled is False
