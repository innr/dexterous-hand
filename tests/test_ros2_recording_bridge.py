from recording.ros2_bridge import BridgeError, EpisodeBridge

def test_bridge_collects_commands_and_states():
    bridge = EpisodeBridge()
    values = [0.01 * i for i in range(16)]
    bridge.on_joint_command(values, timestamp_ns=10)
    bridge.on_joint_state(type("State", (), {"positions": values})(), timestamp_ns=20)
    episode = bridge.episode()
    assert episode["commands"][0]["timestamp_ns"] == 10
    assert tuple(episode["states"][0]["positions"]) == tuple(values)

def test_bridge_rejects_wrong_joint_count():
    bridge = EpisodeBridge()
    try:
        bridge.on_joint_command([0.0], timestamp_ns=1)
    except BridgeError:
        pass
    else:
        raise AssertionError("wrong joint count should fail")
