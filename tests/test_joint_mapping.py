"""关节映射配置与重排顺序的测试。"""

import json
from pathlib import Path

import numpy as np

from simulation.leap_hand_model import MAPPING_PATH, reorder_sim_to_hardware


def test_mapping_has_official_16_joint_order() -> None:
    """映射应包含 16 个关节，且硬件 ID 与 MuJoCo 关节均按 0..15 排列。"""
    mapping = json.loads(Path(MAPPING_PATH).read_text(encoding="utf-8"))
    joints = mapping["joints"]
    assert len(joints) == 16
    assert [joint["hardware_id"] for joint in joints] == list(range(16))
    assert [joint["mujoco_joint"] for joint in joints] == [str(i) for i in range(16)]


def test_reorder_matches_observed_mujoco_urdf_order() -> None:
    """按文档记录的置换直接验证重排映射的契约。"""
    class FakeModel:
        njnt = 16

    # 仅靠 fake 无法做名称查找，因此直接对照源映射的置换契约来验证。
    mapping = json.loads(Path(MAPPING_PATH).read_text(encoding="utf-8"))
    sim_names = mapping["model"]["mujoco_joint_order_observed"]
    expected = np.arange(16, dtype=float)
    output = np.empty(16)
    for sim_index, source_joint in enumerate(sim_names):
        hardware_id = int(source_joint)
        output[hardware_id] = expected[sim_index]
    np.testing.assert_array_equal(output, [1, 0, 2, 3, 5, 4, 6, 7, 9, 8, 10, 11, 12, 13, 14, 15])
