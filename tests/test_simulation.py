"""单指 MuJoCo 模型与目标轨迹的测试。"""

from pathlib import Path
from xml.etree import ElementTree

import numpy as np

from simulation.run_single_finger import MODEL_PATH, target_positions


def test_model_declares_four_joints_and_actuators() -> None:
    """模型应声明 4 个关节与 4 个位置执行器。"""
    root = ElementTree.parse(Path(MODEL_PATH)).getroot()
    assert len(root.findall("./worldbody//joint")) == 4
    assert len(root.findall("./actuator/position")) == 4


def test_trajectory_stays_inside_control_limits() -> None:
    """目标轨迹应始终落在各关节的控制限位内，且首尾闭合。"""
    samples = np.stack([target_positions(t) for t in np.linspace(0, 4, 101)])
    lower = np.array([-0.4, -0.1, 0.0, 0.0])
    upper = np.array([0.4, 1.3, 1.4, 1.1])
    assert np.all(samples >= lower)
    assert np.all(samples <= upper)
    np.testing.assert_allclose(samples[0], samples[-1], atol=1e-12)
