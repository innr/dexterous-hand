from perception.mediapipe_adapter import LandmarkError, normalize_landmarks

def test_normalize_media_pipe_points():
    points = [(i / 10, i / 20, 0.0) for i in range(21)]
    hand = normalize_landmarks(points, handedness="Right", timestamp_ns=10)
    assert len(hand.points) == 21
    assert len(hand.as_flattened()) == 63
    assert hand.points[3].x == 0.3

def test_landmark_validation():
    try:
        normalize_landmarks([(0.0, 0.0, 0.0)] * 20)
    except LandmarkError:
        pass
    else:
        raise AssertionError("wrong landmark count should fail")
