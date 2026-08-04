from npu_sim_bringup.display_logic import DisplayConfig, validate_external_pose


CONFIG = DisplayConfig()
QUAT = (0.0, 0.0, 0.0, 1.0)


def test_valid_pose_accepted():
    assert validate_external_pose((1.0, -2.0, 1.5), QUAT, 'world', CONFIG) is None


def test_wrong_frame_rejected():
    reason = validate_external_pose((0.0, 0.0, 1.0), QUAT, 'map', CONFIG)
    assert reason is not None
    assert 'map' in reason


def test_non_finite_pose_rejected():
    assert validate_external_pose((float('nan'), 0.0, 1.0), QUAT, 'world', CONFIG)
    assert validate_external_pose((0.0, 0.0, float('inf')), QUAT, 'world', CONFIG)


def test_degenerate_quaternion_rejected():
    assert validate_external_pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 0.0),
                                  'world', CONFIG)


def test_out_of_bounds_rejected():
    assert validate_external_pose((0.0, 0.0, -0.5), QUAT, 'world', CONFIG)
    assert validate_external_pose((0.0, 0.0, 99.0), QUAT, 'world', CONFIG)
    assert validate_external_pose((99.0, 0.0, 1.0), QUAT, 'world', CONFIG)


def test_boundary_pose_accepted():
    assert validate_external_pose(
        (CONFIG.bounds_x[0], CONFIG.bounds_y[1], CONFIG.bounds_z[0]),
        QUAT, 'world', CONFIG) is None
