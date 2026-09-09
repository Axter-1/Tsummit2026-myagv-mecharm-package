from home_service_behaviors import grasp_recovery


def test_safe_stop_and_stalled_are_recoverable_candidates():
    assert grasp_recovery.is_recoverable_approach_status('STALLED')
    assert grasp_recovery.is_recoverable_approach_status('SAFE_STOP')
    assert not grasp_recovery.is_recoverable_approach_status('BLOCKED')


def test_stop_distance_requires_the_calibrated_band():
    assert grasp_recovery.stop_distance_is_valid(0.090, 0.090, 0.025)
    assert not grasp_recovery.stop_distance_is_valid(0.199, 0.090, 0.025)
    assert not grasp_recovery.stop_distance_is_valid(0.439, 0.090, 0.025)


def test_scan_must_confirm_action_result():
    assert grasp_recovery.scan_agrees_with_result(0.092, 0.090, 0.030)
    assert not grasp_recovery.scan_agrees_with_result(0.130, 0.090, 0.030)


def test_chassis_clearance_must_be_finite_and_safe():
    assert grasp_recovery.chassis_clearance_is_valid(0.070, 0.070)
    assert not grasp_recovery.chassis_clearance_is_valid(0.062, 0.070)
    assert not grasp_recovery.chassis_clearance_is_valid(float('nan'), 0.070)


def test_clearance_can_be_read_from_safe_stop_diagnostic():
    message = 'Parada segura: despeje chasis=0.078 m, LiDAR-pared=0.201 m'
    assert grasp_recovery.chassis_clearance_from_message(message) == 0.078
    assert grasp_recovery.chassis_clearance_from_message('sin despeje') is None
