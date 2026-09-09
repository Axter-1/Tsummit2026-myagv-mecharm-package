from home_service_behaviors import grasp_recovery


def test_only_stalled_is_recoverable():
    assert grasp_recovery.is_recoverable_approach_status('STALLED')
    assert not grasp_recovery.is_recoverable_approach_status('BLOCKED')


def test_stop_distance_requires_the_calibrated_band():
    assert grasp_recovery.stop_distance_is_valid(0.090, 0.090, 0.025)
    assert not grasp_recovery.stop_distance_is_valid(0.199, 0.090, 0.025)
    assert not grasp_recovery.stop_distance_is_valid(0.439, 0.090, 0.025)


def test_scan_must_confirm_action_result():
    assert grasp_recovery.scan_agrees_with_result(0.092, 0.090, 0.030)
    assert not grasp_recovery.scan_agrees_with_result(0.130, 0.090, 0.030)
