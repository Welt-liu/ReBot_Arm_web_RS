import pytest
from rebotarmcontroller.grip_haptics import GripHaptics, PositionWall, validate_settings


def contact(ident='red', left=1, right=1, x=0):
    return dict(id=ident, left_n=left, right_n=right, relative_mm=[x, 0, 0])


def update(engine, now, contacts=None, actual=25, target=25):
    contacts = [contact()] if contacts is None else contacts
    engine.update(force_n=sum(c['left_n'] + c['right_n'] for c in contacts), contacts=contacts,
                  actual_mm=actual, target_mm=target, now=now)


def test_requires_two_fingers_on_same_object_for_stable_grip():
    engine = GripHaptics()
    contacts = [contact(left=5, right=0), contact('blue', left=0, right=5)]
    update(engine, 0, contacts)
    update(engine, 1, contacts)
    assert engine.state == 'touch'
    assert engine.target_raw <= 45
    update(engine, 2)
    assert engine.state == 'touch'
    update(engine, 2.11)
    assert engine.state == 'gripping'
    assert engine.held == 'red'


def test_virtual_wall_and_closing_damping_then_opening_release():
    engine = GripHaptics({'opening_fraction': 0})
    update(engine, 0)
    update(engine, .11)
    update(engine, .13, target=23)
    assert engine.state == 'gripping'
    assert engine.spring_term == pytest.approx(1.8 * 8)
    assert engine.damping_term > 0
    update(engine, .15, target=30)
    assert engine.state == 'opening'
    assert engine.target_raw == 0
    assert engine.held is None


def test_hysteresis_preserves_light_contact():
    engine = GripHaptics()
    update(engine, 0)
    update(engine, .11, [contact(left=.2, right=.2)])
    assert engine.state == 'gripping'
    update(engine, .13, [contact(left=.1, right=.1)])
    assert engine.state == 'free'


@pytest.mark.parametrize('displacement', [True, False])
def test_slip_requires_held_object_and_persistent_displacement_or_contact_loss(displacement):
    engine = GripHaptics()
    update(engine, 0)
    update(engine, .11)
    changed = [contact(x=4)] if displacement else []
    update(engine, .13, changed)
    assert engine.state != 'slip'
    update(engine, .20, changed)
    assert engine.state == 'slip'
    assert engine.target_raw == 0  # Cue disabled by default.
    update(engine, .25, changed)
    assert engine.state == 'slip'
    update(engine, .42, changed)
    assert engine.state == ('gripping' if displacement else 'free')


def test_optional_slip_cue_is_bounded_and_brief():
    engine = GripHaptics({'slip_pulse_raw': 20, 'max_raw': 12})
    update(engine, 0)
    update(engine, .11)
    update(engine, .13, [])
    update(engine, .20, [])
    assert engine.target_raw == 12
    update(engine, .30, [])
    assert engine.state == 'slip'
    assert engine.target_raw == 0


def test_slew_limit_live_reduction_and_zero_release():
    engine = GripHaptics({'slew_raw_s': 100, 'smoothing_ms': 0})
    update(engine, 0, [contact(left=20, right=20)])
    assert engine.advance(0) == 2
    assert engine.advance(.02) == 4
    engine.configure({'max_raw': 1})
    assert engine.advance(.04) == 1
    update(engine, .06, [])
    assert engine.advance(.06) == 0


@pytest.mark.parametrize('setting', [{'max_raw': 151}, {'max_raw': 1.5}, {'force_gain': float('nan')},
                                    {'contact_off_n': .5}, {'full_scale_n': 1, 'force_deadband_n': 2},
                                    {'unknown': 1}, [], {'angle_scale': 0}])
def test_invalid_settings_rejected(setting):
    with pytest.raises(ValueError):
        validate_settings(setting)


def test_invalid_contact_or_opening_rejected():
    engine = GripHaptics()
    with pytest.raises(ValueError):
        update(engine, 0, [contact(left=float('nan'))])
    with pytest.raises(ValueError):
        update(engine, 0, actual=float('nan'))


def test_contact_boundary_is_fixed_even_when_gripper_is_compressed():
    engine = GripHaptics()
    update(engine, 0, actual=25, target=25)
    update(engine, .11, actual=23, target=22)
    assert engine.state == 'gripping'
    assert engine.wall_position_mm == 25
    assert engine.spring_term == pytest.approx(2.8 * 8)
    update(engine, .13, [])
    assert engine.wall_position_mm is None


def test_old_opening_fraction_cannot_retain_output_and_light_contact_has_hold():
    engine = GripHaptics({'opening_fraction': 1, 'force_gain': 0})
    update(engine, 0)
    assert engine.target_raw == 35
    engine.advance(0)
    update(engine, .02, target=30)
    assert engine.target_raw == engine.advance(.02) == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_wall_is_unilateral_releases_locally_and_rearms_at_same_boundary(direction):
    wall = PositionWall()
    wall.select('red', direction * 90, direction)
    def command(position):
        return wall.command(direction * position, 150, release_deg=.35, rearm_deg=.35)
    assert command(91) == 0  # Free side must never be pulled closed.
    assert command(90) == 150
    assert command(89) == 150
    wall.select('red', direction * 80, direction)
    assert wall.angle_deg == direction * 90
    assert command(89.4) == 0  # Opening while still inside the wall.
    assert wall.released
    assert command(89.4) == 0  # Stopping does not re-enable output.
    assert command(89.2) == 0  # Ignore tiny reversals/noise.
    assert command(89) == 150
    assert not wall.released
    assert command(91) == 0
    assert command(90.5) == 0  # Closing on the free side is still free.
    assert command(90) == 150
    wall.select(None, 0, direction)
    assert command(80) == 0


@pytest.mark.parametrize('direction', [-1, 1])
def test_position_quantization_never_commands_towards_closed_side(direction):
    wall = PositionWall()
    wall.select('red', -42, direction)
    assert direction * (wall.angle_deg + 42) >= 0
    assert abs(wall.angle_deg + 42) < 360 / 4096
