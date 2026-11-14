"""User's circle/angle model, inverse command and additive baseline invariants."""
import math
import pytest
from janggi_pick_place2.geometry import (command_from_width,width_from_command,
    height_from_command,tcp_length,tcp_length_from_command,flange_pose)
from janggi_pick_place2.executor import Settings


def test_user_supplied_coefficients_and_reference_are_used_directly():
    assert width_from_command(0)==pytest.approx(105.89999955703884)
    assert width_from_command(700)==pytest.approx(6.129353399605171)
    assert height_from_command(0)==pytest.approx(0.,abs=1e-6)
    assert height_from_command(700)==pytest.approx(27.696400707520077)
    for c in (0,100,350,600,700):
        theta=.532602+.00151819*c
        assert width_from_command(c)==pytest.approx(2*(4.446124+56.302390*math.cos(theta)))
        assert height_from_command(c)==pytest.approx(-28.589039+56.302390*math.sin(theta))
        assert command_from_width(width_from_command(c))==c
        assert tcp_length_from_command(c)==pytest.approx(132.6717+height_from_command(c))


def test_inverse_command_quantization_and_endpoints():
    assert command_from_width(6)==700 and command_from_width(106)==0
    for w in (20.,26.,32.,30.,36.,42.):
        c=command_from_width(w)
        assert abs(width_from_command(c)-w)<.09
        assert tcp_length(w)==pytest.approx(132.6717+height_from_command(c))


def test_parameter_is_constant_while_effective_extension_changes():
    settings=Settings()
    assert settings.tcp_length_at_6mm_mm==132.6717
    for w in (20.,30.,40.,105.9):
        pose=flange_pose([400,50,-18,0],w,settings.tcp_length_at_6mm_mm)
        assert pose[2]==pytest.approx(-18+132.6717+height_from_command(command_from_width(w)))
        assert settings.tcp_length_at_6mm_mm==132.6717
    assert tcp_length(20)>tcp_length(40)
    with pytest.raises(ValueError):tcp_length(20,160)
