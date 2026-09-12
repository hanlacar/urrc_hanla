import math
from race_control.ramp_filter import RampPitchFilter
from race_control.course_mission import CourseMission, MissionInput


def test_iir_step_response_and_outlier():
    f=RampPitchFilter()
    f.update(0,0)
    f.update(5,.25)
    assert math.isclose(f.pitch,5*(1-math.exp(-1)))
    assert not f.update(80,.3)
    assert not f.valid
    assert f.update(4.8,.35)
    assert f.pitch==4.8


def run(values):
    m=CourseMission()
    history=[]
    for i,pitch in enumerate(values):
        m.update(MissionInput(section=2,now=i*.05,pitch_deg=pitch,
            imu_valid=True,odom_distance_valid=True,odom_distance_m=i*.025))
        history.append(m.ramp_crossing)
    return m,history


def test_bump_does_not_confirm_but_sustained_slope_does():
    _,history=run([0]*25+[12]*3+[0]*60)
    assert not any(history)
    m,history=run([0]*25+[4.8]*100)
    assert not any(history[:45])
    assert m.ramp_crossing


def test_rough_pitch_blocks_confirmation_and_then_recovers():
    m,history=run([0]*25+[10 if i%2 else 0 for i in range(100)])
    assert m.ramp_filter.roughness>m.ramp_filter.rough_limit
    assert not any(history)
    m,history=run([0]*25+[10 if i%2 else 0 for i in range(100)]+[4.8]*100)
    assert m.ramp_crossing


def test_filtered_dip_restarts_confirmation():
    m,history=run([0]*25+[4.8]*20+[0]*15+[4.8]*100)
    assert not any(history[:65])
    assert m.ramp_crossing
