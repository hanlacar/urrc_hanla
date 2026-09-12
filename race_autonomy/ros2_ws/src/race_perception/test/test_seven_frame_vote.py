import pytest
from race_perception.traffic_light_color import SevenFrameVote


@pytest.mark.parametrize('frames,winner', [
    (['GREEN','RED','GREEN','UNKNOWN','GREEN','RED','GREEN'], 'GREEN'),
    (['RED']*4+['GREEN']*3, 'RED'),
    (['YELLOW']*4+['UNKNOWN']*3, 'YELLOW'),
    (['LEFT']*4+['GREEN']*3, 'LEFT'),
    (['GREEN']*3+['RED']*3+['UNKNOWN'], 'UNKNOWN'),
    (['UNKNOWN']*7, 'UNKNOWN'),
])
def test_exactly_seven_and_majority_including_unknown(frames, winner):
    vote = SevenFrameVote()
    vote.arm('stop1')
    for i, state in enumerate(frames):
        result = vote.update(state, i+1., i*.05)
        assert result['state'] == (winner if i == 6 else 'UNKNOWN')
    assert result['frames'] == 7


def test_duplicate_frames_and_new_stop_cannot_reuse_votes():
    vote = SevenFrameVote()
    vote.arm('first')
    for i in range(7):
        vote.update('GREEN', i+1., i*.05)
    assert vote.update('GREEN', 7., .4) is None
    vote.arm('second')
    assert vote.update('GREEN', 7., .5) is None
    assert vote.update('GREEN', 8., .55)['state'] == 'UNKNOWN'
    vote.arm('')
    assert vote.update('GREEN', 9., .6) is None


def test_gap_and_different_signal_reset_votes():
    vote = SevenFrameVote()
    vote.arm('stop')
    for i in range(6):
        vote.update('GREEN', i+1., i*.05, [0,0,10,10])
    result = vote.update('GREEN', 7., .3, [100,100,110,110])
    assert result['frames'] == 1
    assert vote.update('GREEN', 8., 1.)['frames'] == 1
