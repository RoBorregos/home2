from speech.spatial.tracker import SpeakerTracker


def test_unsure_voice_from_known_direction_is_attributed():
    tr = SpeakerTracker()
    assert tr.update(0.0, "mario", True, -95.0) == ("mario", "voice")
    assert tr.update(2.0, None, False, -100.0) == ("mario", "direction")
    assert tr.update(2.5, None, False, 40.0) == ("unknown", "none")


def test_tracks_expire_and_follow_the_person():
    tr = SpeakerTracker()
    tr.update(0.0, "mario", True, 0.0)
    tr.update(3.0, "mario", True, 40.0)  # walked to the left
    assert abs(tr.tracks["mario"].azimuth_deg - 20.0) < 1e-6
    assert tr.update(4.0, None, False, 25.0) == ("mario", "direction")
    assert tr.update(20.0, None, False, 25.0) == ("unknown", "none")  # stale


def test_unreliable_doa_never_attributes_and_wraps():
    tr = SpeakerTracker()
    tr.update(0.0, "ana", True, 175.0)
    assert tr.update(1.0, None, False, -178.0) == ("ana", "direction")  # wraps
    assert tr.update(1.0, None, False, 175.0, doa_reliable=False) == ("unknown", "none")
