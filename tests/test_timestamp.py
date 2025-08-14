from pd4_viewer import now_utc_seconds


def test_now_utc_seconds_no_decimals():
    ts = now_utc_seconds()
    assert '.' not in ts, ts
