"""Frame continuity checks must not turn incomplete captures into successful scans."""
from tools.diagnostics.record_room_rgbd import FrameStats, assess_playback
from tools.diagnostics import record_room_rgbd as recording


def frames(count=180, first=0):
    stats = FrameStats()
    for n in range(first, first + count):
        stats.add(n, n * 1000 / 30)
    return stats


def test_reused_synchronizer_frame_is_not_counted_twice():
    stats = FrameStats()
    for n in range(180):
        stats.add(n, n * 1000 / 30)
        stats.add(n, n * 1000 / 30)
    result = stats.report()
    assert result["frames"] == 180
    assert result["missing_frame_numbers"] == 0
    assert not assess_playback({"color": result, "depth": result}, 6, 6)


def test_large_gap_is_reported_even_when_average_rate_looks_good():
    stats = FrameStats()
    for n in range(900):
        if not 300 <= n < 309:
            stats.add(n, n * 1000 / 30)
    result = stats.report()
    assert result["observed_fps"] > 29
    assert result["missing_frame_numbers"] == 9
    assert "color: frame timing needs review" in assess_playback({"color": result, "depth": result}, 30)


def test_missing_depth_and_early_eof_are_not_success():
    issues = assess_playback({"color": frames().report(), "depth": FrameStats().report()}, 6, 180)
    assert "depth: incomplete duration" in issues
    assert "recording ended earlier than expected" in issues


def test_one_stream_cut_short_is_reported():
    issues = assess_playback({"color": frames(300).report(), "depth": frames().report()}, 10)
    assert "depth: incomplete duration" in issues


def test_camera_timestamp_reset_is_reported():
    stats = frames()
    stats.add(1, 10)
    result = stats.report()
    assert result["nonmonotonic"] == 1
    assert assess_playback({"color": result, "depth": result}, 6)


def test_playback_initial_stopped_state_is_not_treated_as_eof(monkeypatch, tmp_path):
    """The SDK starts asynchronously and can still report STOPPED after sensor.start()."""
    from datetime import timedelta
    from types import SimpleNamespace as NS

    callbacks = []

    class Sensor:
        def __init__(self, name):
            self.profile = NS(stream_type=lambda: name, width=lambda: 1280, height=lambda: 720,
                              fps=lambda: 30, format=lambda: "bgr8" if name == "color" else "z16")
            self.profile.as_video_stream_profile = lambda: self.profile

        def get_stream_profiles(self):
            return [self.profile]

        def open(self, profiles):
            pass

        def start(self, callback):
            callbacks.append((self.profile, callback))

        def stop(self):
            pass

        def close(self):
            pass

    playback = NS(set_real_time=lambda value: None, get_duration=lambda: timedelta(seconds=6),
                  current_status=lambda: "stopped", query_sensors=lambda: [Sensor("color"), Sensor("depth")])
    device = NS(as_playback=lambda: playback)
    sdk = NS(context=lambda: NS(load_device=lambda path: device),
             stream=NS(color="color", depth="depth"), format=NS(bgr8="bgr8", z16="z16"),
             playback_status=NS(stopped="stopped"))

    def complete_pending_read(_):
        for profile, callback in callbacks:
            for n in range(180):
                callback(NS(get_profile=lambda: profile, get_frame_number=lambda: n,
                            get_timestamp=lambda: n * 1000 / 30))

    monkeypatch.setattr(recording.time, "sleep", complete_pending_read)
    bag = tmp_path / "recording.bag"
    bag.write_bytes(b"test")
    result = recording.verify(bag, sdk, 6)
    assert result["status"] == "pass"
    assert result["streams"]["color"]["frames"] == 180
    assert result["streams"]["depth"]["frames"] == 180
