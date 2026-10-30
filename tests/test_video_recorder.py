"""Per-run video recording: video_recorder.py and its app.py routes.

The camera stream and the H.264 encoder are replaced by fakes, so these run
anywhere in a couple of seconds and never touch the real data/videos folder.
"""
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import database  # noqa: E402
import video_recorder  # noqa: E402
from app import create_app  # noqa: E402

JPEG = cv2.imencode(".jpg", np.full((540, 960, 3), 90, np.uint8))[1].tobytes()


class FakeStream:
    """latest() hands out a new frame every call, like a live 10 fps feed."""

    def __init__(self, url, frames=True):
        self.url = url
        self.frames = frames
        self.seq = 0
        self.stopped = False
        self.error = None

    def latest(self):
        if not self.frames:
            return None, 0, None
        self.seq += 1
        return JPEG, self.seq, time.monotonic()

    def stop(self):
        self.stopped = True


class FakeWriter:
    def __init__(self, base_path, fps, size):
        self.path = base_path + ".mp4"
        self.codec = "h264"
        self.browser_playable = True
        self.size = size
        self.frames = []
        self._fh = open(self.path, "wb")

    def write(self, frame):
        assert frame.shape[1::-1] == self.size, "every frame matches the file"
        self.frames.append(frame)
        self._fh.write(b"\0" * 64)

    def release(self):
        self._fh.close()


def make_recorder(tmp_path, frames=True, **kw):
    db_path = str(tmp_path / "m.sqlite3")
    database.connect(db_path).close()
    opts = dict(fps=20.0, tail_sec=0.2, first_frame_wait_sec=0.2,
                source_factory=lambda url: FakeStream(url, frames),
                writer_factory=FakeWriter)
    opts.update(kw)
    return video_recorder.VideoRecorder(db_path, str(tmp_path / "videos"),
                                        stream_url="http://fake/stream",
                                        **opts)


def wait_idle(rec, timeout=5.0):
    end = time.monotonic() + timeout
    while rec.status()["recording"] is not None:
        assert time.monotonic() < end, "recording never stopped"
        time.sleep(0.02)


def new_run(rec, **values):
    conn = database.connect(rec.db_path)
    try:
        return database.create_run(conn, {"robot_count": 6,
                                          "traffic_method": "cbs", **values})
    finally:
        conn.close()


def video_rows(rec):
    conn = database.connect(rec.db_path)
    try:
        return [dict(r) for r in
                conn.execute("SELECT * FROM run_videos ORDER BY video_id")]
    finally:
        conn.close()


# ---- naming -----------------------------------------------------------------

def test_label_names_the_video_and_its_file():
    t = datetime(2026, 10, 17, 14, 30, 2)
    title, base = video_recorder.video_names(t, "R037 S1_6r PP", {}, "abcd1234ffff")
    assert title == "R037 S1_6r PP"
    assert base == "20261017-143002_R037_S1_6r_PP"


def test_unlabelled_name_comes_from_the_run():
    t = datetime(2026, 10, 17, 14, 30, 2)
    title, base = video_recorder.video_names(
        t, "", {"robot_count": 4, "traffic_method": "astar"}, "abcd1234ffff")
    assert base == "20261017-143002_4r_astar_abcd1234"
    assert title == "4 robots · A* · 2026-10-17 14:30"


def test_slug_strips_path_characters():
    assert video_recorder.slug("../../evil name?") == "evil_name"


def test_find_file_refuses_to_leave_the_video_folder(tmp_path):
    rec = make_recorder(tmp_path)
    (tmp_path / "outside.mp4").write_bytes(b"x")
    assert rec.find_file("../outside.mp4") is None


# ---- lifecycle ----------------------------------------------------------------

def test_run_is_recorded_from_start_to_end_plus_tail(tmp_path):
    rec = make_recorder(tmp_path)
    run_id = new_run(rec)
    rec.arm(run_id, "R001 S2_6r CBS")
    rec.on_run_started(run_id, "browser")
    time.sleep(0.3)
    assert rec.status()["recording"]["title"] == "R001 S2_6r CBS"
    rec.on_run_started(run_id, "supervisor", lag_sec=0.05)
    time.sleep(0.2)
    rec.on_run_ended(run_id, "RUN_COMPLETED")
    wait_idle(rec)
    (row,) = video_rows(rec)
    assert row["status"] == "ready"
    assert row["run_id"] == run_id
    assert row["stop_reason"] == "RUN_COMPLETED"
    assert row["offset_source"] == "supervisor"
    # The supervisor anchor replaced the browser one: about 0.3 s in,
    # minus the 0.05 s lag.
    assert 0.15 < row["run_t0_offset_sec"] < 0.5
    assert row["file_name"] == "R001_S2_6r_CBS_main.mp4"
    assert rec.find_file(row["file_name"])
    # Fixed-rate clock: video length tracks wall time (0.5 s + 0.2 s tail).
    assert 0.6 <= row["duration_sec"] <= 1.2
    assert row["camera_frames"] > 0


def test_browser_anchor_never_replaces_the_supervisor_one(tmp_path):
    rec = make_recorder(tmp_path)
    run_id = new_run(rec)
    rec.on_run_started(run_id, "supervisor")
    time.sleep(0.2)
    rec.on_run_started(run_id, "browser")
    assert rec.status()["recording"]["offset_sec"] == pytest.approx(0.0, abs=0.05)
    rec.stop(wait=True)


def test_auto_off_records_nothing(tmp_path):
    rec = make_recorder(tmp_path)
    rec.set_auto(False)
    rec.on_run_started(new_run(rec), "supervisor")
    assert rec.status()["recording"] is None
    assert video_rows(rec) == []
    # ...and the setting survives a restart of the app.
    assert make_recorder(tmp_path).auto is False


def test_no_camera_frames_is_a_failed_recording_with_no_file(tmp_path):
    rec = make_recorder(tmp_path, frames=False)
    run_id = new_run(rec)
    rec.on_run_started(run_id, "supervisor")
    time.sleep(0.4)
    rec.on_run_ended(run_id, "RUN_ABORTED")
    wait_idle(rec)
    (row,) = video_rows(rec)
    assert row["status"] == "failed"
    assert "camera stream" in row["error"]
    assert rec.find_file(row["file_name"]) is None


def test_manual_recording_ignores_run_end_but_takes_its_anchor(tmp_path):
    rec = make_recorder(tmp_path)
    run_id = new_run(rec)
    rec.start(label="hand test")
    time.sleep(0.2)
    rec.on_run_started(run_id, "supervisor")
    rec.on_run_ended(run_id, "RUN_COMPLETED")
    time.sleep(0.4)
    status = rec.status()["recording"]
    assert status is not None, "a manual recording stops only by hand"
    assert status["run_id"] == run_id
    assert status["offset_sec"] > 0.1
    rec.stop(wait=True)
    (row,) = video_rows(rec)
    assert row["status"] == "ready" and row["started_by"] == "manual"
    assert row["run_id"] == run_id


def test_sending_the_next_mission_closes_a_stale_recording(tmp_path):
    rec = make_recorder(tmp_path)
    first = new_run(rec)
    rec.on_run_started(first, "browser")     # its end event never arrives
    time.sleep(0.2)
    rec.arm(new_run(rec), "next")
    wait_idle(rec)
    (row,) = video_rows(rec)
    assert row["stop_reason"] == "next mission sent"


def test_a_new_run_starting_supersedes_the_old_recording(tmp_path):
    rec = make_recorder(tmp_path)
    first, second = new_run(rec), new_run(rec)
    rec.on_run_started(first, "browser")
    time.sleep(0.2)
    rec.on_run_started(second, "supervisor")
    assert rec.status()["recording"]["run_id"] == second
    rec.stop(wait=True)
    rows = video_rows(rec)
    assert [r["run_id"] for r in rows] == [first, second]
    assert rows[0]["stop_reason"].startswith("run ")


def test_max_duration_cap_stops_a_runaway_recording(tmp_path):
    rec = make_recorder(tmp_path, max_sec=0.4)
    rec.on_run_started(new_run(rec), "browser")
    wait_idle(rec)
    (row,) = video_rows(rec)
    assert "cap" in row["stop_reason"]
    assert row["duration_sec"] <= 0.6


def test_a_recording_left_open_by_a_crash_is_marked_interrupted(tmp_path):
    rec = make_recorder(tmp_path)
    conn = database.connect(rec.db_path)
    vid = database.create_video(conn, {"title": "t", "file_name": "x",
                                       "status": "recording"})
    conn.close()
    make_recorder(tmp_path)                 # the app starting up again
    conn = database.connect(rec.db_path)
    assert database.get_video(conn, vid)["status"] == "interrupted"
    conn.close()


def test_frames_carry_the_caption_strip(tmp_path):
    writers = []

    def factory(base, fps, size):
        writers.append(FakeWriter(base, fps, size))
        return writers[-1]

    rec = make_recorder(tmp_path, writer_factory=factory)
    rec.on_run_started(new_run(rec), "supervisor")
    time.sleep(0.2)
    rec.stop(wait=True)
    assert writers[0].size == (960, 540 + video_recorder.STRIP_PX)


# ---- app routes ---------------------------------------------------------------

@pytest.fixture()
def recording_app(tmp_path):
    app = create_app(db_path=str(tmp_path / "m.sqlite3"))
    app.config["TESTING"] = True
    rec = make_recorder(tmp_path)
    app.extensions["video_recorder"] = rec
    with app.test_client() as c:
        yield c, rec


def test_app_records_a_run_named_from_run_and_monitor(recording_app):
    c, rec = recording_app
    run_id = c.post("/api/runs", json={"robot_count": 6,
                                       "traffic_method": "pp",
                                       "video_label": "R010 S3_6r PP"}
                    ).get_json()["run_id"]
    assert c.patch(f"/api/runs/{run_id}",
                   json={"status": "running"}).status_code == 200
    events = [{"seq": 1, "elapsed_ms": 0, "event_type": "RUN_STARTED"},
              {"seq": 2, "elapsed_ms": 100, "event_type": "DEADLOCK_DETECTED",
               "details": {"robot_ids": ["Alvik1", "Alvik5"]}},
              {"seq": 3, "elapsed_ms": 150, "event_type": "COLLISION_AVOIDED",
               "details": {"robots": ["Alvik3", "Alvik4"], "gap_in": 2.33}}]
    c.post(f"/api/runs/{run_id}/events/batch", json={"events": events})
    status = c.get("/api/recorder").get_json()
    assert status["enabled"] and status["recording"]["title"] == "R010 S3_6r PP"
    time.sleep(0.3)
    c.post(f"/api/runs/{run_id}/events/batch", json={"events": [
        {"seq": 4, "elapsed_ms": 400, "event_type": "RUN_COMPLETED"}]})
    wait_idle(rec)

    videos = c.get("/api/videos").get_json()["videos"]
    assert len(videos) == 1
    v = videos[0]
    assert v["status"] == "ready" and v["file_exists"]
    assert v["robot_count"] == 6 and v["traffic_method"] == "pp"

    runs = c.get("/api/runs").get_json()["runs"]
    assert runs[0]["video_id"] == v["video_id"]

    # Range request: the player seeks without downloading everything.
    r = c.get(v["url"], headers={"Range": "bytes=0-9"})
    assert r.status_code == 206 and len(r.data) == 10
    assert r.mimetype == "video/mp4"
    dl = c.get(v["url"] + "?download=1")
    assert "R010_S3_6r_PP.mp4" in dl.headers["Content-Disposition"]

    m = c.get(f"/api/videos/{v['video_id']}/markers").get_json()
    kinds = [x["event_type"] for x in m["markers"]]
    assert kinds == ["RUN_STARTED", "DEADLOCK_DETECTED", "COLLISION_AVOIDED",
                     "RUN_COMPLETED"]
    dead = m["markers"][1]
    assert dead["category"] == "safety" and dead["robots"] == "Alvik1, Alvik5"
    assert dead["t_video"] == pytest.approx(m["offset_sec"] + 0.1, abs=0.01)
    assert "gap 2.33 in" in m["markers"][2]["text"]


def test_rename_and_delete_a_video(recording_app):
    c, rec = recording_app
    rec.start(label="scratch")
    time.sleep(0.2)
    assert c.delete("/api/videos/1").status_code == 409   # still recording
    c.post("/api/recorder/stop")
    r = c.patch("/api/videos/1", json={"title": "  keep me  "})
    assert r.get_json()["title"] == "keep me"
    assert c.patch("/api/videos/1", json={"title": " "}).status_code == 400
    path = rec.find_file(r.get_json()["file_name"])
    assert path and os.path.exists(path)
    assert c.delete("/api/videos/1").get_json()["file_removed"] is True
    assert not os.path.exists(path)
    assert c.get("/api/videos/1").status_code == 404


def test_manual_start_and_stop_routes(recording_app):
    c, rec = recording_app
    assert c.post("/api/recorder/start",
                  json={"run_id": "nope"}).status_code == 404
    assert c.post("/api/recorder/start", json={"label": "a"}).status_code == 200
    assert c.post("/api/recorder/start", json={"label": "b"}).status_code == 409
    out = c.post("/api/recorder/stop").get_json()
    assert out["recording"] is None and out["last"]["status"] == "ready"
    c.post("/api/recorder/settings", json={"auto": False})
    assert c.get("/api/recorder").get_json()["auto"] is False


def test_an_app_without_a_recorder_ignores_run_events(tmp_path):
    app = create_app(db_path=str(tmp_path / "m.sqlite3"))
    with app.test_client() as c:
        run_id = c.post("/api/runs", json={"video_label": "x"}
                        ).get_json()["run_id"]
        r = c.post(f"/api/runs/{run_id}/events/batch", json={"events": [
            {"seq": 1, "elapsed_ms": 0, "event_type": "RUN_STARTED"}]})
        assert r.status_code == 200
        assert c.get("/api/recorder").get_json()["enabled"] is False
        assert c.post("/api/recorder/start", json={}).status_code == 409
        assert c.get("/api/videos").get_json()["videos"] == []


# ---- second camera ----------------------------------------------------------------

class FakeSecond:
    """Stands in for overhead_recorder.py on the second camera's laptop."""

    url = "http://second:8090"

    def __init__(self, fail=None):
        self.fail = fail
        self.calls = []

    def start(self, name, stamp, file_core=""):
        self.calls.append(("start", name, stamp))
        if self.fail:
            raise RuntimeError(self.fail)
        core = file_core or video_recorder.slug(name)
        prefix = f"{stamp}_" if stamp else ""
        return {"recording": {"file": f"{prefix}{core}_second.mp4",
                              "elapsed_sec": 1.5, "first_frame_sec": 0.4}}

    def stop(self, file):
        self.calls.append(("stop", file))
        return {}


def test_second_camera_records_the_same_run_under_the_same_name(tmp_path):
    second = FakeSecond()
    rec = make_recorder(tmp_path, second_camera=second)
    run_id = new_run(rec)
    rec.arm(run_id, "R037 S1_6r CBS")
    rec.on_run_started(run_id, "supervisor")
    time.sleep(0.3)
    assert rec.status()["recording"]["second_state"] == "recording"
    rec.on_run_ended(run_id, "RUN_COMPLETED")
    wait_idle(rec)
    time.sleep(0.1)
    (row,) = video_rows(rec)
    kind, name, _stamp = second.calls[0]
    assert (kind, name) == ("start", "R037 S1_6r CBS")
    # Same name; only the camera tag differs.
    assert row["file_name"] == "R037_S1_6r_CBS_main.mp4"
    assert row["second_camera_file"] == "R037_S1_6r_CBS_second.mp4"
    assert second.calls[-1] == ("stop", row["second_camera_file"])
    assert row["second_camera_url"] == "http://second:8090"
    # Reply arrived at ~0 s on this clock; the second clip's 0:00 is
    # 1.5 - 0.4 = 1.1 s before that.
    assert row["second_offset_sec"] == pytest.approx(-1.1, abs=0.1)


def test_second_camera_failure_never_touches_the_main_video(tmp_path):
    rec = make_recorder(tmp_path, second_camera=FakeSecond(fail="unreachable"))
    run_id = new_run(rec)
    rec.on_run_started(run_id, "supervisor")
    time.sleep(0.3)
    rec.on_run_ended(run_id, "RUN_COMPLETED")
    wait_idle(rec)
    (row,) = video_rows(rec)
    assert row["status"] == "ready"
    assert row["second_camera_file"] is None
    assert row["second_error"] == "unreachable"


def test_second_camera_can_be_switched_off(tmp_path):
    second = FakeSecond()
    rec = make_recorder(tmp_path, second_camera=second)
    rec.set_second(False)
    rec.on_run_started(new_run(rec), "supervisor")
    time.sleep(0.2)
    rec.stop(wait=True)
    assert second.calls == []
    assert make_recorder(tmp_path).second_on is False     # remembered


def test_sync_nudge_is_saved(recording_app):
    c, rec = recording_app
    rec.start(label="nudge")
    time.sleep(0.2)
    c.post("/api/recorder/stop")
    assert c.patch("/api/videos/1", json={"second_offset_sec": "x"}).status_code == 400
    r = c.patch("/api/videos/1", json={"second_offset_sec": 0.4})
    assert r.get_json()["second_offset_sec"] == 0.4


def test_unnamed_run_files_still_match(tmp_path):
    second = FakeSecond()
    rec = make_recorder(tmp_path, second_camera=second)
    rec.on_run_started(new_run(rec), "supervisor")       # no label typed
    time.sleep(0.3)
    rec.stop(wait=True)
    time.sleep(0.1)
    (row,) = video_rows(rec)
    assert row["file_name"].replace("_main.mp4", "") ==         row["second_camera_file"].replace("_second.mp4", "")


# ---- names from the loaded plan -----------------------------------------------

def _record_one(rec, run_id, label=None, plan=None):
    rec.arm(run_id, label, plan)
    rec.on_run_started(run_id, "supervisor")
    time.sleep(0.25)
    rec.stop(wait=True)
    return video_rows(rec)[-1]


def test_videos_are_named_after_the_plan_with_a_repeat_number(tmp_path):
    second = FakeSecond()
    rec = make_recorder(tmp_path, second_camera=second)
    run1, run2, run3 = new_run(rec), new_run(rec), new_run(rec)
    rec.arm(run1, None, "DOE_6r_cbs.json")
    assert rec.status()["armed"]["next_name"] == "DOE_6r_cbs_1"
    a = _record_one(rec, run1, plan="DOE_6r_cbs.json")
    b = _record_one(rec, run2, plan="generated_plans/DOE_6r_cbs.json")
    c = _record_one(rec, run3, plan="DOE_4r_pp.json")
    assert (a["title"], a["file_name"]) == ("DOE_6r_cbs_1", "DOE_6r_cbs_1_main.mp4")
    assert b["file_name"] == "DOE_6r_cbs_2_main.mp4"
    assert c["file_name"] == "DOE_4r_pp_1_main.mp4"
    time.sleep(0.1)
    assert a["second_camera_file"] == "DOE_6r_cbs_1_second.mp4"


def test_numbering_continues_after_a_restart(tmp_path):
    rec = make_recorder(tmp_path)
    for _ in range(3):
        _record_one(rec, new_run(rec), plan="DOE_4r_wait.json")
    rec2 = make_recorder(tmp_path)          # the app restarted
    assert _record_one(rec2, new_run(rec2), plan="DOE_4r_wait.json")[
        "file_name"] == "DOE_4r_wait_4_main.mp4"


def test_a_typed_name_wins_and_repeats_get_a_number(tmp_path):
    rec = make_recorder(tmp_path)
    a = _record_one(rec, new_run(rec), label="SHAKE CBS", plan="DOE_6r_cbs.json")
    b = _record_one(rec, new_run(rec), label="SHAKE CBS")
    assert a["file_name"] == "SHAKE_CBS_main.mp4"
    assert b["file_name"] == "SHAKE_CBS_2_main.mp4"
    assert b["title"] == "SHAKE CBS"


def test_the_run_row_records_its_plan_file(recording_app):
    c, rec = recording_app
    run_id = c.post("/api/runs", json={"robot_count": 6, "traffic_method": "cbs",
                                       "plan_file": "DOE_6r_cbs.json"}
                    ).get_json()["run_id"]
    assert c.get(f"/api/runs/{run_id}").get_json()["plan_file"] == "DOE_6r_cbs.json"
    assert rec.status()["armed"]["next_name"] == "DOE_6r_cbs_1"
