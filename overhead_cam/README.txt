OVERHEAD CAMERA RECORDER
Secondary camera laptop: industrious@192.168.0.73 (Linux)
Web page: http://192.168.0.73:8090/   (any machine on the lab network)

======================================================================
1. WHAT IT DOES
======================================================================

  Record       Type a name, press Record, press Stop.
  Name         The file is <date>-<time>_<name>.mp4, e.g.
               20261019-143002_R037_S1_6r_CBS.mp4. The name as you
               typed it is kept and shown in the list.
  Rename       Click a recording, edit its name, Save name. The file is
               renamed too (the date-time prefix stays).
  Preview      Live view of the camera, also shown while recording.
  Playback     Click a recording to play it in the browser: seek,
               0.5x-4x speed, Download, Delete.
  Storage      Only on that laptop, in ~/Videos/testbed/. Nothing is
               copied anywhere else.
  Old files    Anything already in ~/Videos/testbed/ (for example the
               6_robots_encoder+camera.mp4 recorded over ssh) shows up in
               the list as well.

  With runs    The main app (app.py on the Windows laptop) starts and
               stops this camera together with its own run recording,
               under the same name and date-time prefix:
                 20261019-143002_R037_S1_6r_CBS_main.mp4    (Windows)
                 20261019-143002_R037_S1_6r_CBS_second.mp4  (this laptop)
               Its Recordings page (tab 6) plays the two side by side,
               in sync. Turn it off there with "Second camera on/off" in
               Run & Monitor. A recording started by hand on this page is
               never stopped by the main app.

======================================================================
2. FILES
======================================================================

  overhead_recorder.py       the server (Python 3.8+ and ffmpeg only)
  overhead_recorder.html     the page it serves
  overhead-recorder.service  systemd user service
  ../tools/deploy_overhead_cam.sh   copies the above and restarts it

======================================================================
3. SETUP (once)
======================================================================

  On the Windows laptop, Git Bash, in the VRP folder:
    bash tools/deploy_overhead_cam.sh

  On the camera laptop, only if the deploy script says so:
    sudo apt install ffmpeg
    sudo loginctl enable-linger industrious
  (linger keeps the service running after you log out and starts it
  at boot)

  If the page does not open from another machine and ufw is on, run this
  on the camera laptop:
    sudo ufw allow 8090/tcp

======================================================================
4. RECORDING SETTINGS
======================================================================

  Same as the ssh command that worked:
    camera    /dev/video0, MJPEG, 1920x1080, 30 fps
    encode    libx264, preset veryfast, CRF 23, yuv420p, no audio
  Change any of them with arguments in the service ExecStart line:
    --device /dev/video2  --size 1280x720  --fps 30  --crf 23
    --preset veryfast  --dir ~/Videos/testbed  --port 8090
    --max-minutes 180    (a recording nobody stopped ends after this)

======================================================================
5. GOOD TO KNOW
======================================================================

  - The camera can be used by one program at a time. The live view
    holds it only while a page is showing it, and lets go 10 s after
    nobody is watching. Record takes the camera from the live view by
    itself. An ffmpeg started by hand over ssh will make Record fail
    with "Device or resource busy" until it is stopped.
  - Crash-safe: while recording, the file is <name>.part.mp4 written in
    2 s fragments. If the laptop, ffmpeg or the server dies, everything
    up to the last 2 s is kept and is recovered automatically the next
    time the server starts.
  - Stop takes a few seconds: the file is rewritten (no re-encode) so it
    seeks instantly in a browser.
  - No login. Anyone on the lab network can record, rename and delete.
  - Service control on the camera laptop:
      systemctl --user status overhead-recorder
      systemctl --user restart overhead-recorder
      journalctl --user -u overhead-recorder -n 50
  - Testing without the camera:
      python3 overhead_recorder.py --input "-re -f lavfi -i testsrc2=size=1920x1080:rate=30"
