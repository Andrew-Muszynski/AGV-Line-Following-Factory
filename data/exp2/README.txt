EXPERIMENT 2 DATA ARCHIVE (joint/integrated versus sequential ALNS plans)
Archived November 9, 2026. Software: commit 294e29c (the exact supervisor used).

CONTENTS
- logs/  The 19 supervisor transcripts of the runs below, copied unchanged
         from the Linux laptop's runs/ folder: the 16 counted runs and the
         3 failed attempts made during collection.
- exp2_metrics.sqlite3.gz  The metrics database, restricted to the same 19
         runs (every table; metric_definitions in full). Decompress with
         gunzip, or with Python: gzip.open(...).
- ../../generated_plans/ALNS24_joint.json and ALNS24_seq.json  The two
         frozen plans.

COUNTED RUNS (pair, form, log prefix, metrics run_id prefix)
  1  I  20261103-1601  29d435fc      1  S  20261103-1607  de5c6e73
  2  S  20261103-1614  62a30c6e      2  I  20261103-1625  c5a7bc2f
  3  S  20261103-1632  b2812331      3  I  20261103-1638  bd8be1ce
  4  I  20261103-1644  6bd6a556      4  S  20261103-1702  3d9cf85e
  5  S  20261109-0859  257677b5      5  I  20261109-0915  463eb6de
  6  I  20261109-0927  e57700b4      6  S  20261109-0933  441743be
  7  S  20261109-0939  541fd681      7  I  20261109-0950  5e59c0de
  8  I  20261109-0955  a51a1f61      8  S  20261109-1002  50b2b365
  I = integrated (plan file ALNS24_joint), S = sequential (ALNS24_seq).

FAILED ATTEMPTS DURING COLLECTION (not counted; rerun in their slot)
  20261103-1650  S  d2e0e29b   collision reported (0.03 in), bay 1
  20261109-0908  I  9511a414   yield return and exclusion at bay 9
  20261109-0921  I  d0730c7f   the same fault at bay 9

REPRODUCING THE THESIS TABLES
- Table 5.6 and 5.7: actual_makespan_sec and the other metric_values rows
  of the counted run_ids; planned makespans from the plan files
  (traffic_metrics.planned_makespan_sec). The paired tests use the eight
  pairs of actual makespans.
- Table 5.9:  python3 tools/exp2_critical_robot.py --table
  One run interval by interval (the worked example of Appendix D):
              python3 tools/exp2_critical_robot.py --example 7I
