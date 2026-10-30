# generated_plans/

Plans saved from the dashboard's **Save to VRP** button, and listed in its
**Saved plans** dropdown. Served by `plan_store.py` through either server:

| Server | Started by | Endpoints |
|---|---|---|
| `app.py` (Flask, :8000) | `START_TESTBED.bat` | `/api/plans`, `/api/plans/<name>` |
| `plan_server.py` (:8082) | `start_dashboard.bat` | `/plans`, `/plans/<name>` |

Whichever is running answers; the page probes same-origin first, then :8082.

## Why this is not one of the other JSON folders

| Folder | Holds |
|---|---|
| **`generated_plans/`** | Full page exports **saved deliberately**. Hand-picked, meant to be re-run |
| `solved_plans/` | Every `plan_server` `/solve` result, archived automatically. Append-only and noisy |
| `runs/` | Records of runs that were actually executed |
| `fleet/routes/` | Per-robot route files for the supervisor. Not plans |

Keeping them apart is the point: a BRKGA solve worth minutes of CPU should not
be buried among hundreds of automatic archives.

Gitignored, like `runs/` and `solved_plans/`. This README is the exception so
the folder survives a fresh clone.

## Saving by hand

A plan dropped in here by any other means shows up in the dropdown too, as
long as it is a `.json` file whose top level has either `visits` or `routes`.
Names must be a bare filename of letters, digits, dot, dash and underscore.
