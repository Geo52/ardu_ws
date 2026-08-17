# Captured occupancy grids

Real Cartographer maps saved from SITL runs, with the failure each one
contains. They exist because flying a hypothesis costs 15 minutes and
replaying one against a captured grid costs a second — every detector
and threshold decision in this package was made against these, and
several of the subtler bugs are invisible on synthetic grids because
they depend on how Cartographer's partially-observed band actually
behaves.

Each `.npy` is the raw `nav_msgs/OccupancyGrid` data reshaped to
`(height, width)`, `int8`, -1 unknown and 0..100 occupancy. The
matching `.json` carries `resolution`, `width`, `height`, `origin_x`,
`origin_y`, so cells convert to world coordinates.

```python
import json
import numpy as np
grid = np.load("run75_corridor_hidden.npy")
info = json.load(open("run75_corridor_hidden.json"))
```

| Fixture | What it holds |
|---|---|
| `run70_through_wall` | End of run 70. A 163-cell frontier at (7.76, -3.74) sits in already-mapped corridor pointing at unknown on the *far side* of the wall at y = -3.5. Reproduces the leak that comes from a dilation as wide as a wall, and disappears when the sight-line test uses `LOS_WALL_MIN` rather than `WALL_MIN`. Integration note 29. |
| `run75_corridor_hidden` | End of run 75, which landed leaving 50.4 m² of the southern corridor 97% unknown. At `unknown_dilation=3` the detector finds **1** cluster on the whole map and none into that corridor; at 4 it finds 12 and reaches it. The opposite failure to the one above, and the reason the reach is set to the wall thickness. Integration note 23. |
| `run72_midrun` + `run72_final` | Two grids from the *same* run, so they align in the map frame. Comparing what each fog cell became by the end is the measurement behind integration note 27: no occupancy threshold separates wall from free, because even cells at 80-89 resolve free about twice as often as wall. |
| `run78_complete` | A complete map (368.0 m², whole maze). Useful as the "nothing left to find" case — a detector change that invents frontiers here is inventing them. |

## Adding to these

Snapshot the live map with the grabber used to make these:

```python
# subscribe to /map, then
np.save(name + ".npy", np.asarray(msg.data, dtype=np.int8).reshape(
    msg.info.height, msg.info.width))
```

Save the grid whenever a run fails in a way you had to *watch* to
notice. The expensive part is never the disk, it is being airborne
when the interesting thing happens.
