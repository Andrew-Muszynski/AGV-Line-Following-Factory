import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np

# Grid parameters
GRID_SIZE = 10        # 10x10 cells -> 11x11 intersections (~121 nodes)
NUM_DEPOTS = 3        # number of depot cells
NUM_HUMANS = 15       # number of human workstation nodes
SEED = 42

rng = np.random.default_rng(SEED)

# Randomly select depot cells (row, col) in grid cell space
depot_cells = set()
while len(depot_cells) < NUM_DEPOTS:
    r = rng.integers(0, GRID_SIZE)
    c = rng.integers(0, GRID_SIZE)
    depot_cells.add((r, c))

fig, ax = plt.subplots(figsize=(8, 8))

# Draw depot cells first (black filled squares)
for (r, c) in depot_cells:
    rect = patches.Rectangle(
        (c, r), 1, 1,
        linewidth=0,
        facecolor='black',
        zorder=1
    )
    ax.add_patch(rect)
    # Label depot at cell center
    ax.text(
        c + 0.5, r + 0.5, 'D',
        color='white', fontsize=11, fontweight='bold',
        ha='center', va='center', zorder=3
    )

# Draw grid lines (robot travel paths)
for i in range(GRID_SIZE + 1):
    ax.plot([0, GRID_SIZE], [i, i], color='#888888', linewidth=0.8, zorder=2)
    ax.plot([i, i], [0, GRID_SIZE], color='#888888', linewidth=0.8, zorder=2)

# All intersection nodes
all_nodes = [(c, r) for r in range(GRID_SIZE + 1) for c in range(GRID_SIZE + 1)]

# Randomly select human workstation nodes (avoid depot cell corners for clarity)
human_nodes = set()
shuffled = rng.permutation(len(all_nodes))
for idx in shuffled:
    if len(human_nodes) >= NUM_HUMANS:
        break
    human_nodes.add(all_nodes[idx])

human_set = set(human_nodes)

# Separate regular vs human nodes
reg_xs = [x for (x, y) in all_nodes if (x, y) not in human_set]
reg_ys = [y for (x, y) in all_nodes if (x, y) not in human_set]
hum_xs = [x for (x, y) in human_set]
hum_ys = [y for (x, y) in human_set]

# Draw regular intersection nodes
ax.scatter(reg_xs, reg_ys, s=30, color='steelblue', zorder=4,
           label=f'Intersection nodes ({len(all_nodes) - NUM_HUMANS})')

# Draw human workstation nodes — larger orange circle with "N" inside
ax.scatter(hum_xs, hum_ys, s=180, color='darkorange', zorder=5,
           edgecolors='black', linewidths=0.8,
           label=f'Human workstations ({NUM_HUMANS})')
for (x, y) in human_set:
    ax.text(x, y, 'N', fontsize=6, fontweight='bold', color='white',
            ha='center', va='center', zorder=6)

# Bounding box
for spine in ax.spines.values():
    spine.set_linewidth(2)

ax.set_xlim(-0.3, GRID_SIZE + 0.3)
ax.set_ylim(-0.3, GRID_SIZE + 0.3)
ax.set_aspect('equal')
ax.set_xticks(range(GRID_SIZE + 1))
ax.set_yticks(range(GRID_SIZE + 1))
ax.tick_params(labelsize=8)
ax.set_xlabel('X (grid units)', fontsize=11)
ax.set_ylabel('Y (grid units)', fontsize=11)
total_nodes = len(all_nodes)
ax.set_title(
    f'Warehouse Grid — {total_nodes} Nodes, {NUM_DEPOTS} Depots, {NUM_HUMANS} Human Workstations\n'
    f'Grid lines = robot travel paths',
    fontsize=12
)

# Legend
depot_patch = patches.Patch(color='black', label=f'Depot cells ({NUM_DEPOTS})')
node_handle = plt.Line2D([0], [0], marker='o', color='w',
                          markerfacecolor='steelblue', markersize=7,
                          label=f'Intersection nodes ({total_nodes - NUM_HUMANS})')
human_handle = plt.Line2D([0], [0], marker='o', color='w',
                           markerfacecolor='darkorange', markersize=10,
                           markeredgecolor='black', markeredgewidth=0.8,
                           label=f'Workstations ({NUM_HUMANS})')
ax.legend(handles=[depot_patch, node_handle, human_handle],
          loc='upper right', fontsize=9)

plt.tight_layout()
plt.savefig('warehouse_grid_workstations.png', dpi=150, bbox_inches='tight')
plt.show()
print(f"Saved: warehouse_grid_workstations.png")
print(f"Total nodes: {total_nodes}")
print(f"Workstations: {sorted(human_set)}")
print(f"Depot cells: {sorted(depot_cells)}")
