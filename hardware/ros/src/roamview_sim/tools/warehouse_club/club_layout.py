"""Warehouse club world: one layout, used for the plan drawing now and the Gazebo world later.

World frame: x east, y north, metres; origin = south-west inside corner of the building; floor at z = 0.
Everything here is a generic, unbranded warehouse-club layout at real scale (130 x 100 m sales building).
"""

W, D, H = 130.0, 100.0, 11.0          # inside length (x), depth (y), clear height under the joists
WALL = 0.3

# ---- doors in the outer walls: (wall, start, end, name) -------------------------------------
DOORS = [("S", 112.0, 118.0, "Entrance"), ("S", 22.0, 28.0, "Exit"),
         ("N", 113.0, 116.5, "Dock door 1"), ("N", 121.0, 124.5, "Dock door 2")]
DOCK_DROP = 1.22                       # dock height: floor to the truck apron outside the dock doors

# ---- front of store ---------------------------------------------------------------------------
ENTRY_MAT = (110.0, 120.0, 0.0, 4.0, 0.012)                    # x0, x1, y0, y1, height
SERVICE_DESK = (119.0, 128.5, 5.5, 7.0)                         # membership / returns counter
CART_CORRALS = [(100.0 + 2.2 * i, 1.0, 101.6 + 2.2 * i, 5.5) for i in range(4)]
CHECKOUTS = [(32.0 + 5.0 * i, 8.0) for i in range(14)]           # lane: counter 0.9 x 5.0 m from (x, y)
FOOD_TABLES = [(3.0 + 3.6 * i, 2.5 + 3.0 * j) for i in range(4) for j in range(3)]   # 1.2 x 0.8 m
FOOD_COUNTER = (0.3, 1.3, 3.0, 11.0)                            # counter along the west wall
RECEIPT_PODIUM = (26.0, 4.0)

# ---- aisles -----------------------------------------------------------------------------------
FRONT_RACETRACK = (5.0, 125.0, 14.0, 20.0)
BACK_RACETRACK = (5.0, 125.0, 78.0, 84.0)
CROSS_AISLE = (5.0, 125.0, 47.0, 51.0)

# ---- pallet racking: back-to-back double rows running north-south -------------------------------
RACK_DEPTH, AISLE, BAY, RACK_H = 2.4, 3.6, 2.7, 5.5             # double-row depth, aisle, bay length, height
ROW_SEGMENTS = [(22.0, 46.3), (51.7, 76.0)]                     # 9 bays of 2.7 m each
WEST_ROWS = [8.0 + (RACK_DEPTH + AISLE) * k for k in range(6)]  # x of each double row's west face
EAST_ROWS = [W - x - RACK_DEPTH for x in WEST_ROWS]             # mirror image
PERIMETER_RACKS = [(0.0, 1.2, 22.0, 76.0), (W - 1.2, W, 22.0, 76.0)]

# ---- centre of store: display tables (1.2 x 2.4 m, 0.8 m high) ------------------------------------
TABLE_SIZE, TABLE_H = (1.2, 2.4), 0.8
TABLE_ROWS_Y = [22.6, 27.2, 31.8, 36.4, 41.0, 52.2, 56.8, 61.4, 66.0, 70.6]
TABLES = [(48.2 + 3.6 * i, y) for i in range(10) for y in TABLE_ROWS_Y]

# ---- back of store ------------------------------------------------------------------------------
COOLER_WALL = (40.0, 90.0, 97.8, 99.7, 2.3)                     # glass-door fridges, x0 x1 y0 y1 height
FREEZER_CHESTS = [(42.0 + 6.0 * i, 88.5) for i in range(8)]      # 4.5 x 1.1 m, 0.9 m high
SERVICE_CASES = [(90.5 + 5.0 * i, 91.0) for i in range(4)]       # meat / bakery display cases 4.2 x 1.0 m
PREP_ROOM_WALL = (90.0, 110.0, 93.5)                             # wall behind the cases (x0, x1, y)
RECEIVING = (110.0, W, 84.0, D)                                  # walled room
RECEIVING_DOOR = (117.0, 121.0)                                  # opening in its south wall (y = 84)
OFFICE = (0.0, 22.0, 84.0, D)                                    # office and break room
OFFICE_DOOR = (86.0, 89.0)                                       # opening in its east wall (x = 22)
CHARGING_DOCK = (0.6, 92.0)                                      # against the west wall, AprilTag facing east

# ---- structure: columns sit inside rack rows where they can, 12 x 11-12 m grid --------------------
_rack_centres = [x + RACK_DEPTH / 2 for x in WEST_ROWS[::2]] + [x + RACK_DEPTH / 2 for x in EAST_ROWS[::2]]
COLUMNS = ([(x, y) for x in _rack_centres for y in (22.6, 34.2, 45.7, 52.3, 64.1, 75.4)]
           + [(x, y) for x in (57.8, 72.2) for y in (35.3, 64.9)]
           + [(30.0, 92.0), (60.0, 93.5), (100.0, 96.5)])

# ---- things RoamView should notice (test hazards) -------------------------------------------------
HAZARDS = [
    ("Pallet jack, forks down (75 mm)", (24.2, 38.0)),
    ("Empty pallet (144 mm)", (56.0, 49.0)),
    ("Wet-floor sign", (96.0, 16.0)),
    ("Abandoned cart", (99.8, 60.0)),
    ("Dock edges (1.22 m drop)", (118.8, 99.3)),
    ("Entrance mat (12 mm)", (115.0, 2.0)),
]

# ---- routes --------------------------------------------------------------------------------------
SPAWNS = {"dock": (1.6, 92.0, 0.0), "entrance": (115.0, 6.0, 1.5708), "receiving": (119.0, 90.0, 1.5708)}
PATROL = [(1.6, 92.0), (5.5, 92.0), (16.0, 87.5), (26.0, 87.5), (30.0, 81.0), (44.3, 81.0), (44.3, 49.0),
          (4.6, 49.0), (4.6, 17.0), (65.0, 17.0), (65.0, 81.0), (85.7, 81.0), (119.0, 81.0), (119.0, 94.0),
          (119.0, 81.0), (125.4, 81.0), (125.4, 17.0), (85.7, 17.0), (85.7, 49.0), (44.3, 49.0), (44.3, 17.0),
          (4.6, 17.0), (4.6, 81.0), (26.0, 81.0), (26.0, 87.5), (16.0, 87.5), (5.5, 92.0), (1.6, 92.0)]
SHOPPERS = [
    [(115.0, 3.0), (115.0, 17.0), (61.4, 17.0), (61.4, 49.0), (105.8, 49.0), (105.8, 17.0), (115.0, 3.0)],
    [(18.2, 17.0), (18.2, 81.0), (79.4, 81.0), (79.4, 17.0), (18.2, 17.0)],
    [(36.2, 24.0), (36.2, 74.0), (36.2, 24.0)],
    [(93.8, 24.0), (93.8, 74.0), (93.8, 24.0)],
    [(50.0, 86.5), (86.0, 86.5), (86.0, 81.5), (50.0, 81.5), (50.0, 86.5)],
    [(114.0, 90.0), (125.0, 90.0), (125.0, 96.0), (114.0, 96.0), (114.0, 90.0)],
    [(19.5, 2.5), (19.5, 12.5), (29.5, 12.5), (29.5, 2.5), (19.5, 2.5)],
    [(68.6, 40.0), (75.8, 40.0), (75.8, 60.3), (68.6, 60.3), (68.6, 40.0)],
]
