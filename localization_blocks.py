import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import convolve2d

def sense_u(world, mask, p, SenVal, heading ):
    """
    Models sensor reading update for localization.
    
    Parameters:
        world : np.ndarray
            Ultrasonic map or sensor reading world.
        mask : np.ndarray
            Binary mask of valid regions (1 = free, 0 = blocked).
        p : np.ndarray
            Current probability distribution.
        SenVal : int or float
            Current sensor measurement value.
    Returns:
        pnew : np.ndarray
            Updated and normalized probability map.
    """
    # Probabilities for sensor accuracy
    pHit = 0.9
    pMiss = 0.1
    threshold = 15.54  # in cm (6 inches)
    
    #Claasify into zone type based on sensor value
    u0, u1, u2, u3, u4 = SenVal
    close = [u < threshold for u in [u0, u1, u2, u3]]
    close_count = sum(close)

    steps = int((heading % 360) / 90)
    rotated_close = np.roll(close, steps)
    
    if close_count == 0:
        sensor_zone = 0  # free
    elif (close[0] and close[2]) or (close[1] and close[3]):
        sensor_zone = 5  # opposite blocking
    elif ((close[0] and close[1]) or (close[1] and close[2]) or
          (close[2] and close[3]) or (close[3] and close[0])):
        sensor_zone = 2  # adjacent blocking
    elif close_count == 1:
        sensor_zone = 3  # single side blocking
    else:
        sensor_zone = 1  # general blocking (catch‐all)

    # Define expected obstacle patterns per zone type (customize as needed)
    expected = {
        0: np.array([0, 0, 0, 0]),  # free
        1: np.array([1, 0, 0, 0]),  # partial (e.g. wall behind)
        2: np.array([1, 1, 0, 0]),  # adjacent (e.g. corner)
        3: np.array([0, 0, 1, 0]),  # single side (front)
        4: np.array([1, 1, 1, 1]),  # obstacle (ignored)
        5: np.array([1, 0, 1, 0])   # opposite
    }
    
    # Create multiplier matrix
    mult = np.full_like(world, pMiss, dtype=float) #full like creates a new array with the same shape and type as a given array, filled with a specified value
    #mult[world == sensor_zone] = pHit
    for zone_val, pattern in expected.items():
        dist = np.sum(np.abs(rotated_close - pattern))
        # The smaller the distance, the higher the match weight
        weight = pHit if dist == 0 else (0.5 if dist == 1 else pMiss)
        mult[world == zone_val] = weight

    # Apply sensor update
    pnew = p * mult

    # Apply mask
    pnew[mask ==4] = 0  # set probabilities to zero where mask indicates obstacle

    # Normalize (avoid divide-by-zero)
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = (mask != 4).astype(float)
        pnew = np.ones_like(pnew) / np.size(pnew)

    return pnew



def move(p, mask, heading, Move):
    """
    Models probabilistic movement and heading update.

    Parameters:
        p : np.ndarray
            Current probability map.
        mask : np.ndarray
            Binary mask for valid regions (1 = free, 0 = blocked).
        heading : float
            Current heading angle in degrees.
        Move : str
            Movement command: 'F', 'L', 'B', 'R'.

    Returns:
        pnew : np.ndarray
            Updated probability map.
        heading : float
            Updated heading after rotation.
    """

    # Movement error kernel (same as MATLAB's K)
    K = np.array([
        [0.1, 0.1, 0.1],
        [0.1, 0.8, 0.1],
        [0.1, 0.1, 0.1]
    ])

    # 2D convolution with same size output
    pnew = convolve2d(p, K, mode='same', boundary='fill', fillvalue=0)

    print(f"heading: {heading}")

    # --- DIRECTION HANDLING ---
    if Move == 'L':
        heading = (heading + 90) % 360
    elif Move == 'R':
        heading = (heading - 90) % 360
    elif Move == 'B':
        heading = (heading + 180) % 360
    # 'F' means forward → no change to heading
    
    rad = np.deg2rad(heading + 180)
    col_step = int(np.round(np.cos(rad)))  # -1, 0, or 1
    row_step = int(np.round(-np.sin(rad)))  # -1, 0, or 1

    step_fraction = 1  # 5" / 12.5"

    # Column shift
    if col_step == 1:
        shifted = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif col_step == -1:
        shifted = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # Row shift
    if row_step == 1:
        shifted = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif row_step == - 1:
        shifted = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted


    # # Determine movement direction
    # col_move = np.cos(np.deg2rad(heading))
    # row_move = np.sin(np.deg2rad(heading))

    # # --- COLUMN (X) MOVEMENT ---
    # step_fraction = 0.5#rover moves 5 inches = 0.4 of a cell (12.5 inches)
    
    # if col_move > 0:
    #     # Move right
    #     shifted = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
    #     pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    # elif col_move < 0:
    #     # Move left
    #     shifted = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])
    #     pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # # --- ROW (Y) MOVEMENT ---
    # if row_move < 0:
    #     # Move up
    #     shifted = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
    #     pnew = (1 - step_fraction) * pnew + step_fraction * shifted
        
    # elif row_move > 0:
    #     # Move down
    #     shifted = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
    #     pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # Apply mask
    #pnew = pnew * mask
    pnew[mask ==4] = 0  # set probabilities to zero where mask indicates obstacle

    # Normalize
    total = np.sum(pnew)
    if total > 0:
        pnew /= total
    else:
        pnew = (mask != 4).astype(float)
        pnew = np.ones_like(pnew) / np.size(pnew)

    print(f"move: {Move} heading: {heading}")

    return pnew, heading

# # Test movement directions
# p_test = np.zeros((16, 32))
# p_test[8, 16] = 1  # start roughly in center
# M_test = np.ones_like(p_test)
# headings = [0, 90, 180, 270]

# for h in headings:
#     p_move, _ = move(p_test, M_test, h, 'F')
#     plt.imshow(p_move, cmap='hot', origin='upper')
#     plt.title(f"Heading {h}° (should move: 0°=Right, 90°=Down, 180°=Left, 270°=Up)")
#     plt.pause(1)
# plt.show()


# --- INITIAL SETUP ---

# Heading (degrees) //get from gyroscope
heading = 180 # 270° implies downward, 90 is up, 0 is left, 180 is right

# Provide measurements and movements 
m_u = [7.55904, 69.39280000000001, 37.932359999999996, 123.62180000000001, 7.574280000000001, 7.05358, 70.45706, 41.32326, 124.11710000000001, 137.09395999999998, 8.382, 22.85492, 16.3703, 15.23238, 23.15464, 67.61226, 127.35813999999999, 8.884920000000001, 38.41496, 33.99028, 65.76822, 108.88726, 9.05002, 46.149260000000005, 48.07458, 56.91378, 23.599140000000002, 8.56488, 52.85994, 52.35194, 51.0413, 7.8740000000000006, 9.33958, 57.69356, 59.895739999999996, 42.33164, 9.37768, 8.371839999999999, 68.86956, 71.33590000000001, 36.12642, 8.67664, 8.93064, 79.375, 77.89672, 24.940260000000002, 8.387080000000001, 8.9535, 87.36584, 87.60968, 21.912580000000002, 8.4836, 9.02208, 90.84818000000001, 90.23604, 12.682220000000001, 9.3472, 8.82142, 99.95154, 99.5426, 7.19836, 8.74522, 8.945879999999999, 96.24822, 100.28936, 7.7343, 9.232899999999999, 9.51738, 98.75774, 102.26801999999999, 7.45744, 8.66394, 8.79348, 98.53675999999999, 109.74578, 7.442200000000001, 8.26008, 8.95096, 96.82988, 103.13924, 7.6936599999999995, 9.24814, 9.07034, 102.49662000000001, 95.3643, 6.94436, 9.21512, 9.19988, 101.96830000000001, 104.21366, 7.37616, 9.09066, 8.79348, 102.27564, 100.27919999999999]
m_m = ['R', 'R', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F']

# Initialization of the world
dim1, dim2 = 32, 16   # world dimensions (must match ultra)
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size
zones_x, zones_y = dim1 // 4, dim2 // 4  # number of 4×4 blocks

# Define what each zone actually is (the “ground truth”)
zone_layout = np.array([
    [2,1,5,2,4,3,4,3],
    [1,2,4,1,2,0,5,1],
    [5,4,3,4,4,5,4,5],
    [2,5,1,5,5,2,4,3]
])

# Define patterns for each zone type
zone_patterns = {
    0: np.full((4, 4), 0),  # free
    1: np.full((4, 4), 1),  # partial blocking
    2: np.full((4, 4), 2),  # adjacent blocking
    3: np.full((4, 4), 3),  # single side blocking
    4: np.full((4, 4), 4),  # total blocking (obstacle)
    5: np.full((4, 4), 5)   # opposite blocking
}

# Build the full-size map
world = np.zeros((dim2, dim1), dtype=int)
for r in range(zones_y):
    for c in range(zones_x):
        ztype = zone_layout[r, c]
        block = zone_patterns[ztype]
        world[r*4:(r+1)*4, c*4:(c+1)*4] = block

# Add a wall border (4s)
#world = np.pad(world, pad_width=1, mode='constant', constant_values=4)


# --- VISUALIZE MAP ---
plt.figure(figsize=(10, 5))
#cmap = plt.cm.gray.copy()
#cmap.set_bad(color='black')

Mask_array = np.ma.masked_where(world == 4, world)
#MASK
M = np.ones_like(world, dtype=int)  # 1 = free
M[world == 4] = 4                   # 4 = obstacle


plt.imshow(world, cmap= 'gray', origin='upper', vmin=0, vmax=5)
plt.title("Ground Truth World Map")
plt.axis('off')

for i in range(world.shape[0]):
    for j in range(world.shape[1]):
        if world[i, j] != 4:
            plt.text(j, i, str(world[i, j]), ha='center', va='center', fontsize=6, color='black')

plt.show()


#plt.title("Ultrasonic World Map (0–5 zones, 4 = Obstacle)")
#plt.axis('off')
#plt.show()

# --- GENERATE ULTRASONIC WORLD ---
m_u_arr = np.array(m_u)
usable_length = (m_u_arr.size // 5) * 5
m_u_arr = m_u_arr[:usable_length]

# number of 4x4 zones
zones_x = dim1 // 4  # 8
zones_y = dim2 // 4  # 4
num_zones = zones_x * zones_y

# Sensor data for each block
sensor_data = m_u_arr.reshape(-1, 5)

# Threshold for classifying ultrasonic readings (adjust as needed)
threshold = 15.54  # in cm (6 inches)

# Correct ultra array shape (rows=dim2, cols=dim1)
ultra = np.zeros((dim2, dim1), dtype=int)

print(f"Total zones: {num_zones}, Sensor data sets: {sensor_data.shape[0]}")

for i in range(min(sensor_data.shape[0], num_zones)):
    u2, u3, u0, u1, u4 = sensor_data[i]
    close = [u < threshold for u in [u0, u1, u2, u3]]
    close_count = sum(close)

    # --- PRIORITY ORDER ---
    if close_count == 0:
        val = 0  # free
    elif (close[0] and close[2]) or (close[1] and close[3]):
        val = 5  # opposite blocking
    elif ((close[0] and close[1]) or (close[1] and close[2]) or
          (close[2] and close[3]) or (close[3] and close[0])):
        val = 2  # adjacent blocking
    elif close_count == 1:
        val = 3  # 3 blocking
    else:
        val = 1  # one side blocked

    # --- DEBUG PRINT ---
    if i < 8:  # print first few classifications
        print(f"Zone {i}: close={close}, close_count={close_count}, val={val}")

    # --- Fill 4×4 region ---
    zone_row = (i // zones_x) * 4
    zone_col = (i % zones_x) * 4
    ultra[zone_row:zone_row+4, zone_col:zone_col+4] = val

print("Unique values BEFORE padding:", np.unique(ultra))

# --- PAD WALLS (add 4-border) ---
# We only want to pad around the outside — not overwrite existing data.
# Your map currently has values in [2,3,5]; we’ll keep them and just add walls.

#ultra_padded = np.pad(ultra, pad_width=1, mode='constant', constant_values=4)

#print("Unique values AFTER padding:", np.unique(ultra_padded))

# --- MASK WALLS FOR VISUALIZATION ---
# Mask all cells equal to 4 (walls)
#masked_ultra = np.ma.masked_where(ultra_padded == 4, ultra_padded)

# --- VISUALIZATION ---
#plt.figure(figsize=(10, 5))

# Use a copy-safe colormap
# cmap = mpl.cm.get_cmap('gray').copy()
# cmap.set_bad(color='black')  # walls = black

# plt.imshow(masked_ultra, cmap=cmap, origin='upper', vmin=0, vmax=5)
# plt.title("Sensor Generated Ultrasonic Map")
# plt.axis('off')

# mask_bool = np.ma.getmaskarray(masked_ultra)
# for r in range(masked_ultra.shape[0]):
#     for c in range(masked_ultra.shape[1]):
#         if not mask_bool[r, c]:
#             v = int(masked_ultra[r, c])
#             txt_color = 'white' if v >= 3 else 'black'
#             plt.text(c, r, f"{v}", ha='center', va='center', fontsize=6, color=txt_color)

#plt.show()

# Replace ultra with the padded version for later use
# ultra = ultra_padded.copy()

# --- INITIALIZE PROBABILITY ---
p = np.ones((dim2, dim1)) * (1 / n)

step_size_in_inches = 5
zone_size_in_inches = 12.5  # each 4×4 region is 12.5" square
steps_per_zone = int(zone_size_in_inches / step_size_in_inches)  # ~2-3 readings per zone

# --- LOCALIZATION LOOP ---
for k in range(len(m_m)):
    if k*5 + 5 <= len(m_u):
        current_readings = m_u[k*5 : k*5 + 5]
    else:
        break  # stop if we run out of data p = sense_u(world, M, p, current_readings)
    # Sensor update
    p = sense_u(world, M, p, current_readings, heading)  

    # HeatMap Visualization
    plt.clf()
    plt.imshow(p, cmap='hot', interpolation='nearest')
    plt.title(f"Step: {k+1}")
    plt.pause(0.5)

    # Movement update
    p, heading = move(p, M, heading, m_m[k])  

plt.show()

