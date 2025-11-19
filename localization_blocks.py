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
    threshold = 7  # in cm ()
    
    #Claasify into zone type based on sensor value
    u0, u1, u2, u3, u4 = SenVal
    close = [u < threshold for u in [u0, u1, u2, u3]]
    close_count = sum(close)

    
    # if close_count == 0:
    #     sensor_zone = 0  # free
    # elif close_count == 1:
    #     sensor_zone = 1  #1 side blocked, 3 free
    # elif ((close[0] and close[1]) or (close[0] and close[2]) or
    #       (close[2] and close[3]) or (close[1] and close[3])):
    #     sensor_zone = 2  # adjacent blocking
    # elif (close[0] and close[3]) or (close[1] and close[2]):
    #     sensor_zone = 5  # opposite blocking
    # elif close_count == 3:
    #     sensor_zone = 3  #3 blocked, 1 free
    # else:
    #     sensor_zone = 4  # obstacle (all sides blocked)
    
    if close_count == 0:
        sensor_zone = 0  # free
    elif close_count == 1:
        sensor_zone = 1  #1 side blocked, 3 free
    elif close_count == 2:
        if ((close[0] and close[1]) or (close[0] and close[2]) or (close[2] and close[3]) or (close[1] and close[3])):
            sensor_zone = 2  # adjacent blocking
        elif (close[0] and close[3]) or (close[1] and close[2]):
            sensor_zone = 5  # opposite blocking
    elif close_count == 3:
        sensor_zone = 3  #3 blocked, 1 free
    else:
        sensor_zone = 4  # obstacle (all sides blocked)
    
    print(f"Sensor Readings: {SenVal}, Close Count: {close_count} Classified as: {sensor_zone}") #Debug Mapping Test

    # Define expected obstacle patterns per zone type 
    
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
    mult[world == sensor_zone] = pHit

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

    # # --- DIRECTION HANDLING ---
    # if Move == 'L':
    #     heading = (heading + 90) % 360
        
    # elif Move == 'R':
    #     heading = (heading - 90) % 360
        
    # elif Move == 'B':
    #     heading = (heading + 180) % 360
        
    # 'F' means forward → no change to heading
    
    #rad = np.deg2rad((heading + 90) % 360)
    rad = np.deg2rad(heading)
    col_step = int(np.round(np.cos(rad)))  # +1 -> right, 0, or -1 -> left
    row_step = int(np.round(np.sin(rad)))  # +1 -> down, 0, or -1 -> up

    step_fraction = 0.4  # 5" / 12.5"

    # Column shift (positive col_step = move right)
    if col_step == 1:
        # move right: shift contents to the right
        shifted = np.hstack([pnew[:, 1:], np.zeros((pnew.shape[0], 1))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif col_step == -1:
        # move left: shift contents to the left
        shifted = np.hstack([np.zeros((pnew.shape[0], 1)), pnew[:, :-1]])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted

    # Row shift (positive row_step = move down on screen)
    if row_step == 1:
        # move down: shift contents downward
        shifted = np.vstack([pnew[1:, :], np.zeros((1, pnew.shape[1]))])
        pnew = (1 - step_fraction) * pnew + step_fraction * shifted
    elif row_step == -1:
        # move up: shift contents upward
        shifted = np.vstack([np.zeros((1, pnew.shape[1])), pnew[:-1, :]])
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


# --- INITIAL SETUP ---
# Heading (degrees) //get from gyroscope
heading = 180 # 270° is down, 90 is up, 0 is left, 180 is right

# Provide measurements and movements 
m_u = [15.182, 3.62, 3.254, 28.461, 15.238, 15.339, 3.824, 3.274, 27.769, 29.258, 13.898, 3.423, 3.662, 31.035, 32.075, 11.293, 3.367, 3.37, 33.504, 32.181, 10.406, 3.528, 3.411, 35.925, 33.611, 6.666, 15.135, 3.689, 37.091, 35.894, 6.357, 15.531, 3.387, 38.433, 39.441, 3.149, 15.934, 3.349, 39.781, 41.304, 3.105, 9.254, 4.617, 4.357, 3.81, 15.88, 36.086, 3.565, 3.408, 3.675, 14.002, 37.951, 3.459, 6.048, 6.489, 11.16, 39.837, 3.522, 8.09, 7.282, 10.595, 6.988, 3.493, 10.144, 11.12, 7.057, 3.446, 27.279, 11.563, 11.563, 6.564, 3.629, 34.424, 14.443, 15.151, 2.923, 3.484, 49.757, 14.417, 15.956, 3.208, 3.999, 6.229, 5.101, 3.986, 50.672, 3.592, 15.386, 3.431, 3.394, 50.472, 3.654, 15.188, 6.112, 6.625, 46.477, 3.181, 15.367, 7.696, 7.337, 47.972, 3.608, 3.622, 9.856, 10.251, 42.6, 3.593, 3.437, 11.195, 11.786, 43.19, 3.357, 3.315, 13.843, 14.368, 40.831, 3.225, 3.573, 15.508, 15.46, 41.601, 3.533, 3.44, 17.556, 17.566, 33.54, 3.353, 3.338, 18.866, 19.682]
m_m = ['F', 'F', 'F', 'F', 'F', 'F', 'R', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'L', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F', 'F']

# Initialization of the world
dim1, dim2 = 32, 16   # world dimensions (must match ultra)
locationindex = np.arange(1, dim1 * dim2 + 1).reshape(dim2, dim1)
n = locationindex.size
zones_x, zones_y = dim1 // 4, dim2 // 4  # number of 4×4 blocks

# Define what each zone actually is (the “ground truth”)
zone_layout = np.array([
    [2,1,5,2,4,3,4,3],
    [1,2,4,2,5,0,5,1],
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

# --- INITIALIZE PROBABILITY ---
p = np.ones((dim2, dim1)) * (1 / n)

# --- LOCALIZATION LOOP ---
for k in range(len(m_m)):
    if k*5 + 5 <= len(m_u):
        current_readings = m_u[k*5 : k*5 + 5]
    else:
        break  # stop if we run out of data p = sense_u(world, M, p, current_readings)
    
    command = m_m[k]
    if command == 'L':
        heading = (heading + 90) % 360
        continue
    
    elif command == 'R':
        heading = (heading - 90) % 360
        continue
    
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

