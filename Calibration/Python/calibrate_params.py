import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import inv, eigh

def calibrate_accelerometer_ms2(raw_data_ms2):
    N = raw_data_ms2.shape[0]
    x = raw_data_ms2[:, 0]
    y = raw_data_ms2[:, 1]
    z = raw_data_ms2[:, 2]
    
    g = 9.80665
    
    # 1. Design Matrix (X)
    X = np.column_stack([
        x**2, y**2, z**2, 
        2*x*y, 2*y*z, 2*x*z, 
        2*x, 2*y, 2*z
    ])
    Y = np.ones((N, 1)) * (g**2)
    
    # 2. Linear Least Squares
    beta, _, _, _ = np.linalg.lstsq(X, Y, rcond=None)
    beta = beta.flatten()
    
    # 3. Reconstruct algebraic matrices
    A_mat = np.array([
        [beta[0], beta[3], beta[5]],
        [beta[3], beta[1], beta[4]],
        [beta[5], beta[4], beta[2]]
    ])
    V = np.array([beta[6], beta[7], beta[8]])
    
    # 4. Extract Bias Vector
    A_mat_inv = inv(A_mat)
    B = -A_mat_inv.dot(V)
    
    # 5. Extract Correction Matrix via Eigenvalue Decomposition
    k = (g**2) + V.T.dot(A_mat_inv).dot(V)
    A_norm = (A_mat / k) * (g**2)
    
    w, v = eigh(A_norm)
    w = np.abs(w) 
    M_inv = v.dot(np.diag(np.sqrt(w))).dot(v.T)
    
    return M_inv, B.reshape(3, 1)

# ==========================================================
# CSV EXECUTION & VISUALIZATION PIPELINE (NO PANDAS)
# ==========================================================
if __name__ == "__main__":
    g = 9.80665
    csv_filename = 'Samples.csv' 
    
    # --------------------------------------------------------
    # STEP 1: Load CSV using standard Python file operations
    # --------------------------------------------------------
    try:
        data_list = []
        with open(csv_filename, 'r') as f:
            lines = f.readlines()
            
            # Check if the first line is a header (contains non-numeric characters)
            start_row = 0
            try:
                float(lines[0].split(',')[0])
            except ValueError:
                start_row = 1 # Skip header line
                
            for line in lines[start_row:]:
                if line.strip(): # Skip empty lines
                    row = [float(val) for val in line.split(',')]
                    data_list.append(row[:3]) # Grab first 3 columns (X, Y, Z)
                    
        raw_data = np.array(data_list)
        print(f"Successfully loaded {raw_data.shape[0]} data points from {csv_filename}.")
        
    except Exception as e:
        print(f"Error loading CSV file: {e}")
        print("Falling back to generating synthetic full-coverage data for demonstration...")
        # Fallback dummy data generation if file isn't found
        num_points = 500
        phi = np.random.uniform(0, 2 * np.pi, num_points)
        theta = np.arccos(np.random.uniform(-1, 1, num_points))
        x_ideal = g * np.sin(theta) * np.cos(phi)
        y_ideal = g * np.sin(theta) * np.sin(phi)
        z_ideal = g * np.cos(theta)
        raw_data = np.column_stack([x_ideal, y_ideal, z_ideal]) + np.array([0.5, -0.3, 0.2])

    # --------------------------------------------------------
    # STEP 2: Run Calibration
    # --------------------------------------------------------
    M_inv, B = calibrate_accelerometer_ms2(raw_data)
    calibrated_data = (M_inv.dot((raw_data - B.T).T)).T
    
    print("\n--- Extracted Parameters for your STM32 EKF ---")
    print(f"Bias Vector B (m/s²):\n{B.flatten()}")
    print(f"Correction Matrix M_inv:\n{M_inv}\n")
    norm_before = np.mean(np.linalg.norm(raw_data, axis=1))
    norm_after = np.mean(np.linalg.norm(calibrated_data, axis=1))
    print(f"Before Calibration: |g| = {norm_before}")
    print(f"After Calibration: |g| = {norm_after}\n")
    
    
    # --------------------------------------------------------
    # STEP 3: Generate Matplotlib Plots
    # --------------------------------------------------------
    u = np.linspace(0, 2 * np.pi, 30)
    v = np.linspace(0, np.pi, 30)
    xs = g * np.outer(np.cos(u), np.sin(v))
    ys = g * np.outer(np.sin(u), np.sin(v))
    zs = g * np.outer(np.ones(np.size(u)), np.cos(v))
    
    sphere_points = np.vstack([xs.flatten(), ys.flatten(), zs.flatten()])
    M = inv(M_inv)
    ellipsoid_points = (M.dot(sphere_points)).T + B.T
    
    xe = ellipsoid_points[:, 0].reshape(xs.shape)
    ye = ellipsoid_points[:, 1].reshape(ys.shape)
    ze = ellipsoid_points[:, 2].reshape(zs.shape)

    fig = plt.figure(figsize=(14, 6))
    
    # Left Plot: Raw CSV data vs the Calculated Ellipsoid Match
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.scatter(raw_data[:, 0], raw_data[:, 1], raw_data[:, 2], color='red', alpha=0.4, s=10, label='Raw CSV Data')
    ax1.plot_wireframe(xe, ye, ze, color='purple', alpha=0.15, linewidth=0.8, label='Fitted Ellipsoid')
    ax1.scatter(B[0], B[1], B[2], color='black', s=100, marker='X', label='Fitted Center (Bias)')
    ax1.set_title("Before Calibration (Your Logged Data Profile)")
    ax1.set_xlabel("X (m/s²)")
    ax1.set_ylabel("Y (m/s²)")
    ax1.set_zlabel("Z (m/s²)")
    ax1.legend()
    
    # Right Plot: How the data looks once calibrated
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.scatter(calibrated_data[:, 0], calibrated_data[:, 1], calibrated_data[:, 2], color='green', alpha=0.4, s=10, label='Calibrated Data')
    ax2.plot_wireframe(xs, ys, zs, color='blue', alpha=0.1, linewidth=0.8, label='Target Sphere (9.81 m/s²)')
    ax2.scatter(0, 0, 0, color='blue', s=100, marker='o', label='Ideal Center (0,0,0)')
    ax2.set_title("After Calibration (Corrected Output)")
    ax2.set_xlabel("X (m/s²)")
    ax2.set_ylabel("Y (m/s²)")
    ax2.set_zlabel("Z (m/s²)")
    ax2.legend()
    
    # Dynamic axis limit fitting based on data boundaries
    all_data = np.vstack([raw_data, calibrated_data])
    max_val = np.max(np.abs(all_data)) * 1.2
    for ax in [ax1, ax2]:
        ax.set_xlim(-max_val, max_val)
        ax.set_ylim(-max_val, max_val)
        ax.set_zlim(-max_val, max_val)
        
    plt.tight_layout()
    plt.show()