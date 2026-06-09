import numpy as np
import pandas as pd
from scipy.optimize import least_squares
from scipy.linalg import sqrtm


# ----------------------------------------------------------
# Load Data
# ----------------------------------------------------------

df = pd.read_csv("Samples 1780955378.2357106.csv")

accel = df[["ax", "ay", "az"]].to_numpy()
gyro = df[["gx", "gy", "gz"]].to_numpy()


# ----------------------------------------------------------
# Gyroscope Bias
# ----------------------------------------------------------

gyro_bias = np.mean(gyro, axis=0)


# ----------------------------------------------------------
# Accelerometer Ellipsoid Fit
# ----------------------------------------------------------

def residuals(params, data):
    """
    params:
        bx by bz
        m11 m22 m33
        m12 m13 m23
    """

    b = params[0:3]

    M = np.array([
        [params[3], params[6], params[7]],
        [params[6], params[4], params[8]],
        [params[7], params[8], params[5]]
    ])

    d = data - b

    return np.sum((d @ M) * d, axis=1) - 1.0


# Initial Guess
p0 = np.array([
    0, 0, 0,
    1, 1, 1,
    0, 0, 0
])

result = least_squares(
    residuals,
    p0,
    args=(accel,)
)

p = result.x

bias = p[0:3]

M = np.array([
    [p[3], p[6], p[7]],
    [p[6], p[4], p[8]],
    [p[7], p[8], p[5]]
])

# Force symmetry
M = 0.5 * (M + M.T)

# Calibration matrix
A = np.real(sqrtm(M))


# ----------------------------------------------------------
# Verify Calibration
# ----------------------------------------------------------

accel_corrected = (A @ (accel - bias).T).T

norms = np.linalg.norm(accel_corrected, axis=1)

print("Mean calibrated magnitude:", np.mean(norms))
print("Std calibrated magnitude :", np.std(norms))


# ----------------------------------------------------------
# Generate Firmware Header
# ----------------------------------------------------------

print("\n")
print("// ------------------------------------------------")
print("// Accelerometer")
print("// ------------------------------------------------\n")

print("constexpr float ACCEL_BIAS[3] = {")
for x in bias:
    print(f"    {x:.8f},")
print("};\n")

print("constexpr float ACCEL_MATRIX[3][3] = {")
for row in A:
    print("    { " + ", ".join(f"{v:.8f}f" for v in row) + " },")
print("};\n")

print("// ------------------------------------------------")
print("// Gyroscope")
print("// ------------------------------------------------\n")

print("constexpr float GYRO_BIAS[3] = {")
for x in gyro_bias:
    print(f"    {x:.8f}f,")
print("};")