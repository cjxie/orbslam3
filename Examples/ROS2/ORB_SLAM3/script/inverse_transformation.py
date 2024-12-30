import numpy as np

# Create the transformation matrix
transform = np.array([
    [-3.72920861e-03, 9.99988973e-01, -2.85669905e-03, 1.37025189e-04],
    [9.99983966e-01, 3.71699338e-03, -4.26937127e-03, -2.30529237e-02],
    [-4.25870577e-03, -2.87257484e-03, -9.99986827e-01, -2.09780982e-03],
    [0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

# Calculate inverse
inverse = np.linalg.inv(transform)

# Print with high precision
np.set_printoptions(precision=10, suppress=True)
print("Inverse transformation matrix:")
print(inverse)

# Verify the inversion by multiplying original with inverse
verification = np.dot(transform, inverse)
print("\nVerification (should be identity matrix):")
print(verification)