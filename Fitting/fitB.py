import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, splrep, splev

# Generate
np.random.seed(0)
n = 300
ts = np.sort(np.random.uniform(0, 5, size=n))
ys = np.sin(ts) + 0.1*np.random.randn(n)

# Fit
tck = splrep(ts, ys, t=ts[2:-2], k=3)
# Alternative:
# tck = splrep(ts, ys, s=0, k=3)
ys_interp = splev(ts, tck)

# Display
plt.figure(figsize=(12, 6))
plt.plot(ts, ys, '.c')
plt.plot(ts, ys_interp, '-m')
# plt.show()

# Fit
n_interior_knots = 5
qs = np.linspace(0, 1, n_interior_knots+2)[1:-1]
knots = np.quantile(ts, qs)
tck = splrep(ts, ys, t=knots, k=3)
ys_smooth = splev(ts, tck)

# Alternative if one really wants to use BSpline:
# ys_smooth = BSpline(*tck)(ts)

# Display
plt.figure(figsize=(12, 6))
plt.plot(ts, ys, '.c')
plt.plot(ts, ys_smooth, '-m')
plt.show()