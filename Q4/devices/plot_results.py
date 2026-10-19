import pandas as pd
import matplotlib.pyplot as plt

data_baseline = pd.read_csv('controllers/controller_movement/data_friction1.csv')
data_low_slip = pd.read_csv('controllers/controller_movement/data_friction5.csv')

plt.figure(figsize=(10, 5))


plt.plot(data_baseline['Time'], data_baseline['s_long'], label='High fricción', color='blue', alpha=0.8)
plt.plot(data_low_slip['Time'], data_low_slip['s_long'], label='Low Friction', color='red', linestyle='--', alpha=0.8)

plt.title('Longitudinal Slip Comparison ($s_{long}$) along the trajectory')
plt.xlabel('Time (s)')
plt.ylabel('Glide ($s_{long}$)')
plt.axhline(0, color='gray', linestyle=':', linewidth=1)
plt.legend()
plt.grid(True)
plt.tight_layout()

plt.savefig('Slip_Comparison.png', dpi=300)
plt.show()