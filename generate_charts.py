import matplotlib.pyplot as plt
import numpy as np

# Generate mock data: Time (0 to 60 minutes)
time = np.linspace(0, 60, 300)

# 1. Heart Rate (BPM): Starts normal (~75), gradually drops or becomes irregular during fatigue (~65)
bpm_normal = 75 + np.random.normal(0, 2, 150)
bpm_fatigue = 65 + np.random.normal(0, 3, 150)
bpm = np.concatenate([bpm_normal, bpm_fatigue])
# smooth it a bit
bpm = np.convolve(bpm, np.ones(5)/5, mode='same')

# 2. Eye Aspect Ratio (EAR): Normally around 0.3, blinks drop it to 0.1.
# In fatigue, blinks are longer and EAR baseline might drop slightly.
ear = np.random.normal(0.32, 0.01, 300)
# Add normal blinks
for i in np.random.choice(150, 15, replace=False):
    ear[i] = 0.1
# Add fatigue blinks (more frequent and longer)
for i in np.random.choice(range(150, 300), 25, replace=False):
    ear[i] = 0.1
    if i+1 < 300: ear[i+1] = 0.12 # Microsleep/long blink

# 3. Head Movement (Pitch/Roll): Stable normally, nodding (pitch drops) when fatigued
pitch = np.random.normal(0, 1, 300)
# add nodding events in the second half
for i in np.random.choice(range(150, 290), 5, replace=False):
    pitch[i:i+5] -= np.linspace(0, 15, 5) # Head drops
    pitch[i+5:i+10] += np.linspace(0, 15, 5) # Jerks back up

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

# Plot Heart Rate
ax1.plot(time, bpm, color='red', label='Heart Rate (BPM)')
ax1.axvline(x=30, color='gray', linestyle='--', label='Fatigue Onset')
ax1.set_ylabel('BPM')
ax1.set_title('Sensor 1: Heart Rate (Pulse Sensor)')
ax1.legend()
ax1.grid(True)

# Plot EAR
ax2.plot(time, ear, color='blue', label='Eye Aspect Ratio (EAR)')
ax2.axvline(x=30, color='gray', linestyle='--')
ax2.axhline(y=0.2, color='orange', linestyle=':', label='Blink Threshold')
ax2.set_ylabel('EAR Value')
ax2.set_title('Sensor 2: Eye Blink (Camera)')
ax2.legend()
ax2.grid(True)

# Plot Head Pitch
ax3.plot(time, pitch, color='green', label='Head Pitch (Degrees)')
ax3.axvline(x=30, color='gray', linestyle='--')
ax3.set_ylabel('Angle (deg)')
ax3.set_xlabel('Time (Minutes)')
ax3.set_title('Sensor 3: Head Movement (IMU MPU6050)')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.savefig(r'd:\proj\IoT-Fatigue\sensor_graphs.png', dpi=300)
print("Charts generated at d:\\proj\\IoT-Fatigue\\sensor_graphs.png")
