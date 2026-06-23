import pandas as pd
import matplotlib.pyplot as plt
import os

# -------------------------------
# Load CSV data
# -------------------------------
csv_path = os.path.expanduser('~/ros2_ws/ball_beam_log.csv')
df = pd.read_csv(csv_path)

# Remove any bad rows
df = df.dropna()

# Reference position
x_ref = 0.0

# Tolerance band for settling time
tol = 0.02  # meters

# Output folder
out_dir = os.path.expanduser('~/ros2_ws/ball_beam_plots')
os.makedirs(out_dir, exist_ok=True)

# -------------------------------
# Plot 1: Ball position x(t)
# -------------------------------
plt.figure()
plt.plot(df['time'], df['ball_pos'], label='Ball position x(t)')
plt.axhline(x_ref, linestyle='--', label='Reference x_ref = 0')
plt.axhline(tol, linestyle=':', label='+0.02 m tolerance')
plt.axhline(-tol, linestyle=':', label='-0.02 m tolerance')
plt.xlabel('Time [s]')
plt.ylabel('Ball Position x [m]')
plt.title('Ball Position vs Time')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(out_dir, 'ball_position.png'), dpi=300, bbox_inches='tight')

# -------------------------------
# Plot 2: Ball velocity x_dot(t)
# -------------------------------
plt.figure()
plt.plot(df['time'], df['ball_vel'], label='Ball velocity x_dot(t)')
plt.axhline(0, linestyle='--', label='Zero velocity')
plt.xlabel('Time [s]')
plt.ylabel('Ball Velocity x_dot [m/s]')
plt.title('Ball Velocity vs Time')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(out_dir, 'ball_velocity.png'), dpi=300, bbox_inches='tight')

# -------------------------------
# Plot 3: Beam command theta_cmd(t)
# -------------------------------
plt.figure()
plt.plot(df['time'], df['theta_cmd'], label='Beam command theta_cmd(t)')
plt.axhline(0.38, linestyle=':', label='+ joint limit')
plt.axhline(-0.38, linestyle=':', label='- joint limit')
plt.xlabel('Time [s]')
plt.ylabel('Beam Command theta_cmd [rad]')
plt.title('Beam Command vs Time')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(out_dir, 'beam_command.png'), dpi=300, bbox_inches='tight')

# -------------------------------
# Plot 4: Tracking error e(t)
# -------------------------------
plt.figure()
plt.plot(df['time'], df['tracking_error'], label='Tracking error e(t)')
plt.axhline(0, linestyle='--', label='Zero error')
plt.axhline(tol, linestyle=':', label='+0.02 m tolerance')
plt.axhline(-tol, linestyle=':', label='-0.02 m tolerance')
plt.xlabel('Time [s]')
plt.ylabel('Tracking Error e(t) [m]')
plt.title('Tracking Error vs Time')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(out_dir, 'tracking_error.png'), dpi=300, bbox_inches='tight')

# -------------------------------
# Plot 5: Phase portrait
# -------------------------------
plt.figure()
plt.plot(df['ball_pos'], df['ball_vel'])
plt.scatter(df['ball_pos'].iloc[0], df['ball_vel'].iloc[0], label='Start')
plt.scatter(df['ball_pos'].iloc[-1], df['ball_vel'].iloc[-1], label='End')
plt.axhline(0, linestyle='--')
plt.axvline(0, linestyle='--')
plt.xlabel('Ball Position x [m]')
plt.ylabel('Ball Velocity x_dot [m/s]')
plt.title('Phase Portrait: x_dot vs x')
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(out_dir, 'phase_portrait.png'), dpi=300, bbox_inches='tight')

# -------------------------------
# Performance metrics
# -------------------------------

error = df['tracking_error'].abs()
time = df['time']

# Steady-state error
steady_state_error = error.iloc[-1]

# Maximum beam command
max_beam_command = df['theta_cmd'].abs().max()

# Settling time:
# first time after which error remains inside tolerance forever
settling_time = None
for i in range(len(df)):
    if (error.iloc[i:] <= tol).all():
        settling_time = time.iloc[i]
        break

# Maximum overshoot:
# distance past center after first crossing
initial_error = df['tracking_error'].iloc[0]
tracking_error = df['tracking_error']

overshoot = 0.0

if initial_error > 0:
    crossed = tracking_error[tracking_error <= 0]
    if len(crossed) > 0:
        first_cross_index = crossed.index[0]
        overshoot = abs(tracking_error.loc[first_cross_index:].min())

elif initial_error < 0:
    crossed = tracking_error[tracking_error >= 0]
    if len(crossed) > 0:
        first_cross_index = crossed.index[0]
        overshoot = abs(tracking_error.loc[first_cross_index:].max())

# Qualitative stability:
# Check whether the final 20% of the simulation stays mostly inside tolerance.
last_part = error.iloc[int(0.8 * len(error)):]

percent_inside_tol = (last_part <= tol).mean() * 100

if percent_inside_tol >= 90:
    qualitative_stability = 'Stable within tolerance'
elif percent_inside_tol >= 70:
    qualitative_stability = 'Marginally stable / small residual oscillation'
else:
    qualitative_stability = 'Unstable'

# Save metrics
metrics_path = os.path.join(out_dir, 'performance_metrics.txt')

with open(metrics_path, 'w') as f:
    f.write('Ball-Beam LQR Performance Metrics\n')
    f.write('=================================\n\n')
    f.write(f'Settling time: {settling_time} seconds\n')
    f.write(f'Steady-state error: {steady_state_error:.5f} m\n')
    f.write(f'Maximum overshoot: {overshoot:.5f} m\n')
    f.write(f'Maximum beam command: {max_beam_command:.5f} rad\n')
    f.write(f'Qualitative stability: {qualitative_stability}\n')

print('Plots saved to:', out_dir)
print()
print('Performance Metrics')
print('-------------------')
print(f'Settling time: {settling_time} seconds')
print(f'Steady-state error: {steady_state_error:.5f} m')
print(f'Maximum overshoot: {overshoot:.5f} m')
print(f'Maximum beam command: {max_beam_command:.5f} rad')
print(f'Qualitative stability: {qualitative_stability}')
