import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator
from collections import deque

# SERIAL SETTINGS
SERIAL_PORT = '/dev/tty.usbmodem1403'
BAUD_RATE = 115200
PLOT_LEN = 100

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05)
ser.reset_input_buffer()

# DATA VARIABLES
analog_data = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
analog_moving_average = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
digital_data = deque([0]*PLOT_LEN, maxlen=PLOT_LEN)

# PLOT STYLE
plt.style.use('dark_background')

# PLOT CONFIGURATION
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 7))
fig.canvas.manager.set_window_title('Serial Plot')

line1, = ax1.plot([], [], color='yellow', label='Analog Data')
line1moving, = ax1.plot([], [], color='red', label='Analog Moving Average')
line2, = ax2.plot([], [], color='magenta')

ax1.set_ylim(-0.5, 5.5)
ax2.set_ylim(-0.5, 1.5)
ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
ax1.set_xlim(0, PLOT_LEN-1)
ax2.set_xlim(0, PLOT_LEN-1)

ax1.set_title("Analog Value")
ax2.set_title("Digital Value")
ax1.set_ylabel("Volts")
ax2.set_ylabel("State")
ax1.legend(loc='lower right')

plt.tight_layout()

# UPDATE PLOT
def update_plot(frame):
    try:
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            if not line:
                continue

            try:
                mv, moving_average_mv, digital_val = map(int, line.replace(' ', '').split(','))
            except ValueError:
                continue

            volts = mv / 1000.0
            moving_average_volts = moving_average_mv / 1000.0
            analog_data.append(volts)
            analog_moving_average.append(moving_average_volts)
            digital_data.append(digital_val)

            line1.set_data(range(len(analog_data)), analog_data)
            line1moving.set_data(range(len(analog_moving_average)), analog_moving_average)
            line2.set_data(range(len(digital_data)), digital_data)

    except serial.SerialException:
        pass

    return line1, line1moving, line2

ani = animation.FuncAnimation(
    fig,
    update_plot,
    interval=10,
    blit=False,
    cache_frame_data=False
)

# DISPLAY PLOT
plt.show()