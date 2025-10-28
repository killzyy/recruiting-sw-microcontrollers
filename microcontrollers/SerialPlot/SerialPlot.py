import numpy as np
import sounddevice as sd
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.ticker import MaxNLocator
from collections import deque

# SERIAL SETTINGS
SERIAL_PORT = '/dev/tty.usbmodem1403'
BAUD_RATE = 9600
PLOT_LEN = 100

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
ser.reset_input_buffer()

# AUDIO SETTINGS
SAMPLE_RATE = 44100
BUFFER_SIZE = 1024

# DATA VARIABLES
analog_data = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
analog_moving_average = deque([0.0]*PLOT_LEN, maxlen=PLOT_LEN)
digital_data = deque([0]*PLOT_LEN, maxlen=PLOT_LEN)

# PLOT STYLE
plt.style.use('dark_background')

# PLOT CONFIG
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

def map_voltage_to_frequency(volts, min_freq=220, max_freq=880):
    return min_freq + (volts / 5.0) * (max_freq - min_freq)

def map_voltage_to_amplitude(volts):
    return min(max(volts / 5.0, 0.0), 1.0)

def audio_callback(outdata, frames, time, status):
    global analog_data
    if status:
        print(status)

    volts = analog_data[-1]
    freq = map_voltage_to_frequency(volts)
    amp = map_voltage_to_amplitude(volts)

    t = (np.arange(frames) + audio_callback.phase) / SAMPLE_RATE
    outdata[:, 0] = amp * np.sin(2 * np.pi * freq * t)
    audio_callback.phase += frames


audio_callback.phase = 0

# Start audio stream
stream = sd.OutputStream(channels=1, callback=audio_callback, samplerate=SAMPLE_RATE, blocksize=BUFFER_SIZE)
stream.start()

# UPDATE PLOT
def update_plot(frame):
    try:
        while ser.in_waiting:
            line = ser.readline().decode(errors='ignore').strip()
            print(line)
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