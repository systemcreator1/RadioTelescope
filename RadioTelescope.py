import tkinter as tk
import numpy as np
import matplotlib.pyplot as plt
from rtlsdr import RtlSdr
from scipy.signal import butter, lfilter
import RPi.GPIO as GPIO
import time
import csv

# GPIO setup for motors
AZIMUTH_PIN = 18  # Horizontal (azimuth)
ELEVATION_PIN = 23  # Vertical (elevation)
GPIO.setmode(GPIO.BCM)
GPIO.setup(AZIMUTH_PIN, GPIO.OUT)
GPIO.setup(ELEVATION_PIN, GPIO.OUT)
azimuth_pwm = GPIO.PWM(AZIMUTH_PIN, 50)
elevation_pwm = GPIO.PWM(ELEVATION_PIN, 50)
azimuth_pwm.start(0)
elevation_pwm.start(0)

# SDR Setup
sdr = RtlSdr()
sdr.sample_rate = 2.048e6
sdr.center_freq = 1420e6  # Example frequency
sdr.gain = 49.6

# Amplify signal function
def amplify_signal(signal, factor=10):
    return signal * factor

# Lowpass filter
def butter_lowpass_filter(data, cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return lfilter(b, a, data)

# Servo control functions
def set_servo_angle(pwm, angle):
    duty_cycle = 2 + (angle / 18)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

# Function to start scanning
def start_scanning():
    azimuth = azimuth_slider.get()
    elevation = elevation_slider.get()
    set_servo_angle(azimuth_pwm, azimuth)
    set_servo_angle(elevation_pwm, elevation)
    
    # Record position and frequency data
    with open("scan_data.csv", "a") as file:
        writer = csv.writer(file)
        
        # Frequency sweep range
        frequency_range = np.linspace(1400e6, 1430e6, 100)
        
        for freq in frequency_range:
            sdr.center_freq = freq
            samples = sdr.read_samples(256*1024)
            
            # Amplify and filter signal
            amplified_samples = amplify_signal(samples, factor=15)
            filtered_samples = butter_lowpass_filter(amplified_samples, cutoff=0.1 * sdr.sample_rate, fs=sdr.sample_rate)
            
            # Calculate FFT and power
            fft_result = np.fft.fftshift(np.fft.fft(filtered_samples))
            power = np.abs(fft_result)**2
            
            # Save azimuth, elevation, frequency, and power
            writer.writerow([azimuth, elevation, freq, np.max(power)])
            
            # Live plotting
            plt.clf()
            plt.plot(np.fft.fftshift(np.fft.fftfreq(len(filtered_samples), 1/sdr.sample_rate)), power)
            plt.xlabel('Frequency (Hz)')
            plt.ylabel('Power')
            plt.title(f'Freq Spectrum at {freq/1e6} MHz, Azimuth: {azimuth}, Elevation: {elevation}')
            plt.grid()
            plt.pause(0.1)
    plt.show()

# UI setup
root = tk.Tk()
root.title("Enhanced Radio Telescope Control")

# Azimuth Control
tk.Label(root, text="Azimuth Control (0-180)").pack()
azimuth_slider = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL)
azimuth_slider.pack()

# Elevation Control
tk.Label(root, text="Elevation Control (0-180)").pack()
elevation_slider = tk.Scale(root, from_=0, to=180, orient=tk.HORIZONTAL)
elevation_slider.pack()

# Scan Button
scan_button = tk.Button(root, text="Start Scanning", command=start_scanning)
scan_button.pack()

# Cleanup and close UI
def on_closing():
    azimuth_pwm.stop()
    elevation_pwm.stop()
    GPIO.cleanup()
    sdr.close()
    root.destroy()

root.protocol("WM_DELETE_WINDOW", on_closing)
root.mainloop()
