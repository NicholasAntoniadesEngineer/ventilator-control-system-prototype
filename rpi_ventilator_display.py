"""
Ventilator Control System

Author: Nicholas Antoniades

Description:
This script displays an application window for the Ventilator Control System,
using Tkinter for the GUI and Matplotlib for real-time data plotting.

The application reads pressure and flow data from a serial port, processes it, and
updates the GUI with real-time plots of pressure (in cm H₂O) and volume (in L/min).
It also displays the maximum pressure and volume over specified intervals.

"""

import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import numpy as np
import time
import matplotlib.animation as animation
import serial
from numpy import trapz


class MainPage(tk.Tk):
    """
    Main application window class for the Ventilator Control System.
    Initializes the serial communication and sets up the GUI.
    """

    def __init__(self):
        super().__init__()

        # Serial communication settings
        self.ser = serial.Serial('/dev/ttyS0', 115200, timeout=1)
        settings_new = {
            'baudrate': 115200,
            'bytesize': serial.EIGHTBITS,
            'parity': serial.PARITY_EVEN,
            'stopbits': serial.STOPBITS_TWO,
            'xonxoff': False,
            'dsrdtr': False,
            'rtscts': False,
            'timeout': 1,
            'write_timeout': None,
            'inter_byte_timeout': None
        }

        # Apply settings to serial communication
        stop = False
        while not stop:
            try:
                self.ser.apply_settings(settings_new)
                stop = True
            except Exception as e:
                print(f"Error applying serial settings: {e}")
                time.sleep(1)

        self.ser.flushInput()

        # Configure main window
        self.title("Ventilator Control System")
        width = self.winfo_screenwidth()
        height = self.winfo_screenheight()
        self.geometry(f"{width}x{height}+0+0")

        # Container frame to hold other frames
        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        # Initialize frames and show the main plotting graph
        self.frames = {}
        frame = PlottingGraph(container, self)
        self.frames[PlottingGraph] = frame
        frame.grid(row=0, column=0, sticky="nsew")
        self.show_frame(PlottingGraph)

    def show_frame(self, cont):
        """
        Display the specified frame.

        Args:
            cont: The frame class to be displayed.
        """
        frame = self.frames[cont]
        frame.update()
        frame.tkraise()


class PlottingGraph(tk.Frame):
    """
    Frame containing the plotting graphs for pressure and volume.
    Reads data from the serial port and updates the plots in real-time.
    """

    def __init__(self, parent, controller):
        super().__init__(parent)

        self.controller = controller
        self.ser = controller.ser

        # Initialize counters (if needed for future extensions)
        self.update_counter1 = 0
        self.update_counter2 = 0

        # Initialize data arrays
        self.number_of_entries = 300  # Number of data points on the x-axis
        self.x_data = np.arange(self.number_of_entries)
        self.y1_data = np.zeros(self.number_of_entries, dtype=np.float64)  # Pressure data
        self.y2_data = np.zeros(self.number_of_entries, dtype=np.float64)  # Volume data

        # Initialize integral flow and time arrays for volume calculation
        self.integral_flow = []
        self.time_flow = []

        # Create labels for displaying current values and max values
        self.label1 = tk.Label(self, text='Pressure', background='lightskyblue',
                               font=('Helvetica', 20), width=10)
        self.label1.place(relx=0.045, rely=0.1, anchor='nw')

        self.label2 = tk.Label(self, text='Volume', background='lightskyblue',
                               font=('Helvetica', 20), width=10)
        self.label2.place(relx=0.045, rely=0.6, anchor='nw')

        self.label3 = tk.Label(self, text='Max\n0', background='violet',
                               font=('Helvetica', 20), width=11, height=2)
        self.label3.place(relx=0.176, rely=0.1, anchor='c')

        self.label4 = tk.Label(self, text='Max\n0', background='violet',
                               font=('Helvetica', 20), width=11, height=2)
        self.label4.place(relx=0.176, rely=0.6, anchor='c')

        # Create the matplotlib figure and axes
        self.figure = Figure(dpi=100)
        self.figure.set_facecolor('midnightblue')

        # Create subplots for pressure and volume
        self.ax1 = self.figure.add_subplot(211)
        self.ax2 = self.figure.add_subplot(212)

        # Configure axes appearance
        self.ax1.set_facecolor('midnightblue')
        self.ax2.set_facecolor('midnightblue')

        self.ax1.set_ylim(-10, 100)  # Pressure y-axis limits
        self.ax2.set_ylim(-10, 1200)  # Volume y-axis limits

        self.ax1.set_ylabel('cm H₂O', rotation='horizontal', labelpad=20)
        self.ax2.set_ylabel('L/min', rotation='horizontal', labelpad=20)

        self.ax1.spines["top"].set_visible(False)
        self.ax1.spines["right"].set_visible(False)
        self.ax2.spines["top"].set_visible(False)
        self.ax2.spines["right"].set_visible(False)

        self.ax1.grid(which='both', linestyle='--', color='gray')
        self.ax2.grid(which='both', linestyle='--', color='gray')

        self.ax1.yaxis.set_label_coords(0, 1.02)
        self.ax2.yaxis.set_label_coords(0, 1.02)

        self.figure.tight_layout()

        # Plot initial data
        self.line1, = self.ax1.plot(self.x_data, self.y1_data, color='lightskyblue')
        self.line2, = self.ax2.plot(self.x_data, self.y2_data, color='lightskyblue')

        # Add vertical lines to indicate intervals
        self.ax1.axvline(50, color='white')
        self.ax1.axvline(150, color='white')
        self.ax2.axvline(50, color='white')
        self.ax2.axvline(150, color='white')

        # Create the Tkinter canvas to display the matplotlib figure
        self.canvas = FigureCanvasTkAgg(self.figure, self)
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.canvas._tkcanvas.pack(side=tk.RIGHT, expand=True)

        # Bring labels to the front
        self.label1.lift()
        self.label2.lift()
        self.label3.lift()
        self.label4.lift()

        # Start the animation functions
        self.ani1 = animation.FuncAnimation(self.figure, self.animate1, interval=10, blit=True)
        self.ani2 = animation.FuncAnimation(self.figure, self.animate2, interval=10, blit=True)

    def update_data(self):
        """
        Update the data arrays with new readings from the serial port.
        Reads pressure and flow data, computes volume, and updates the data arrays.
        """
        if self.ser.in_waiting > 0:
            # Read 12 bytes from the serial port
            line = self.ser.read(12)
            self.time_flow.append(time.time())
            y = np.frombuffer(line, dtype=np.uint8)

            try:
                # Shift data in arrays to make room for new data
                self.y1_data = np.roll(self.y1_data, 1)
                self.y2_data = np.roll(self.y2_data, 1)

                # Parse pressure data (combine two bytes into one value)
                pressure = (y[1] << 8) | y[0]

                # Parse flow data and calculate volume
                if y[3] == 0:
                    # Flow is positive
                    flow = y[2]
                    self.integral_flow.append(flow)
                    # Calculate volume using the trapezoidal rule
                    volume = trapz(np.array(self.integral_flow) * (1000 / 60), self.time_flow)
                else:
                    # Flow is negative
                    flow = -y[2]
                    # Reset integral flow and time if flow changes direction
                    self.integral_flow = []
                    self.time_flow = []
                    volume = 0

                # Check if the message is valid (based on a flag byte)
                if y[-1] == 255:
                    self.y1_data[0] = pressure
                    self.y2_data[0] = volume
                else:
                    # If invalid, repeat the last data point
                    self.y1_data[0] = self.y1_data[1]
                    self.y2_data[0] = self.y2_data[1]

                # Clear the serial input buffer
                self.ser.flushInput()
            except Exception as e:
                print(f"Error updating data: {e}")
                # In case of error, repeat the last data point
                self.y1_data[0] = self.y1_data[1]
                self.y2_data[0] = self.y2_data[1]

            # Update labels with the latest data
            try:
                self.label1.config(text=f"{round(self.y1_data[0], 2)}")
                self.label2.config(text=f"{round(self.y2_data[0], 2)}")
            except Exception as e:
                print(f"Error updating labels: {e}")

    def animate1(self, _):
        """
        Animation function for the pressure plot.
        Updates the plot with new pressure data.

        Args:
            _: Unused argument required by FuncAnimation.

        Returns:
            A tuple containing the updated line plot.
        """
        self.update_data()
        self.line1.set_ydata(self.y1_data)
        # Calculate max pressure over the interval 50:150
        max_pressure = round(np.max(self.y1_data[50:150]), 2)
        self.label3.config(text=f"Max\n{max_pressure}")
        return self.line1,

    def animate2(self, _):
        """
        Animation function for the volume plot.
        Updates the plot with new volume data.

        Args:
            _: Unused argument required by FuncAnimation.

        Returns:
            A tuple containing the updated line plot.
        """
        self.line2.set_ydata(self.y2_data)
        # Calculate max volume over the interval 50:150
        max_volume = round(np.max(self.y2_data[50:150]), 2)
        self.label4.config(text=f"Max\n{max_volume}")
        return self.line2,


if __name__ == "__main__":
    
    # Use the 'dark_background' style for matplotlib
    plt.style.use("dark_background")

    # Create and run the application
    app = MainPage()
    app.mainloop()
