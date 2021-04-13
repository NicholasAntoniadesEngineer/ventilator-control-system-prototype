import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.ticker import AutoMinorLocator
from matplotlib.figure import Figure
import numpy as np
import sys
import time
import random
import matplotlib
import matplotlib.animation as animation
import serial
from numpy import trapz
#matplotlib.use('TkAgg')

#Does the new clock speed make a difference? Ask = when it changes.
class Main_page(tk.Tk):
    def __init__(self, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        global ser
        settings_new = {'baudrate': 115200,
         'bytesize': 8,
         'parity': 'E',
         'stopbits': 2,
         'xonxoff': False,
         'dsrdtr': False,
         'rtscts': False,
         'timeout': 1,
         'write_timeout': None,
         'inter_byte_timeout': None}
    
    
        ser = serial.Serial('/dev/ttyS0',115200,timeout=1)
        STOP = 0
        while STOP == 0:
            try:
                ser.apply_settings(settings_new)
                STOP = 1
            except:
                ''
        ser.flushInput()
#         print(dir(ser))
        
        #self.overrideredirect(True)
#        self.fullscreen_function()
        width = self.winfo_screenwidth()
        height = self.winfo_screenheight()
#        print(width, height)
        self.geometry("%dx%d+%d+%d" % (width, height, 0, 0))
        
        container = tk.Frame(self)
        container.pack(side = "top", fill = "both", expand = True)
        container.grid_rowconfigure(0, weight = 1)
        container.grid_columnconfigure(0, weight = 1)

#         self.frames = {}
#         global plot_frame,figure, plotter
#         plot_frame = Plotting_Graph(container, self)
#         self.frames[Plotting_Graph] = plot_frame
#         plot_frame.grid(row = 0, column = 0, sticky = "nsew")

        self.frames = {}
#         for F in (Plotting_Graph,Plotting_Graph2):
        for F in [Plotting_Graph]:
            frame = F(container, self)
            self.frames[F] = frame
            frame.grid(row = 0, column = 0, sticky = 'nsew')
            
        self.show_frame(Plotting_Graph)
#        self.fullscreen_function()
        
    def show_frame(self, cont):
        frame = self.frames[cont]
#        self.fullscreen_50function()
        frame.update()
        frame.tkraise()
        
#    def fullscreen_function(self):
#        self.attributes("-fullscreen", True)

class Plotting_Graph(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        global figure_canvas, figure, sub_axes_1, sub_axes_2, subplot_1, subplot_2, x,y1,y2, scatters, label1, label2,ser, update_counter1, update_counter2
        
        update_counter1 = 0
        update_counter2 = 0
        
        label1 = tk.Label(self, text = 'Teheightst', background = 'lightskyblue', font= ('Helvetica', 20), width = 5)
        label1.place(relx = 0.045, rely = 0.1, anchor = 'nw')
        
        label2 = tk.Label(self, text = 'Test', background = 'lightskyblue', font= ('Helvetica', 20), width = 5)
        label2.place(relx = 0.045, rely = 0.6, anchor = 'nw')
        
        global label3
        label3 = tk.Label(self, text = 'Max \n 10', background = 'violet', font= ('Helvetica', 20), width = 11, height = 2)
        label3.place(relx = 0.176, rely = 0.1, anchor = 'c')
        
        global label4
        label4 = tk.Label(self, text = 'Max \n 10', background = 'violet', font= ('Helvetica', 20), width = 11, height = 2)
        label4.place(relx = 0.176, rely = 0.6, anchor = 'c') 
        
        figure = Figure(dpi = 100)
        figure.set_facecolor('midnightblue')

        sub_axes_1 = figure.add_subplot(211)
        sub_axes_2 = figure.add_subplot(212)
        sub_axes_1.set_facecolor('midnightblue')
        sub_axes_2.set_facecolor('midnightblue')
        sub_axes_1.set_ylim(-10, 100)
        sub_axes_2.set_ylim(-10, 1200)

        sub_axes_1.set_ylabel('cm $H_{2}0$', rotation = 'horizontal')
        sub_axes_2.set_ylabel('L/min', rotation = 'horizontal')         
        sub_axes_1.spines["top"].set_visible(False)
        sub_axes_1.spines["right"].set_visible(False)
        sub_axes_2.spines["top"].set_visible(False)
        sub_axes_2.spines["right"].set_visible(False)
#         sub_axes_1.set_yticks(np.arange(0, 1000, 10))
#         sub_axes_2.set_yticks(np.arange(0, 1000, 20))
        sub_axes_1.grid(which = 'both', linestyle = '--', c = 'gray')
        sub_axes_2.grid(which = 'both', linestyle = '--', c = 'gray')
                 
# class Plotting_Graph2(tk.Frame):
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
# 
#         figure2 = Figure()
#         figure2.set_facecolor('midnightblue')
#         
#         sub_axes_12 = figure2.add_subplot(211)
#         sub_axes_22 = figure2.add_subplot(212)
#         sub_axes_12.set_facecolor('midnightblue')
#         sub_axes_22.set_facecolor('midnightblue')
#         sub_axes_22.axis('off')
#         sub_axes_12.axis('off')256000
#         
#         global y1, text1
#         text1 = sub_axes_12.text(0.3, 0.5, 'Current Pressure ' + str(round(y1[0], 2)) + ' cm $H_{2}O$', fontsize = 10)
# #         sub_axes_12.set_xlim(0, 1000)
#         global y2, text2    
#         text2 = sub_axes_22.text(0.3, 0.5, "Current Flow " +  str(round(y2[0], 2)) + ' L/min', fontsize = 10)
#        
#         figure2.set_tight_layout(True)
#         
#         figure_canvas2 = FigureCanvasTkAgg(figure2, self)
#         figure_canvas2.get_tk_widget().pack(side=tk.LEFT, fill = tk.BOTH, expand = True)
#         figure_canvas2._tkcanvas.pack(side = tk.RIGHT, expand = True)
#         
#         ani22 = animation.FuncAnimation(figure2, self.animate12, interval = 1000, blit = True)
#         ani23 = animation.FuncAnimation(figure2, self.animate22, interval = 1000, blit = True)
#               
#         def change_view2(event):
#             if event.inaxes:
#                 controller.show_frame(Plotting_Graph)
#                 
#         figure_canvas2.mpl_connect('button_press_event', change_view2)      
#         
#     def animate12(self, i):
#         global figure_canvas2,subplot_12,subplot_22,sub_axes_12 , sub_axes_22, figure2,y1, text1
# #         print(y1)
#         try:
#             text1.set_text('Current Pressure ' + str(round(y1[0], 2)) + ' cm $H_{2}O$')
# #            self.update()
#         except:
#             ''
#         return text1,
#     
#     def animate22(self, j):
#         global figure_canvas2,subplot_12,subplot_22,sub_axes_12 , sub_axes_22, figure2,y1, text1, y2
# #         print(y1)
#         try:
#             text2.set_text('Current Flow ' +  str(round(y2[0], 2)) + ' L/min')
# #            self.update()
#         except:
#             ''
#         return text2,
#         sub_axes_1.tick_params(which = 'major', length = 10, width = 2)
#         sub_axes_1.tick_params(which = 'minor', length = 6, width = 2)
#         sub_axes_1.xaxis.set_minor_locator(AutoMinorLocator(10+1))
#         sub_axes_1.yaxis.set_minor_locator(AutoMinorLocator(10+1))^C


        sub_axes_1.yaxis.set_label_coords(0, 1.02)        
        sub_axes_2.yaxis.set_label_coords(0, 1.02)
        
#         sub_axes_2.tick_params(which = 'major', length = 10, width = 2)
#         sub_axes_2.tick_params(which = 'minor', length = 1, width = 2)
        
        figure.set_tight_layout(True)
        
        figure_canvas = FigureCanvasTkAgg(figure, self)
        figure_canvas.get_tk_widget().pack(side=tk.LEFT, fill = tk.BOTH, expand = True)
        figure_canvas._tkcanvas.pack(side = tk.RIGHT, expand = True)
        
        self.initial_new_data(300) #Nick: Set range of x-axis

          
        ani1 = animation.FuncAnimation(figure, self.animate1, interval = 10, blit = True)
        ani2 = animation.FuncAnimation(figure, self.animate2, interval = 10, blit = True)
        
        sub_axes_1.axvline(50)#Nick: Draw the vertical lines. NOTE that this is only graphics- the rest of the code must still be updated.
        sub_axes_1.axvline(150)
        sub_axes_2.axvline(50)
        sub_axes_2.axvline(150)
        figure.set_tight_layout(True)
        label1.lift()
        label2.lift()
        label3.lift()
        label4.lift()
        global integral_flow
        integral_flow = []
        global time_flow
        time_flow = []
#         print(np.array(b"".join(ser.readlines(1000)).decode('utf-8').split('\r\n'))[1:-1].astype(np.float))
                
                
#         print(ser.read(9600).decode('utf-8').split('\r\n'))
#         def change_view(event):
#             if event.inaxes:
#                 controller.show_frame(Plotting_Graph2)
                
#         figure_canvas.mpl_connect('button_press_event', change_view)
        

    def initial_new_data(self, number_of_entries):
        global subplot_1, subplot_2, x, y1,y2, sub_axes_1, sub_axes_2, figure, figure_canvas, scatters
        x = [i for i in range(0, number_of_entries)]
        y1 = np.array([0 for i in range(0, number_of_entries)], dtype = np.float)
        y2 = np.array([0 for i in range(0, number_of_entries)], dtype = np.float)
    
        subplot_1, = sub_axes_1.plot(x, y1, c = 'lightskyblue')
        subplot_2, = sub_axes_2.plot(x, y2, c = 'lightskyblue')
        figure.set_tight_layout(True)
     
    def update_y1_date(self):
        global y1, label1, figure_canvas, ser,update_c100ounter1,update_counter2, y2, label2, integral_flow, time_flow

        
#         if update_counter2 >= update_counter1:
#        ser.flushInput()
        if ser.in_waiting > 0:
#            ser.flushInput()
            line = ser.read(12)
            time_flow.append(time.time())
            test_array = bytearray(line)
            y = np.frombuffer(test_array, dtype = np.uint8)
#            print(y)
            
            try:
                y1 = np.roll(y1,1)
                y2 = np.roll(y2,1)
                
                Pressure = y[1]<<8|y[0]
                
                if y[3] == 0:
                    Flow = y[2]
                    
                    
                    integral_flow.append(Flow)
                    Volume = trapz(np.array(integral_flow)*(1000/60), time_flow)
#                    if len(integral_flow) > 20:
#                        integral_flow = []
                else:
                    Flow = (-1)*y[2]
#                    print(len(time_flow))
#                    print(y[3])
                    time_flow = []
                    integral_flow = []
                    Volume = 0
                    
#                Volume = y[5]<<8|y[4]
                
                if y[-1] == 255:
                    y1[0] = Pressure
                    y2[0] = Volume
#                    y3[0] = Volume
                else:
                    y1[0] = y1[1]
                    y2[0] = y2[1]
#                    y3[0] = y3[1]
                ser.flushInput()
            except:
                y1[0] = float(y1[1])
                y2[0] = float(y2[1])
#                y3[0] = float(y3[1])
#                 y1[0] = np.median(np.array(ser.read(50).decode('utf-8').split('\r\n'))[1:-1].astype(np.float))
               
    #            print(ser.readline().decode('utf-8'))
            try:
                label1.config(text = str(round(y1[0], 2)))
                label2.config(text = str(round(y2[0], 2)))
            except:
                ''
#            update_counter1 += 1
        
        
#     def update_y2_date(self):
#         global y2, label2, ser,update_counter2, figure_canvas,update_counter1, y1
        
#         if ser.in_waiting >0:
#             ser.flushInput()
#         line = ser.readline()
#         test_array = bytearray(line)
#         y = np.frombuffer(test_array, dtype = np.uint8)
#         try:
#             y2 = np.roll(y2,1)
#             Flow = (((float(y[2])*100 + float(y[3]))))
#             y2[0] = Flow
#         except:
#             y2[0] = float(y2[1])
#         try:
#             label2.config(text = str(round(y2[0], 2)))
#         except:
#             ''
#         update_counter2 += 1
#         
    def animate1(self, i):
        global figure_canvas,subplot_1,subplot_2,sub_axes_1 , sub_axes_2, figure,x,y1,y2, scatters, label3, ser, label4
        self.update_y1_date()
        subplot_1.set_ydata(y1)  

        
        label3.config(text = "Max \n" + str(round(max(y1[50:150]), 2))) #Nick: ccyhange range here
        
    #        print(subplot_1.get_ydata())
        return subplot_1,
    
    def animate2(self, j):
        global figure_canvas,subplot_1,subplot_2,sub_axes_1 , sub_axes_2, figure,x,y1,y2, label4
#        self.update_y1_date()
        subplot_2.set_ydata(y2)
        label4.config(text = "Max \n" + str(round(max(y2[50:100]), 2))) #8
        return subplot_2,
    

         
# class Plotting_Graph2(tk.Frame):
#     def __init__(self, parent, controller):
#         tk.Frame.__init__(self, parent)
# 
#         figure2 = Figure()
#         figure2.set_facecolor('midnightblue')
#         
#         sub_axes_12 = figure2.add_subplot(211)
#         sub_axes_22 = figure2.add_subplot(212)
#         sub_axes_12.set_facecolor('midnightblue')
#         sub_axes_22.set_facecolor('midnightblue')
#         sub_axes_22.axis('off')
#         sub_axes_12.axis('off')256000
#         
#         global y1, text1
#         text1 = sub_axes_12.text(0.3, 0.5, 'Current Pressure ' + str(round(y1[0], 2)) + ' cm $H_{2}O$', fontsize = 10)
# #         sub_axes_12.set_xlim(0, 1000)
#         global y2, text2    
#         text2 = sub_axes_22.text(0.3, 0.5, "Current Flow " +  str(round(y2[0], 2)) + ' L/min', fontsize = 10)
#        
#         figure2.set_tight_layout(True)
#         
#         figure_canvas2 = FigureCanvasTkAgg(figure2, self)
#         figure_canvas2.get_tk_widget().pack(side=tk.LEFT, fill = tk.BOTH, expand = True)
#         figure_canvas2._tkcanvas.pack(side = tk.RIGHT, expand = True)
#         
#         ani22 = animation.FuncAnimation(figure2, self.animate12, interval = 1000, blit = True)
#         ani23 = animation.FuncAnimation(figure2, self.animate22, interval = 1000, blit = True)
#               
#         def change_view2(event):
#             if event.inaxes:
#                 controller.show_frame(Plotting_Graph)
#                 
#         figure_canvas2.mpl_connect('button_press_event', change_view2)      
#         
#     def animate12(self, i):
#         global figure_canvas2,subplot_12,subplot_22,sub_axes_12 , sub_axes_22, figure2,y1, text1
# #         print(y1)
#         try:
#             text1.set_text('Current Pressure ' + str(round(y1[0], 2)) + ' cm $H_{2}O$')
# #            self.update()
#         except:
#             ''
#         return text1,
#     
#     def animate22(self, j):
#         global figure_canvas2,subplot_12,subplot_22,sub_axes_12 , sub_axes_22, figure2,y1, text1, y2
# #         print(y1)
#         try:
#             text2.set_text('Current Flow ' +  str(round(y2[0], 2)) + ' L/min')
# #            self.update()
#         except:
#             ''
#         return text2,
    
if __name__ == "__main__":
    global plotter
    plt.style.use("dark_background")
    plotter = Main_page()
#    plotter.attributes("-fullscreen", True)
    plotter.mainloop()