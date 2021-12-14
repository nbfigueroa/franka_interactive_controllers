#!/usr/bin/env python3
import shlex
from psutil import Popen
from tkinter import *
from tkinter import messagebox

root = Tk()
root.title("Franka Gripper Control")
root.geometry("300x75+1000+0")

def open(): 
   node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 1'))
   messagebox.showinfo("Open Gripper", "Gripper Opened")
   node_process.terminate()

def close():   
   node_process = Popen(shlex.split('rosrun franka_interactive_controllers franka_gripper_run_node 0'))
   messagebox.showinfo("Close Gripper", "Gripper Closed")
   node_process.terminate()

B1 = Button(root, text = "Open Gripper", command = open)
B1.place(x = 30,y = 20)

B2 = Button(root, text = "Close Gripper", command = close)
B2.place(x = 160,y = 20)


root.mainloop()
