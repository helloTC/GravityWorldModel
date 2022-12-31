import tkinter as tk
import numpy as np
import os

def prepare_tkscale(result_tmp):
    """
    Main Function to generate a tkinterace scale frame.
    """
    window = tk.Tk()
    window.geometry("+800+270")
    s1 = tk.Scale(window,from_=0,to=7,tickinterval=1,resolution=1,length=500)
    s1.pack()

    def save_judgement():
        result_tmp.append(s1.get()) 
        np.save('judgment_stability_tmp.npy', result_tmp)
        window.destroy()

    b1 = tk.Button(window, text='Confirm', command=save_judgement)
    b1.pack()

    window.mainloop()

def generate_scaleframe(filename):
    """
    """
    if os.path.isfile(filename):
        result_tmp = list(np.load(filename))
    else:
        result_tmp = []
    prepare_tkscale(result_tmp)

if __name__ == '__main__':
    generate_scaleframe('judgment_stability_tmp.npy')
