import os
import PIL
import numpy as np
import matplotlib.pyplot as plt
cwd = os.getcwd()

ind = '09'
file_name = "/home/xintong/Documents/PyProjects/zivid_camera/imgs/"+ind+".png"

file = PIL.Image.open(file_name).convert('LA')
file.save("/home/xintong/Documents/PyProjects/zivid_camera/imgs/"+ind+"_gray.png")