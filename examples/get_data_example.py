import libSense.get_data
# define path, RGBimg_name, Depthimg_name
# if you don't define, it will choose its own in current directory
path = (r"C:\Users\Python\....")
RGBname = "sensergb.jpg"
Depthname = "senseD.png"

# function that gets you RGB and Depth img
data = libSense.get_data(path, RGBname, Depthname)
# just get data and save it
libSense.get_data.RGB(data)
# set live stream with optional save data
# libSense.get_data.RGBLive(data)

