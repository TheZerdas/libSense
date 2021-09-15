import libSense.get_data
# define path, RGBimg_name, Depthimg_name
# if you don't define, it will choose its own in current directory
path = (r"C:\Users\Python\....")
RGBname = "sensergb.jpg"
Depthname = "senseD.png"

# function that gets you RGB and Depth img
libSense.get_data.RGB(path, RGBname, Depthname)

