import pyrealsense2 as rs, numpy as np
from PIL import Image

p = rs.pipeline()
c = rs.config()
c.enable_stream(rs.stream.infrared, 1, 1280, 720, rs.format.y8, 30)
p.start(c)
f = p.wait_for_frames().get_infrared_frame(1)  # gives raw pixel buffe r for left IR camera, with values  0-255 representing brightness of each pixel in the IR image
p.stop()
img = np.asarray(f.get_data())  # convert raw IR frame data to a NumPy image array, lets me index pixels
print(img) # it prints 720 and 1280, in 2d representation, each value is the brightness of that pixel in the IR image, from 0 (black) to 255 (white)
print("first 10 IR pixel values:", img.ravel()[:10]) # it flattens the 2D image array into a 1D array and prints the first 10 pixel brightness values, which should be integers between 0 and 255
Image.fromarray(img).save("infrared.png") # takes the 2D NumPy array of pixel brightness values and saves it as a grayscale PNG image file named "infrared.png"
#print("Saved infrared.png")

print(f"image shape is {img.shape}")    # should be (720, 1280) for 720p IR stream 720 rows, height, and 1280 columns, width
a = int(input("press any index value of pixel to see its brightness (0-255): "))
b = int(input("press any index value of pixel to see its brightness (0-255): "))
print(f"pixel value at [ {a},{b} ] is {img[a,b]}")

img = np.frombuffer(f.get_data(), dtype=np.uint8).reshape((720, 1280))
print(img)  # does the same as print(img)  