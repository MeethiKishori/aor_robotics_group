import cv2
import numpy as np

# 1. Load your image
# Replace 'test.jpg' with the actual path to your image file
image = cv2.imread("test.jpg")

if image is None:
    print("Error: Could not open or find the image. Check the file path!")
    exit()

# 2. Convert the image from BGR (OpenCV default) to HSV color space
hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

# 3. Define the Red color ranges
# Red is tricky because it sits at both the very beginning and the very end of the Hue wheel (0-180).
# We define two ranges and combine them.

# Range 1: Covers orange-red (Hue from 0 to 10)
lower_red1 = np.array([0, 120, 70], dtype=np.uint8)
upper_red1 = np.array([10, 255, 255], dtype=np.uint8)

# Range 2: Covers pink-red (Hue from 170 to 180)
lower_red2 = np.array([170, 120, 70], dtype=np.uint8)
upper_red2 = np.array([180, 255, 255], dtype=np.uint8)

# 4. Create masks to isolate the red pixels
mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

# Combine both masks using a bitwise OR operation
# (A pixel is kept if it matches either the lower red or upper red range)
full_red_mask = cv2.bitwise_or(mask1, mask2)

# 5. Apply the mask onto the original image to cut out the red objects
# This turns everything black EXCEPT the areas that match our mask
red_isolated_result = cv2.bitwise_and(image, image, mask=full_red_mask)

# 6. Display the results in windows
cv2.imshow("Original Image", image)
cv2.imshow("Red Mask (White = Red detected)", full_red_mask)
cv2.imshow("Isolated Red Result", red_isolated_result)

print("Displaying windows. Press any key while focused on a window to close.")
cv2.waitKey(0)
cv2.destroyAllWindows()