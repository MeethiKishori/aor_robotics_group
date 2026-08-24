import cv2
import numpy as np
from pathlib import Path


h_low1, h_high1 = 0,12
h_low2, h_high2 = 150 , 180
sat_min = 124
val_brig_min = 140
minbloob_area = 10

#test on an image not on an video


def detectredled(image, h_low1, h_low2, sat_min, val_brig_min):

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) # convert BGR to HSV color space
    H,S,V = cv2.split(hsv) # split HSV channels
    
    cv2.imshow('original image after hsv is ', hsv)    #  show the original image in HSV color space
    hsv_after = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR) # convert back to BGR color space
    #cv2.imshow('converted to real image ', hsv_after)

    mask1 = cv2.inRange(hsv, np.array([h_low1,sat_min, val_brig_min]),np.array([h_high1, 255,255]))  # when not in range 0,then black and when in range then white
    mask2 = cv2.inRange(hsv, np.array([h_low2,sat_min, val_brig_min]), np.array([h_high2, 255,255])) # for upper values of red
    #cv2.imshow('mask1',mask1)    # show the mask for red hue in low range
    #cv2.imshow('mask2',mask2)

    mask1_2  = cv2.bitwise_or(mask1, mask2)
    #cv2.imshow('mask1_2',mask1_2) # show the combined mask for red hue
    return mask1_2
    
    


def sliderun(image):
    global h_low1, h_low2, sat_min, val_brig_min
    cv2.namedWindow('Slider', cv2.WINDOW_NORMAL)
    cv2.createTrackbar('Hue_red_low1', 'Slider', h_low1, 20, lambda x: None)
    cv2.createTrackbar('Hue_red_low2', 'Slider', h_low2, 180, lambda x: None)
    cv2.createTrackbar('Saturation', 'Slider', sat_min, 255, lambda x: None)
    cv2.createTrackbar('Brightness', 'Slider', val_brig_min, 255, lambda x: None)
    cv2.waitKey(1)

    while True:
        h_low1 = cv2.getTrackbarPos('Hue_red_low1', 'Slider')
        h_low2 = cv2.getTrackbarPos('Hue_red_low2', 'Slider')
        sat_min = cv2.getTrackbarPos('Saturation', 'Slider')
        val_brig_min = cv2.getTrackbarPos('Brightness', 'Slider')

        mask1_2 = detectredled(image, h_low1, h_low2, sat_min, val_brig_min)
        display = cv2.cvtColor(mask1_2, cv2.COLOR_GRAY2BGR)
        
        labels = [
            f"Only Red control active",
            f"Hue_low1  : {h_low1}  (0-20)",
            f"Hue_low2  : {h_low2}  (0-180)",
            f"Saturation: {sat_min}  (0-255)",
            f"Brightness: {val_brig_min}  (0-255)",
        ]
        for i, text in enumerate(labels):
            cv2.putText(display, text, (10, 30 + i * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow('Slider', display) 

        


        if cv2.waitKey(1) == ord('q'):
            break

    cv2.destroyAllWindows()


def main():
    image_path = Path(__file__).parents[3] / "recordings" / "images2.jpg"
    print(f"found at {image_path}")
    image = cv2.imread(str(image_path))
    cv2.imshow('original image is ', image)
    #detectredled(image, h_low1)
    sliderun(image)




if __name__ == '__main__':
    main()
