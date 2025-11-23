import cv2          # The OpenCV library
import numpy as np    # For array manipulation
import pyautogui    # The library that takes the screenshot

def takeScreenshot():
       # 1. Take the screenshot (returns a PIL/Pillow Image object)
       print("Taking screenshot...")
       pil_image = pyautogui.screenshot()
       print("Screenshot captured.")
       # 2. Convert the PIL Image to a NumPy Array
       # This is crucial for OpenCV. PIL uses RGB, so we convert it to a NumPy array first.
       numpy_array = np.array(pil_image)

       # 3. Convert the color format: RGB (pyautogui) -> BGR (OpenCV)
       # OpenCV, historically, uses BGR (Blue, Green, Red) instead of the more common RGB.
       # If you skip this, the colors will look "off" (e.g., blue will look red).
       screenshot_bgr = cv2.cvtColor(numpy_array, cv2.COLOR_RGB2BGR)

       # 4. Save the screenshot to a file
       filename = 'opencv_screenshot.png'
       cv2.imwrite(filename, screenshot_bgr)
       print(f"Screenshot saved as {filename}")

       #Display the image 
       cv2.imshow('Screenshot', screenshot_bgr)
       cv2.waitKey(0)        # Wait indefinitely until a key is pressed
       cv2.destroyAllWindows() # Close the display window
if __name__ == "__main__":
    takeScreenshot()

