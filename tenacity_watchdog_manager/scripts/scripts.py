import cv2  
import numpy as np  
   
 # create a blank image, black background  
blank_image = np.zeros((500,1000,3), np.uint8)  
   
text_to_show = "The quick brown fox jumps over the lazy dog"  
   
cv2.putText(blank_image,  
           "Hershey Simplex : " + text_to_show,  
           (20, 40),  
           fontFace=cv2.FONT_HERSHEY_SIMPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Plain : " + text_to_show,  
           (20, 80),  
           fontFace=cv2.FONT_HERSHEY_PLAIN,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Duplex : " + text_to_show,  
           (20, 120),  
           fontFace=cv2.FONT_HERSHEY_DUPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Complex : " + text_to_show,  
           (20, 160),  
           fontFace=cv2.FONT_HERSHEY_COMPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Triplex : " + text_to_show,  
           (20, 200),  
           fontFace=cv2.FONT_HERSHEY_TRIPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Complex Small : " + text_to_show,  
           (20, 240),  
           fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Script Simplex : " + text_to_show,  
           (20, 280),  
           fontFace=cv2.FONT_HERSHEY_SCRIPT_SIMPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.putText(blank_image,  
           "Hershey Script Complex : " + text_to_show,  
           (20, 320),  
           fontFace=cv2.FONT_HERSHEY_SCRIPT_COMPLEX,  
           fontScale=1,  
           color=(255, 255, 255))  
   
cv2.imshow('Fonts', blank_image)  
cv2.waitKey(0)  
   
cv2.destroyAllWindows()  
