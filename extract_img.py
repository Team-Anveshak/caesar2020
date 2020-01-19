import cv2 
import os 
  
# Read the video from specified path 
cam = cv2.VideoCapture('ball14.avi') 
  
try: 
      
    # creating a folder named data 
    if not os.path.exists('ball2'): 
        os.makedirs('ball2') 
  
# if not created then raise error 
except OSError: 
    print ('Error: Creating directory of data') 
  
# frame 
currentframe = 1
count=1
  
while(True): 
      
    # reading from frame 
    ret,frame = cam.read() 
    count+=1

    if ret:
  
        if count%8==0: 
            # if video is still left continue creating images 
            name = './ball2/14ball' + str(currentframe) + '.jpg'
            print ('Creating...' + name) 
  
            # writing the extracted images 
            cv2.imwrite(name, frame) 
  
            # increasing counter so that it will 
        # show how many frames are created 
            currentframe += 1
    else:
        break
    
  
# Release all space and windows once done 
cam.release() 
cv2.destroyAllWindows() 
