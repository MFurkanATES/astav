
# import the opencv library
import cv2  
# define a video capture object
vid = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter('appsrc  ! h264parse ! '
                      'rtph264pay config-interval=1 pt=96 ! '
                      'gdppay ! tcpserversink host=192.168.1.27 port=5000 ',
                      fourcc, 20.0, (640, 480))  
while(True):
      
    # Capture the video frame
    # by frame
    ret, frame = vid.read()
  
    # Display the resulting frame
    out.write(frame)
    cv2.imshow('frame', frame)
      
    # the 'q' button is set as the
    # quitting button you may use any
    # desired button of your choice
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
  
# After the loop release the cap object
vid.release()
# Destroy all the windows
cv2.destroyAllWindows()