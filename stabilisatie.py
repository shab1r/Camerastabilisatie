# import libraries
from vidgear.gears import VideoGear
from vidgear.gears import WriteGear
import cv2
import datetime
import os

output_path = "output1234.avi"  # or other video file name
codec = {"-fourcc":"h264"}

writer = WriteGear(output_path, codec, logging=True)  # create a WriteGear object with the desired output file path

stream = VideoGear(source=0, stabilize = True).start() # To open any valid video stream(for e.g device at 0 index)


# infinite loop
while True:

    frame = stream.read()# read stabilized frames
    frame = cv2.resize(frame, (1280, 720))
    # check if frame is None
    if frame is None:
        #if True break the infinite loop
        break

    # cv2.rectangle(frame, (0,0), (1280, 720), (0, 255, 0), 200)
    cv2.rectangle(frame, (0,0), (1920, 1080), (0, 255, 0), 200)
    frame = frame[100:650, 100:1200] # crop the frame to the desired size [y:y+h, x:x+w]

    # do something with stabilized frame here
    writer.write(frame)  # write the frame to the output filerq
    cv2.imshow("Stabilized Frame", frame)
    # Show output window

    key = cv2.waitKey(1) & 0xFF
    # check for 'q' key-press
    if key == ord("q"):
        #if 'q' key-pressed break out
        break

cv2.destroyAllWindows()
# close output window

writer.close()  # release the writer resources

stream.stop()
# safely close video stream