import threading
import numpy as np 
import cv2


#####HSV Colour Ranges#################
# red  --> values optained using ChatGPT
'''
redLow1 = np.array([0, 70, 50])
redHigh1 = np.array([10, 255, 255])
redLow2 = np.array([170, 70, 50])
redHigh2 = np.array([180, 255, 255])
'''
redLow1  = np.array([0, 140, 80])
redHigh1 = np.array([10, 255, 255])

redLow2  = np.array([170, 140, 80])
redHigh2 = np.array([180, 255, 255])



# yellow  --> values optained using ChatGPT
yellowLowMask = (20, 100, 100)
yellowHighMask = (35, 255, 255)
class Tracker:
    def __init__(self, pointColor, goalColor):
        self.point = (0, 0, 0)
        self.goal = (0, 0, 0)
        self.home = None
        thread = threading.Thread(target=self.TrackerThread, args=(pointColor, goalColor), daemon=True)
        thread.start()
        
    def get_pixels(self):
        if self.point is None or self.home is None:
            return [0,0], [0,0]
        point, goal = self.point,   self.home #self.goal
        #print(self.goal)
        #print(goal)
        point = np.asarray(point).reshape(-1, 3)[0]  # ensures (3,)
        goal = np.asarray(goal).reshape(-1, 3)[0]
        point = [float(point[0]), float(point[1])]
        goal = [float(goal[0]), float(goal[1])]
        return point, goal
    
    def get_error(self,point, goal):
        point,goal = self.get_pixels()
        ex =  goal[0] - point[0]
        ey =  point[1] - goal[1]
        return [ex, ey]


    def TrackerThread(self, pointColor, goalColor):
        print("Tracker Started")
        # Get the camera
        #vc = cv2.VideoCapture(1)
        #vc = cv2.VideoCapture(0)
        vc = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        vc.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        vc.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        vc.set(cv2.CAP_PROP_FPS, 60)
        vc.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        print("FPS--------------:", vc.get(cv2.CAP_PROP_FPS))
        if vc.isOpened():
            rval, frame = vc.read()
        else:
            rval = False

        while rval:
            # Handle current frame
            rval, frame = vc.read()
            for _ in range(2):
                vc.grab()

            # get end effector and goal tracker image coordinates
            circlesPoint = self.GetLocation(frame, pointColor)
            circlesGoal = self.GetLocation(frame, goalColor)
            self.DrawCircles(frame, circlesPoint, (255, 0, 0))
            self.DrawCircles(frame, circlesGoal, (0, 0, 255))

            # update end effector and goal positions
            if circlesPoint is not None:
                self.point = circlesPoint[0]
                
            

            if circlesGoal is not None:
                self.goal = circlesGoal[0]
                if self.home is None:
                    self.home = circlesGoal[0].copy()

            # Show original image with the detected circles drawn.
            cv2.imshow("Result", frame)

            # check if esc key pressed
            key = cv2.waitKey(20)
            if key == 27:
                break

        vc.release()
        cv2.destroyAllWindows()
        print("Tracker Ended")
    '''
    def GetLocation(self, frame, color):
        # Uncomment for gaussian blur
        # blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        blurred = cv2.medianBlur(frame, 11)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if color == "r":
            mask1 = cv2.inRange(hsv, redLow1, redHigh1)
            mask2 = cv2.inRange(hsv, redLow2, redHigh2)
            mask = cv2.bitwise_or(mask1, mask2)
        if color == "y":
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)

        # Perform erosion and dilation in the image (in 11x11 pixels squares) in order to reduce the "blips" on the mask
        mask = cv2.erode(mask, np.ones((11, 11), np.uint8), iterations=2)
        mask = cv2.dilate(mask, np.ones((11, 11), np.uint8), iterations=5)

        # Mask the blurred image so that we only consider the areas with the desired colour
        masked_blurred = cv2.bitwise_and(blurred, blurred, mask=mask)
        # masked_blurred = cv2.bitwise_and(frame,frame, mask= mask)
        # Convert the masked image to gray scale (Required by HoughCircles routine)
        result = cv2.cvtColor(masked_blurred, cv2.COLOR_BGR2GRAY)
        # Detect circles in the image using Canny edge and Hough transform
       
        circles = cv2.HoughCircles(
            result,
            cv2.HOUGH_GRADIENT,
            1.5,
            300,
            param1=80,     # a bit softer
            param2=10,     # more sensitive
            minRadius=2,   # <--- key change
            maxRadius=80,  # adjust to your dot size
        )
        return circles
    '''
    def GetLocation(self, frame, color):
    # 1. Blur a bit to reduce noise
        blurred = cv2.GaussianBlur(frame, (5, 5), 0)

    # 2. Convert to HSV
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 3. Color mask
        if color == "r":
            mask1 = cv2.inRange(hsv, redLow1, redHigh1)
            mask2 = cv2.inRange(hsv, redLow2, redHigh2)
            mask = cv2.bitwise_or(mask1, mask2)
        elif color == "y":
            mask = cv2.inRange(hsv, yellowLowMask, yellowHighMask)
        else:
            return None

    # 4. Clean mask: open + close instead of huge erode/dilate
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Optional: debug view
    # cv2.imshow(f"mask_{color}", mask)

    # 5. Find contours
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(cnts) == 0:
            return None

    # 6. Largest contour = our circle marker
        c = max(cnts, key=cv2.contourArea)
        (x, y), radius = cv2.minEnclosingCircle(c)

        if radius < 3:  # ignore tiny noise blobs
            return None
        
        if color == "r":
            radius = min(radius, 20)

        x = int(x)
        y = int(y)
        radius = int(radius)

    # Return in same "circles" format as Hough so rest of code still works:
        circles = np.array([[[x, y, radius]]], dtype=np.float32)
        return circles



    def DrawCircles(self, frame, circles, dotColor):
        # ensure at least some circles were found
        if circles is not None:
            # convert the (x, y) coordinates and radius of the circles to integers
            circles = np.round(circles[0, :]).astype("int")
            # loop over the (x, y) coordinates and radius of the circles
            for x, y, r in circles:
                # print("Circle: " + "("+str(x)+","+str(y)+")")
                # draw the circle in the output image, then draw a rectangle corresponding to the center of the circle
                # The circles and rectangles are drawn on the original image.
                cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), dotColor, -1)