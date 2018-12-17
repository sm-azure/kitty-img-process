import pickle
import cv2
import numpy as np

class FindLanes(object):

    def __init__(self, calibration_file):
        self._calibration_file = calibration_file
        serialize = pickle.load( open( self._calibration_file, "rb" ) )
        self._mtx = serialize["mtx"]
        self._dist = serialize["dist"]
    

    # Process the image and give it back
    def img_process(self, img_BGR):
        # Conver from BGR to RGB to process 
        img_RGB = cv2.cvtColor(img_BGR, cv2.COLOR_BGR2RGB)
        color_binary, combined, RGB,  src, dst, M  = self.combined_threshold_mask(img_RGB)
        
        leftx, lefty, rightx, righty, out_imgS, confidence_left, confidence_right, center_point, cte = self.find_lane_pixels_no_history(combined)
        left_fitx, right_fitx, out_imgS, left_fit, right_fit = self.fit_polynomial(leftx, lefty, rightx, righty, out_imgS)
        
        return color_binary, cte # in RGB format

    # Undistort image 
    def undistort(self, img, mask = True):    
        return cv2.undistort(img, self._mtx, self._dist, None, self._mtx)

    # Create a mask remove side noise
    def apply_transform_mask(self, _img , offset=100, bottom_offset = 100):
        _img[_img.shape[0] - bottom_offset:_img.shape[0]:] = 0
        size= _img.shape[0]
        return _img[:size - bottom_offset,:]
        
    # Create birds eye view from undistorted image
    def birds_perspective(self, img, SPEED = 0):    
        # Define source points
        src = np.zeros((4, 2), dtype=np.float32)

        src[0] = [430,550]
        src[1] = [850,550]
        src[2] = [1180,960]
        src[3] = [100,960]
        
        # Destination Points
        dst = np.zeros((4, 2), dtype=np.float32)
        width = img.shape[1]
        height = img.shape[0]
        offset1 = 250 # offset for dst points
        offset2 = 150
        dst[0] = (offset1, 0)
        dst[1] = (width-offset1, 0)
        dst[2] = (width-offset1, height)
        dst[3] = (offset1, height)
        
        M = cv2.getPerspectiveTransform(src, dst)
        dest_img = cv2.warpPerspective(img, M, (width, height),flags=cv2.INTER_LINEAR)
        
        return src, dst, M, dest_img

    # Apply generic threshold for high value targets
    def apply_threshold_channel(self, _img_channel, thresh=(180, 255)):
        binary = np.zeros_like(_img_channel)
        binary[(_img_channel > thresh[0]) & (_img_channel <= thresh[1])] = 1
        return binary 

    # Create mask
    def combined_threshold_mask(self, _img_RGB, show = False, distance=1):
        img_RGB = self.undistort(_img_RGB) 
        src, dst, M, bird_RGB = self.birds_perspective(img_RGB, distance)
        RGB = self.apply_transform_mask(bird_RGB, 100)
        
        gray = cv2.cvtColor(RGB, cv2.COLOR_RGB2GRAY)
        S = self.apply_threshold_channel(gray)
        
                                        
        #Combine
        combined = np.zeros_like(gray)
        blank = np.copy(combined)
        # Color stack
        color_binary = np.dstack((blank, S, blank)) * 255
        # Combined 
        combined[(S==1) ] = 1 # (R_gradx == 1) | 

        return color_binary, combined, RGB,  src, dst, M 

    # Finding lane pixels - from Course material. If you dont know anything about the frame
    def find_lane_pixels_no_history(self, binary_warped):
        # Find confidence of the curve you are providing
        confidence_left = 0
        confidence_right = 0
        # Take a histogram of the bottom half of the image
        histogram = np.sum(binary_warped[binary_warped.shape[0]//2:,:], axis=0)
        # Create an output image to draw on and visualize the result
        out_img = np.dstack((binary_warped, binary_warped, binary_warped))
        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0]//2)
        leftx_base = np.argmax(histogram[:midpoint])
        rightx_base = np.argmax(histogram[midpoint:]) + midpoint
        # Find the offset over center_point_windows windows (immediately infront of the car)
        left_point = 0
        right_point = 0
        center_point_windows = 2
        
        cte = 0.0
        

        # HYPERPARAMETERS
        # Choose the number of sliding windows
        nwindows = 9
        # Set the width of the windows +/- margin
        # Window width exapand to eventually cover atleast half the window
        l_margin = 100
        r_margin = 100
        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Set height of windows - based on nwindows above and image shape
        window_height = np.int(binary_warped.shape[0]//nwindows)
        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = binary_warped.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        # Current positions to be updated later for each window in nwindows
        leftx_current = leftx_base
        rightx_current = rightx_base

        # Create empty lists to receive left and right lane pixel indices
        left_lane_inds = []
        right_lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y (and right and left)
            win_y_low = binary_warped.shape[0] - (window+1)*window_height
            win_y_high = binary_warped.shape[0] - window*window_height
            ### TO-DO: Find the four below boundaries of the window ###
            # Ideally there should be a check to ensure that this window does not cross half way points, low and high
            win_xleft_low = (leftx_current - l_margin) if  (leftx_current - l_margin)> 0 else 0 # Update this
            win_xleft_high = (leftx_current + l_margin) if (leftx_current + l_margin) < binary_warped.shape[1]//2 else binary_warped.shape[1]//2  # Update this
            win_xright_low = (rightx_current - r_margin) if (rightx_current - r_margin) > binary_warped.shape[1]//2 else binary_warped.shape[1]//2   # Update this
            win_xright_high = (rightx_current + r_margin) if (rightx_current + r_margin) < binary_warped.shape[1] else binary_warped.shape[1]  # Update this
            
            # Draw the windows on the visualization image
            cv2.rectangle(out_img,(win_xleft_low,win_y_low),
            (win_xleft_high,win_y_high),(0,255,0), 2) 
            cv2.rectangle(out_img,(win_xright_low,win_y_low),
            (win_xright_high,win_y_high),(255,255,0), 2) 
            
            ### TO-DO: Identify the nonzero pixels in x and y within the window ###
            good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xleft_low) &  (nonzerox < win_xleft_high)).nonzero()[0]
            good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & 
            (nonzerox >= win_xright_low) &  (nonzerox < win_xright_high)).nonzero()[0]
            
            # Append these indices to the lists
            left_lane_inds.append(good_left_inds)
            right_lane_inds.append(good_right_inds)
            #print(left_lane_inds)
            
            ### TO-DO: If you found > minpix pixels, recenter next window ###
            ### (`right` or `leftx_current`) on their mean position ###
            #print (left_lane_inds)
            if len(good_left_inds) > minpix*3:
                # Increase confidence and reduce window margin to default
                confidence_left += 1
                l_margin = 100
                leftx_current = np.int(np.mean(nonzerox[good_left_inds]))    
            else:
                l_margin = int(1.25* l_margin)
            if len(good_right_inds) > minpix*3:        
                # Increase confidence and reduce window margin to default
                confidence_right += 1
                r_margin = 100
                rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
            else:
                #Lets increase window margin width by 25% (incase lane is turning)
                r_margin = int(1.25* r_margin)
                
            # For few windows consider offset
            if(window < center_point_windows):
                # Add left_center point
                left_point += leftx_current
                # Add left_center point
                right_point += rightx_current
                # Compute cte for box
                #cte += (left_point + right_point)//2 - binary_warped.shape[1]//2
            
            #Calculate max error
            if(window == nwindows-1 ):
                cte = (leftx_current + rightx_current)//2 - binary_warped.shape[1]//2
                

        # Concatenate the arrays of indices (previously was a list of lists of pixels)
        try:
            left_lane_inds = np.concatenate(left_lane_inds)
            right_lane_inds = np.concatenate(right_lane_inds)
        except ValueError:
            # Avoids an error if the above is not implemented fully
            pass

        # Extract left and right line pixel positions
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds] 
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        
        #Computer center point
        center_point = (left_point/center_point_windows + right_point/center_point_windows)//2

        return leftx, lefty, rightx, righty, out_img, confidence_left, confidence_right, center_point, cte
    
    # Finding Polynomial Fit
    def fit_polynomial(self, leftx, lefty, rightx, righty, out_img):
    
        ### TO-DO: Fit a second order polynomial to each using `np.polyfit` ###
        left_fit = np.polyfit(lefty, leftx, 2)
        right_fit = np.polyfit(righty, rightx, 2)
        
        ploty = np.linspace(0, out_img.shape[0]-1, out_img.shape[0] )
        
        try:
            left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
            right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
        except TypeError:
            # Avoids an error if `left` and `right_fit` are still none or incorrect
            print('The function failed to fit a line!')
            left_fitx = 1*ploty**2 + 1*ploty
            right_fitx = 1*ploty**2 + 1*ploty

        ## Visualization ##
        # Colors in the left and right lane regions
        out_img[lefty, leftx] = [255, 0, 0]
        out_img[righty, rightx] = [255, 0, 0]

        return left_fitx, right_fitx, out_img, left_fit, right_fit