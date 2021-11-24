import cv2
import cv_bridge
import numpy as np
from sensor_msgs.msg import Image

class ImageProcessing:
    # Color boundaries for pucks
    RED_PUCK_COLOR_BOUNDARIES = ([0, 0, 50], [20, 20, 150])
    GREEN_PUCK_COLOR_BOUNDARIES = ([0, 50, 0], [20, 150, 20])
    BLUE_PUCK_COLOR_BOUNDARIES = ([50, 20, 10], [90, 90, 60])

    def __init__(self, node, show_image=False, save_image_path=None):
        
        # Parent node
        self.__node = node

        # Flag to show image 
        self.__show_image = show_image
        
        # Path to save the image 
        self.__save_image_path = save_image_path
        
        # dict that will hold data that will be be returned to main node 
        self.__puck_color_left_to_right = { 'left': '', 
                                            'center': '',
                                            'right': '',
                                            'error': True }
        # cv bridge provided by ROS2
        self.__bridge = cv_bridge.CvBridge()
        
        # Flag that will tell us if we are subscribed to robot_camera node
        self.__image_processing_subscription = None

        self.__node.get_logger().info(f'IMAGE PROCESSING init done')

    # Will return the dict with data or None if the image processing is not yet started or subscription already destroyed
    def get_puck_colors(self):
        return self.__puck_color_left_to_right 

    # For distroying the image processing node. Should be done when we are done with image processing 
    def destroy_image_processing_subscription(self):
        # Check if we are subscribed to image processing
        if self.__image_processing_subscription is not None:
            self.__node.destroy_subscription(self.__image_processing_subscription)  
            self.__image_processing_subscription = None
    
    # Subscrives to the robot_camera node
    def start_image_processing(self):
        self.__image_processing_subscription = self.__node.create_subscription(Image, '/robot/robot_camera', self.__image_data_callback, 1)

    # Callback that will be called when we get data from camera
    def __image_data_callback(self, image_data):

        # As we need only one image to process the colors we can destroy the subscription right away
        self.destroy_image_processing_subscription()
        
        # Get the image from ROS2 message 
        output = self.__bridge.imgmsg_to_cv2(image_data)

        # Get puck colors
        self.__get_puck_colors(output)


    def __get_puck_colors(self, image):
        temp_image = image
        colors = []
        x_axis_data = []
        
        # Make a mask, so we look only at segment we need 
        mask = np.zeros(temp_image.shape[:2], dtype="uint8")
        cv2.rectangle(mask, (160, 10), (355, 110), 255, -1)
        
        # Apply mask to the image 
        temp_image = cv2.bitwise_and(temp_image, temp_image, mask=mask)

        # Get gray image. The gray image is used for puck detection 
        temp_gray_image = cv2.cvtColor(temp_image, cv2.COLOR_BGR2GRAY)

        # Detect circles (pucks) on image  
        circles = cv2.HoughCircles(temp_gray_image, cv2.HOUGH_GRADIENT, dp=3.1, minDist=15, minRadius=7, maxRadius=35)
        
        # Check if we have detected any circles
        if circles is not None:

            # Round the circle coordinates
            circles = np.round(circles[0, :]).astype("int")

            # Loop thru all detected circles
            for (x, y, r) in circles:

                # Check if we need to mark the circles on the image and mark them if needed
                if self.__save_image_path is not None or self.__show_image is not None:
                    cv2.circle(temp_image, (x, y), r, (0, 255, 0), 4)   # Mark edge of the circle
                    cv2.rectangle(temp_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)    # Mark the center of circle

                red_success_count = 0
                green_success_count = 0
                blue_success_count = 0

                # Loop thru 8bit color data of current x, y coordinated if the image and check the color  
                for i in range(3):
                    # Check if the image pixel on x,y coordinates is between self.RED_PUCK_COLOR_BOUNDARIES
                    if image[y,x][i] >= self.RED_PUCK_COLOR_BOUNDARIES[0][i] and image[y,x][i] <= self.RED_PUCK_COLOR_BOUNDARIES[1][i]:
                        red_success_count += 1  
                        
                        # If 3 times we have success we have red puck
                        # We check 3 times because we have array of 3 8bit colors (0 - 255) and we have to find the right mix
                        # of those colors because red puck can have some blue or green color 
                        if red_success_count == 3:    
                            colors.append('red')    # Save color data   
                            x_axis_data.append(x)   # Save x axis data so we can detect pucks from left to right
                            
                            # Check if we need to mark the colors on the image and mark them if needed
                            if self.__save_image_path is not None or self.__show_image is not None:
                                cv2.putText(img=temp_image, text="R", org=(x-5 , y-20), color=(0,0,255), fontFace=1, fontScale=1, thickness=2)
                            break
                    
                    if image[y,x][i] >= self.GREEN_PUCK_COLOR_BOUNDARIES[0][i] and image[y,x][i] <= self.GREEN_PUCK_COLOR_BOUNDARIES[1][i]:
                        green_success_count += 1
                        if green_success_count == 3:
                            colors.append('green')
                            x_axis_data.append(x)
                            if self.__save_image_path is not None or self.__show_image is not None:
                                cv2.putText(img=temp_image, text="G", org=(x-5 , y-20), color=(0,255,0), fontFace=1, fontScale=1, thickness=2)
                            break
                    
                    if image[y,x][i] >= self.BLUE_PUCK_COLOR_BOUNDARIES[0][i] and image[y,x][i] <= self.BLUE_PUCK_COLOR_BOUNDARIES[1][i]:
                        blue_success_count += 1
                        if blue_success_count == 3:
                            colors.append('blue')
                            x_axis_data.append(x)
                            if self.__save_image_path is not None or self.__show_image is not None:
                                cv2.putText(img=temp_image, text="B", org=(x-5 , y-20), color=(255,0,0), fontFace=1, fontScale=1, thickness=2)
                            break
            # Check if we need to save the image
            if self.__save_image_path is not None:
                cv2.imwrite(self.__save_image_path, np.hstack([temp_image, image]))

            # Check if we need to show the image
            if self.__show_image:
                cv2.imshow("output", np.hstack([temp_image, image]))
                cv2.waitKey(0)

            # Check if theres 3 circles detected. If not there's an error
            if len(circles) == 3:
                # Check the x axis data and determine the colors from left to right 
                if x_axis_data[0] <  x_axis_data[1] and x_axis_data[0] <  x_axis_data[2]:
                    self.__puck_color_left_to_right['left'] = colors[0]
                    if x_axis_data[1] <  x_axis_data[2]:
                        self.__puck_color_left_to_right['center'] = colors[1]
                        self.__puck_color_left_to_right['right'] = colors[2]
                    else:
                        self.__puck_color_left_to_right['center'] = colors[2]
                        self.__puck_color_left_to_right['right'] = colors[1]
                
                elif x_axis_data[1] <  x_axis_data[0] and x_axis_data[1] <  x_axis_data[2]:
                    self.__puck_color_left_to_right['left'] = colors[1]
                    if x_axis_data[0] <  x_axis_data[2]:
                        self.__puck_color_left_to_right['center'] = colors[0]
                        self.__puck_color_left_to_right['right'] = colors[2]
                    else:
                        self.__puck_color_left_to_right['center'] = colors[2]
                        self.__puck_color_left_to_right['right'] = colors[0]

                elif x_axis_data[2] <  x_axis_data[0] and x_axis_data[2] <  x_axis_data[1]:
                    self.__puck_color_left_to_right['left'] = colors[2]
                    if x_axis_data[0] <  x_axis_data[1]:
                        self.__puck_color_left_to_right['center'] = colors[0]
                        self.__puck_color_left_to_right['right'] = colors[1]
                    else:
                        self.__puck_color_left_to_right['center'] = colors[1]
                        self.__puck_color_left_to_right['right'] = colors[0]
            
                # No error detected
                self.__puck_color_left_to_right['error'] = False

                self.__node.get_logger().info('Puck colors from left to right are  {} > {} > {}'\
                .format(self.__puck_color_left_to_right["left"],
                        self.__puck_color_left_to_right["center"],
                        self.__puck_color_left_to_right["right"]))
                return True            
            
            
        self.__node.get_logger().info(f'There was a problem detecting pucks. Everything will be dumped on RED')
        return None


        