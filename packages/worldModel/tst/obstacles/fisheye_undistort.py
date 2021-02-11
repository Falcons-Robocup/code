# Copyright 2019-2020 lucas (Falcons)
# SPDX-License-Identifier: Apache-2.0
import sys
import os
import cv2
import numpy as np

class FisheyeUndistort:

    def __init__(self,camera_matrix,distortion_coefficients,image_dimension):
        self.K = camera_matrix
        self.D = distortion_coefficients
        self.dim = image_dimension
        self.last_M = None

        self.shape_max_dist = 0.25
        self.shape_min_area = 50
        self.max_corner_dist = 50

    def undistort(self,image, output_width):
        
        height,width = image.shape[:2]
        assert width/height == self.dim[0]/self.dim[1],'Input image must have same aspect ratio as calibration image'

        image = cv2.resize(image, self.dim)

        output_dim = (output_width, (output_width*self.dim[1])/self.dim[0])

        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(self.K,self.D,self.dim,np.eye(3),balance=1.0,new_size=output_dim,fov_scale=1.25)
        map1,map2 = cv2.fisheye.initUndistortRectifyMap(self.K,self.D,np.eye(3),new_K,output_dim,cv2.CV_16SC2)
        undistorted_img = cv2.remap(image,map1,map2,interpolation=cv2.INTER_LINEAR,borderMode=cv2.BORDER_CONSTANT)
        return undistorted_img

    def get_corner_distance(self, corner1, corner2):
        dist_x = abs(corner1[0] - corner2[0])
        dist_y = abs(corner1[1] - corner2[1])
        dist = max(dist_x, dist_y)
        return dist

    def are_corners_plausible(self, corners):
        reference_corners = self.get_reference_corner_positions()

        max_dist = 0
        for i in range(4):
            max_dist = max(max_dist, self.get_corner_distance(corners[i], reference_corners[i]))

        if(max_dist > self.max_corner_dist):
            return False
        return True

    def debug_show_contours(self, image, contours):
        cv2.drawContours(image, contours, -1, (255,255,0), 3)
        cv2.imshow("Contours", image)
        cv2.waitKey(0)


    def find_corners(self,image):

        gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        ret,thresh = cv2.threshold(gray,127,255,0)
        _, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # opencv 2
        #contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE) # opencv 3

        reference_corner = self.get_reference_corner_contour()

        corner_contours = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            dist = cv2.matchShapes(cnt, reference_corner, cv2.CONTOURS_MATCH_I1, 0)
            if dist < self.shape_max_dist and area > self.shape_min_area:
                corner_contours.append(cnt)
                #print('dist: ' + str(dist) + ' area: ' + str(area))
    
        #self.debug_show_contours(image, corner_contours)

        if len(corner_contours) < 4:
            return []

        height,width = image.shape[:2]
        img_top_left = (0, 0)
        img_top_right = (width, 0)
        img_bottom_right = (width, height)
        img_bottom_left = (0, height)
                
        corner_top_left = self.get_point_on_contours_closest_to_point(corner_contours, img_top_left)
        corner_top_right = self.get_point_on_contours_closest_to_point(corner_contours, img_top_right)
        corner_bottom_right = self.get_point_on_contours_closest_to_point(corner_contours, img_bottom_right)
        corner_bottom_left = self.get_point_on_contours_closest_to_point(corner_contours, img_bottom_left)
        
        corners = np.concatenate([corner_top_left, corner_top_right, corner_bottom_right, corner_bottom_left])
       
        if not self.are_corners_plausible(corners):
            if self.last_M is None:
                print("First corner detection failed.")
                print("Detected corners: " + str(corners))
                self.debug_show_contours(image, corner_contours)

            corners = []
                
      
        return corners


    def get_corners_on_field(self, output_width):
        field_width = 18.0
        field_height = 12.0
        field_width_pixels = output_width        
        field_height_pixels = int(field_width_pixels * field_height / field_width)
        field_border_pixels = 20

        corners_on_field = np.array([[field_border_pixels, field_border_pixels], 
                                     [field_border_pixels+field_width_pixels, field_border_pixels], 
                                     [field_border_pixels+field_width_pixels, field_border_pixels+field_height_pixels], 
                                     [field_border_pixels, field_border_pixels+field_height_pixels]], np.float32)

        return corners_on_field


    def get_field_image(self, image, output_width):
        
        undistorted_img = self.undistort(image, output_width)
        corners_on_image = self.find_corners(undistorted_img)

        if(corners_on_image != []):
            corners_on_image = np.array(corners_on_image, np.float32)
            corners_on_field = self.get_corners_on_field(output_width)

            M = cv2.getPerspectiveTransform(corners_on_image, corners_on_field)
            self.last_M = M
        else:
            M = self.last_M

        field_width = 18.0
        field_height = 12.0
        field_width_pixels = output_width        
        field_height_pixels = int(field_width_pixels * field_height / field_width)
        field_border_pixels = 20
        field_img = cv2.warpPerspective(undistorted_img, M, (field_width_pixels+2*field_border_pixels, field_height_pixels+2*field_border_pixels))
        return field_img
        

    def get_output_dim(self, input_name, output_width):

        in_video = cv2.VideoCapture(input_name)
        
        for i in xrange(0,10):
            try:
                ret, frame = in_video.read()
                out_frame = self.get_field_image(frame, output_width)
                break
            except:
                # If there was a failure set last_M to None so that debug is shown again
                self.last_M = None

        if self.last_M is None:
            print("Corner detection failed, using reference corner values")
            self.last_M = cv2.getPerspectiveTransform(np.array(self.get_reference_corner_positions(), np.float32), self.get_corners_on_field(output_width))
            out_frame = self.get_field_image(frame, output_width)

        in_video.release()

        output_dim = (out_frame.shape[1], out_frame.shape[0])
        return output_dim
        

    def convert_video_file(self, input_name, output_name, output_width):
        
        in_video = cv2.VideoCapture(input_name)

        fps = in_video.get(cv2.CAP_PROP_FPS)
        output_dim = self.get_output_dim(input_name, output_width)
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out_video = cv2.VideoWriter(output_name,fourcc, fps, output_dim)

        # The go pro video has a metadata track that opencv does not decode correctly
        # This causes the video to close after a few frames if processed the regular way
        # The workaround here checks the total number of frames and trusts that it will be correct
        # even if some frames along the way cannot be read
        frame_num = 0
        total_frame_count = in_video.get(cv2.CAP_PROP_FRAME_COUNT)        
        while frame_num < total_frame_count:
            ret, frame = in_video.read()            
            if ret==True:
                print('Frame: '+str(frame_num)+'/'+str(total_frame_count))
                out_frame = self.get_field_image(frame, output_width)
                out_video.write(out_frame)
                frame_num = frame_num + 1
            else:
                print('Lost Frame')


        in_video.release()
        out_video.release()


    def get_point_on_contours_closest_to_point(self, contour_list, point):
        min_dist = float('inf')
        closest_point = None

        for contour in contour_list:
            for p in contour:
                dist = (p[0][0] - point[0])**2 + (p[0][1] - point[1])**2
                if dist < min_dist:
                    min_dist = dist
                    closest_point = p

        return closest_point


    def get_reference_corner_positions(self):
        #corners = [[200, 192], [561, 207], [565, 438], [188, 447]] # 1600 x 1200
        #corners = [[101, 135], [645, 157], [651, 507], [ 84, 521]] # 4000 x 3000
        #corners = [[202, 194], [553, 217], [560, 443], [177, 452]] # 4000 x 3000
        #corners = [[147, 100], [759, 25], [723, 453], [184, 434]] # 4000 x 3000
        #corners = [[ 98, 147],[621, 181],[653, 522],[ 40, 560]]
        corners = [[136, 58],[768, 48],[689, 462],[163, 419]]

        return corners

    def get_reference_corner_contour(self):

        corner_contour = np.array([(81, 9),(89, 9),(90,10),(90,89),(89,90),(10,90),( 9,89),( 9,81),(10,80),
                                   (10,75),(11,74),(11,70),(12,69),(12,67),(13,66),(13,63),(14,62),(14,61),
                                   (15,60),(15,59),(16,58),(16,56),(17,55),(17,54),(18,53),(18,52),(20,50),
                                   (20,49),(21,48),(21,47),(23,45),(23,44),(25,42),(25,41),(30,36),(30,35),
                                   (35,30),(36,30),(41,25),(42,25),(44,23),(45,23),(47,21),(48,21),(49,20),
                                   (50,20),(52,18),(53,18),(54,17),(55,17),(56,16),(58,16),(59,15),(60,15),
                                   (61,14),(62,14),(63,13),(66,13),(67,12),(69,12),(70,11),(74,11),(75,10)],dtype=np.int32)
        return corner_contour



#DIM=(800, 600)
#K=np.array([[351.3650806668937, 0.0, 397.3861408224207], [0.0, 351.5250732453429, 301.81267059417377], [0.0, 0.0, 1.0]])
#D=np.array([[0.05801172825481665], [-0.008253698514056267], [0.023887039529249598], [-0.013006817816418539]])

DIM=(1600, 1200)
K=np.array([[702.8091050937, 0.0, 795.359615117958], [0.0, 703.1106927541736, 603.8700185234372], [0.0, 0.0, 1.0]])
D=np.array([[0.04977590855634825], [0.021950450337551962], [-0.018199999089202057], [0.006763145632435454]])


if __name__ == '__main__':
    input_video_name = sys.argv[1]

    if len(sys.argv) > 2:
        output_video_name = sys.argv[2]
    else:
        video_time = os.path.getmtime(input_video_name)
        output_video_name = os.path.expanduser('~') + '/game-videos/' + str(video_time) + '.avi'
    output_width = 800
    fish_und = FisheyeUndistort(K, D, DIM)    
    fish_und.convert_video_file(input_video_name, output_video_name, output_width)










