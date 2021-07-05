# CV_task
# Dataset
Initially after seeing the task i realized that i should be doing transfer learning because the existing model(pretrained version) available in darknet was not trained in such kind of images(as the drone captures top view images from certain heights ).So i searche for similar kind of dataset in github and other sites and came accross a dataset which contains images captured by drones with annotations(bounding box info),i had to do certain changes to those annotations ,i wrote a  script for that(as the provided annotations had unwanted infos like truncation,score,and occulution which was not needed by yolo for training).

dataset link : http://aiskyeye.com/
# Drone
I utilized the vitrana drone model and attitude and position control scripts which i used in eyntra competition ,because my task was more focused on CV part rather than the control system and so on.
# CV
For CV part I utilized the Yolo-V4-tiny architecture because this model is kinda tiny as compared to other Yolo(v4 ,v3 and so). inspite of having less accuracy in prediction as compared to V4 tiny is best suited for drone based application considering the computation (which is propotional to no. of CNN layers or Fully connected layers).So I did transfer learining by loading the pretrained weights for the same model and started training with the custom dataset and annotations.

I was not able to utilize the zoom option ,so the drone had to move at altitude of 35 meters for prediction with downward facing camera (initially vishal sir suggested to fly the drone at 70 meters and utilize zoom option for 35 meters which i was not able to figure out).
Also I  was not able generate better results for predicting humans. 


# RUN
Follow the steps to run scripts. 

# World
: roslaunch vitarana_drone drone_land_new.launch
# for the drone 
 • rosrun vitarana_drone position_controller.py
#
 • rosrun vitarana_drone attitude_controller.py
#
 • rosrun vitarana_drone camera_im.py      -- for capturing the image and storing it(i was facing some issues with opencv which im still figuring out so i stored the image captured and save it at a location and then read it through another python script )
Also the change the location to where you want to save the captured image.

# location of car and human clusters 
i have attached the latitude ,longitude and height(which is 35) coordinates for the location of clusters of humans and vehicles in a text file inside vitarana_drone/scripts folder
So to change lat long and alt of drone(to make the drone move over there).we should publish the corresponding lat, long and alt in custom rostopic /set_point_pub
#
• rostopic pub /set_point_pub vitarana_drone/SetPoints "latitude: 0.0          
longitude: 0.0
altitude: 0.0" 
#

# YOLO object detection
the code for object detection(not a ros node) is present inside vitarana_drone/Yolo folder. 





