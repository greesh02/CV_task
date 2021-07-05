# CV_task

Initially after seeing the task i realized that i should be doing transfer learning because the existing model(pretrained version) available in darknet was not trained in such kind of images(as the drone captures top view images from certain heights ).So i searche for similar kind of dataset in github and other sites and came accross a dataset which contains images captured by drones with annotations(bounding box info),i had to do certain changes to those annotations ,i wrote a  script for that(as the provided annotations had unwanted infos like truncation,score,and occulution which was not needed by yolo for training).
# 
dataset link : http://aiskyeye.com/
#
I utilized the vitrana drone model and attitude and position control scripts which i used in eyntra competition ,because my task was more focused on CV part rather than the control system and so on.
#
I utilized the Yolo-V4-tiny architecture because this model is kinda tiny as compared to other Yolo(v4 ,v3 and so). inspite of having less accuracy in prediction as compared to V4 tiny is best suited for drone based application considering the computation (which is propotional to no. of CNN layers or Fully connected layers).

