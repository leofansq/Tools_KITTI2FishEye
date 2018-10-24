# Conversion Tool for FishEye Dataset

## About Imaging Algorithm ##
* The algorithm is based on the equidistance projection ![equidistance projection](/pic/algorithm.png)
* The picture below shows the principle of the imaging algorithm.Every point K in the sphere correspond to a single point Q(x,y) in the raw image. Then K is remapped into imaging plane as P(u,v). The equation between P and Q is ![imaging algorithm](/pic/imaging_algorithm.png)

![imaging algorithm](/pic/algorithm_pic.png)

* The process of image manufacture is shown below. The flat image is transformed to the sphere, then remapped into fisheye imaging plane.

![process](/pic/process.png)

## Implementation (Windows) ##

### For Batch Processing ###
* Get The List of Filename：Put the [get_list.bat](/get_list.bat) into the file and click. The list of filename is written into the .txt file.*Don't forget to delete the filename 'get_list.bat' from the list by yourself!*
* Adjust the parameters and Run the programe in Batch-Processing Mode

### For Single Picture ###
* Adjust the parameters and Run the programe in Single-Picture Mode

## About Generated Dataset ##
* The generated dataset contains images and labels
* The parameters of the images is based on the parameters you set
* The contents of the labels are based on the origin labels and the parameters you set. Normally, it includes two parts, classification and the b-box.