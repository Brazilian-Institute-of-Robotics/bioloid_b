# Image Converter Class

# How to Use:

1. Include the class files:
    - Include on Cmakelist
    - Include the header on cpp/hpp/h file

2. Create a instance of ImageConverter class
    `bir::ImageConverter* ImgConv;`
    `ImagConv = new bir::ImageConverter(encoding)`
    or
    `bir::ImageConverter ImgConv(encoding);`

3. Converting ros image to openCV image (sensor_mgs::ImageConstPtr& to cv::Mat)
    `void imageCb(const sensor_msgs::ImageConstPtr& msg) { `
    `   cv::Mat image = *(ImgConv)*(msg); // (First Instance option) `
    `   cv::Mat image = (ImgConv)*(msg);  // (Second instance option)`
    `   ... `
    `} `

4. Converting openCV image to ros image
    `cv::Mat image ... // imagem is a cv::Mat type`
    `sensor_msgs::ImagePtr ros_img_msg = *(ImgConv)*image; // (First Instance option)`
    `sensor_msgs::ImagePtr img_msg = (ImgConv)*image; // (Second Instance option)`

5. Tips
    - First Instance is usefull when is create as a class's member.
    

> Etevaldo A. Cardoso Neto ( Teo Cardoso )