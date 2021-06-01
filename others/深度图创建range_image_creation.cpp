#include <pcl/range_image/range_image.h>
int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ> pointCloud;
 //生成数据
  for (float y=-0.5f; y<=0.5f; y+=0.01f) {
    for (float z=-0.5f; z<=0.5f; z+=0.01f) {
      pcl::PointXYZ point;
      point.x = 2.0f - y;
      point.y = y;
      point.z = z;
      pointCloud.points.push_back(point);
    }
  }
  pointCloud.width = (uint32_t) pointCloud.points.size();
  pointCloud.height = 1;
//以1度为角分辨率，从上面创建的点云创建深度图像。
  float angularResolution = (float) (  1.0f * (M_PI/180.0f));  //相邻像素点所对应的每个光束相差1°
// 1度转弧度
  float maxAngleWidth     = (float) (360.0f * (M_PI/180.0f));  //距离传感器对周围环境拥有360°视角
// 360.0度转弧度
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f)); //竖直方向
// 180.0度转弧度
  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);//6位置自由度 roll，pitch，yaw=0；
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;//X轴向右，Y轴向下，Z轴向前
  float noiseLevel=0.00;//深度距离值是通过查询点半径为5cm的圆内所包含的点计算平均距离
  float minRange = 0.0f;//min_range设置最小的获取距离，小于最小获取距离的位置为传感器的盲区
  int borderSize = 1;//border_size获得深度图像的边缘的宽度 默认为1
  pcl::RangeImage rangeImage;
  rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight, sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
std::cout << rangeImage << "\n";
}
