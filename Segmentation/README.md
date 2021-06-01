# 3d_Point_cloud_processing
The point cloud processing algorithm used during the master project


1.Point Cloud Segmentation
Examples come from the robot unordered grab, mainly including point cloud drop sampling, plane filtering, point cloud segmentation, the extraction of the top workpiece and so on

a.unsample+remove plane

![image](https://user-images.githubusercontent.com/50473284/120151822-4f96f500-c21f-11eb-8913-f00a9dbb3327.png)![image](https://user-images.githubusercontent.com/50473284/120151861-5aea2080-c21f-11eb-8310-a29aab271c31.png)


b.segment

（Euclidean clustering segmentation）（Region growing）

![image](https://user-images.githubusercontent.com/50473284/120151775-43129c80-c21f-11eb-9394-ec95dfd64a41.png)![image](https://user-images.githubusercontent.com/50473284/120151894-68070f80-c21f-11eb-9e87-97c7820f5a9f.png)

c.Extract the topmost point cloud

![image](https://user-images.githubusercontent.com/50473284/120151993-8705a180-c21f-11eb-8883-171ad8bc1595.png)
![image](https://user-images.githubusercontent.com/50473284/120152005-8967fb80-c21f-11eb-8722-db21002434a4.png)


d.reprojection and Calculate the position

![image](https://user-images.githubusercontent.com/50473284/120152017-8bca5580-c21f-11eb-9f96-6a90ed41a531.png)![image](https://user-images.githubusercontent.com/50473284/120152025-8d941900-c21f-11eb-8147-80a0d0c9a388.png)
![image](https://user-images.githubusercontent.com/50473284/120152031-8ff67300-c21f-11eb-9c2a-eeb6266dd5bd.png)
