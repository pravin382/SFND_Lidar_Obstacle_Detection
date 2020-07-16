/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <math.h>
#include<iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; 
	srand(time(NULL));
	pcl::PointXYZ point1, point2;
	// TODO: Fill in this function
        int num = cloud->points.size();
        //std::cout << num << std::endl;
        float dist = 0.0;
	// For max iterations 
        for(int i=0; i< maxIterations; i++) {
	// Randomly sample subset and fit line
            int index1 = rand()%num;
            int index2;
            do {index2 = rand()%num;} while (index2 == index1);
            point1 = cloud->points[index1];
            point2 = cloud->points[index2];
            // line Ax + By + C = 0  
            double A = point1.y - point2.y;
            double B = point2.x - point1.x;
            double C = point1.x * point2.y - point2.x * point1.y;
            // Measure distance between every point and fitted line
            std::unordered_set<int> inliers_indices;
            for (int j = 0; j < cloud->points.size(); j++) {
                dist = fabs(A* cloud->points[j].x + B * cloud->points[j].y + C)/ sqrt(A*A + B*B);
                //std::cout << "dist = " <<dist <<std::endl;
                if (std::isnan(dist)){
                    std::cout << "A = " << A << " B = " << B << " C = "<< C <<std::endl;
                    break;
                }
                // If distance is smaller than threshold count it as inlier
                if (dist<= distanceTol) {
                    inliers_indices.insert(j);
                }
            }
            if (inliers_indices.size()>= inliersResult.size()){
                inliersResult = inliers_indices;
            }
	
        }
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult; 
	srand(time(NULL));
	pcl::PointXYZ point1, point2, point3;
	// TODO: Fill in this function
        float dist = 0.0;
        //std::cout <<cloud->points.size() <<std::endl;
	// For max iterations 
        for(int i=0; i< maxIterations; i++) {
            // Randomly sample subset and fit line
            std::unordered_set<int> inliers_indices; 
            while(inliers_indices.size()<3) {
                inliers_indices.insert(rand()%cloud->points.size());
            }
            auto itr = inliers_indices.begin();
            point1 = cloud->points[*itr];
            itr ++;
            point2 = cloud->points[*itr];
            itr++;
            point3 = cloud->points[*itr];
            
            // plane Ax + By + Cz +D = 0
            Vect3 v1(point2.x - point1.x,point2.y - point1.y, point2.z - point1.z), v2(point3.x - point1.x, point3.y - point1.y, point3.z - point1.z);
            double A = v1.y * v2.z - v1.z * v2.y;
            double B = v1.z * v2.x - v1.x * v2.z;
            double C = v1.x * v2.y - v1.y * v2.x;
            double D = -(A * point1.x + B * point1.y + C * point1.z);
            //std::cout << A << " " << B  << " " << C << " " << D << std::endl;
            for (int j = 0; j < cloud->points.size(); j++) {
                // if indices is already in inliers_indices, go to next itr
                if (inliers_indices.count(j)>0)
                    continue;
                dist = fabs(A* cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D)/ sqrt(A*A + B*B + C*C);
                //std::cout << "dist = " <<dist <<std::endl;
                // If distance is smaller than threshold count it as inlier
                if (dist<= distanceTol) {
                    inliers_indices.insert(j);
                }
            }
            if (inliers_indices.size()>= inliersResult.size()){
                inliersResult = inliers_indices;
            }
            
	
        }
	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);
        std::unordered_set<int> inliers = RansacPlane(cloud, 100, 0.2);
        
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
