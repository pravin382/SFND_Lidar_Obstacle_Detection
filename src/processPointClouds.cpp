// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "render/render.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered1 (new pcl::PointCloud<PointT>),cloud_filtered2 (new pcl::PointCloud<PointT>);

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // voxel grid point reduction
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered1);
    std::cout << "After voxel grid filtering, number of data point =" << (*cloud_filtered1).size() <<std::endl;

    // crop_box filter
    pcl::CropBox<PointT> cbf;
    cbf.setMin(minPoint);
    cbf.setMax(maxPoint);
    cbf.setInputCloud (cloud_filtered1);
    cbf.filter(*cloud_filtered2);
    std::cout << "After Crop box filtering, number of data point =" << (*cloud_filtered2).size() <<std::endl;
    
    // to remove the roof region
    std::vector<int> indices;
    pcl::CropBox<PointT> roof;
    cbf.setMin(Eigen::Vector4f(-1.5,-1.7, -1, 1));
    cbf.setMax(Eigen::Vector4f(2.6,1.7, -.4, 1));
    cbf.setInputCloud (cloud_filtered2);
    cbf.filter(indices);
    
    // inlier
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for (int i : indices) {
        inliers->indices.push_back(i);
    }

    // Extract
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered2);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_filtered2);
    // end TODO
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered2;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // for extracted point cloud
    typename pcl::PointCloud<PointT>::Ptr roadcloud (new pcl::PointCloud<PointT> ()),obstaclecloud (new pcl::PointCloud<PointT> ());
    // Create the filtering object
     pcl::ExtractIndices<PointT> extract;
    // Extract the inliers (road)
    /*
    // we can also do like this
    for (int i : inliers->indices)
    {
        roadcloud->points.push_back(cloud->point[indices])
    }
    */
     extract.setInputCloud (cloud);
     extract.setIndices(inliers);
     extract.setNegative (false);
     extract.filter (*roadcloud);
     // extract obstacle
     extract.setInputCloud (cloud);
     extract.setIndices(inliers);
     extract.setNegative (true);
     extract.filter (*obstaclecloud);
    // end TODO
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstaclecloud, roadcloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    bool useDefaultPCL = false; // change to true if you want to use default PCL segmentation functions

    if (useDefaultPCL) {

        // TODO:: Fill in this function to find inliers for the cloud.
        // Create the segmentation object
        pcl::SACSegmentation<PointT> seg;

        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (maxIterations);
        seg.setDistanceThreshold (distanceThreshold);
        
        
        //
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        }
    } else {
        std::unordered_set<int> inliersResult;
        srand(time(NULL));
        
        for(int i=0; i< maxIterations; i++) {
            // Randomly sample subset and fit line
            std::unordered_set<int> inliers_indices; 
            while(inliers_indices.size()<3) {
                inliers_indices.insert(rand()%cloud->points.size());
            }
            auto itr = inliers_indices.begin();
            PointT point1 = cloud->points[*itr];
            itr ++;
            PointT point2 = cloud->points[*itr];
            itr++;
            PointT point3 = cloud->points[*itr];
            
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
                double dist = fabs(A* cloud->points[j].x + B * cloud->points[j].y + C * cloud->points[j].z + D)/ sqrt(A*A + B*B + C*C);
                //std::cout << "dist = " <<dist <<std::endl;
                // If distance is smaller than threshold count it as inlier
                if (dist<= distanceThreshold) {
                    inliers_indices.insert(j);
                }
            }
            if (inliers_indices.size()>= inliersResult.size()) {
                inliersResult = inliers_indices;
            }
        }
        inliers->indices.insert(inliers->indices.end(), inliersResult.begin(), inliersResult.end()); // copy unordered set to vector
    }
    // end TODO
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void proximity(int indices, const PointT points,std::vector<bool>& processed, std::vector<int>& new_cluster, KdTree* tree, float distanceTol) {
    processed[indices] = true;
    new_cluster.push_back(indices);
    std::vector<float> search_point = {points[indices].x, points[indices].y, points[indices].z};
    std::vector<int> nearby = tree->search(search_point,distanceTol);
    
    for (int ith_val : nearby) {
        if (!processed[ith_val])
            proximity(ith_val,points,processed,new_cluster,tree,distanceTol);
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    bool useDefaultPCL = false; // change to true if you want to use default PCL segmentation functions

    if (useDefaultPCL) {
        // Creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
        tree->setInputCloud (cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance (clusterTolerance); // 2cm
        ec.setMinClusterSize (minSize);
        ec.setMaxClusterSize (maxSize);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud);
        ec.extract (cluster_indices);

        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it ) {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
                cloud_cluster->points.push_back (cloud->points[*pit]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
    } else {
        KdTree* tree = new KdTree;
        for (int i=0; i<cloud->points.size (); i++) {
            // need to convert to vector from pcl::PointXYZ based on our KDtree implementation
            std::vector<float> point {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
            tree->insert(point,i);
        }
    	    
        std::vector<std::vector<int>> clusters_indices;
        std::vector<bool> processed(cloud->points.size(), false);
        for(int i = 0; i < processed.size(); i++) {
            if (!processed[i]){
                std::vector<int> new_cluster;
                proximity(i, cloud->points, processed, new_cluster, tree, clusterTolerance);
                if ((new_cluster.size() >= minSize) && (new_cluster.size() <= maxSize))
                    clusters_indices.push_back(new_cluster);
            }
            
        }
        for(std::vector<std::vector<int>>::const_iterator it = clusters_indices.begin(); it != clusters_indices.end(); ++it ) {
            typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
            for ( int x : *it)
                cloud_cluster->points.push_back (cloud->points[x]);
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            clusters.push_back(cloud_cluster);
        }
        /*std::cout << clusters_indices.size() << std::endl;
        for (std::vector<int> cluster : clusters_indices)
              std::cout << cluster.size() << std::endl; */
    }
    // end TODO
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}