#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>


void visualize_normals(double** normals,  boost::shared_ptr<pcl::visualization::PCLVisualizer> view, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud, int idMax, int pas)
{

    //on show les normales calculées
    typedef pcl::PointXYZ PointT;
    for( int i = 0 ; i < idMax ; i=i+ pas )
    {
        //viewer.addCoordinateSystem (1.0, "axis", 0);
        double coefVecteur = 20.0;
        PointT p1, p2;
        p1.x = pCloud->points[i].x;
        p1.y = pCloud->points[i].y;
        p1.z = pCloud->points[i].z;
        p2.x = p1.x + coefVecteur * normals[i][0];
        p2.y = p1.y + coefVecteur * normals[i][1];
        p2.z = p1.z + coefVecteur * normals[i][2];
        bool add = true;
        //viewer->spinOnce (100);

	stringstream ss;
	ss << i;
	string str = ss.str();
       
        std::string name;
	name = "a"+str;
           
        view->addArrow ( p1 , p2 , 255.0 , 0.0 , 0.0 ,  name);
    }
}
