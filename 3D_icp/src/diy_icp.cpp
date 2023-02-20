/*
3D registration code using the ICP approach.

By: Saad Alhasan
For: DHBW - Karlsruhe 
*/
#include <cmath>
#include <math.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

using namespace std;
using namespace pcl;
 
using point = PointXYZ;
using normal = PointNormal;
using pointCloud = PointCloud<point>;
using normalCloud = PointCloud<normal>;

// TOOLS
void loadFile (const char* fileName, pointCloud &cloud) 
{
    // mech valid to store the cloud.
    pcl::PolygonMesh mesh;
    // load pcd to mesh and check if the file could not be loaded
    if ( pcl::io::loadPolygonFile ( fileName, mesh ) == -1 ) 
    {
        // printing error message at the L_ERROR verbosity level.
        PCL_ERROR ( "Could not load file." );
        return;
    }
    else 
    {
        // store the cloud data from the mesh in the passed cloud valid.
        pcl::fromPCLPointCloud2<point> ( mesh.cloud, cloud );
        cout << std::string(fileName);
        pcl::console::print ( pcl::console::L_INFO, " was loaded successfully.\n" );
    }
    // remove nan points from cloud and save the resulting PC in the same cloud, and the respective indices in index.
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

void cropClouds (pointCloud::Ptr &source_cloud, pointCloud::Ptr &target_cloud,
                 pointCloud::Ptr &source_overlap, pointCloud::Ptr &target_overlap, string dir)
{
    cout << "cropping clouds..." << endl;
    // calculating extreme points of the clouds after matching the sensor's position
    point tmin;    
    point tmax;
    getMinMax3D( *target_cloud, tmin, tmax );
    point smin;
    point smax;
    getMinMax3D( *source_cloud, smin, smax );

    // clouds cropping (for optimal cropping results, make sure that the overlapping dimensions do not exceed 1/3 the source cloud on each direction)
    if( dir == "y" )
    {
        if( smin.y > tmin.y ) // if source is above target
        {
            // cropping the top relative third of the target cloud
            pcl::PassThrough<point> passer;
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , tmax.y ); 
            passer.filter( *target_overlap );
            // limiting the x-area to be the same as the source's, plus some tolerance area
            passer.setInputCloud( target_overlap );
            passer.setFilterFieldName("x");
            passer.setFilterLimits( ( smin.x ) , ( smax.x ) ); // -10+10
            passer.filter( *target_overlap );
            getMinMax3D( *target_overlap, tmin, tmax );
            // cropping the bottom third of the source cloud
            passer.setInputCloud( source_cloud );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , tmax.y );
            passer.filter( *source_overlap );
        }
        else // if source is below target
        {
            // cropping the bottom relative third of the target cloud
            pcl::PassThrough<point> passer;
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , smax.y );
            passer.filter( *target_overlap );
            // limit the x-area to be the same as the source's, plus some tolerance area
            passer.setInputCloud( target_overlap );
            passer.setFilterFieldName("x");
            passer.setFilterLimits( ( smin.x-50 ) , ( smax.x+50 ) );
            passer.filter( *target_overlap );
            getMinMax3D( *target_overlap, tmin, tmax );
            // crop the top third of the source cloud
            passer.setInputCloud( source_cloud );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , smax.y );
            passer.filter( *source_overlap );
        }
    }
    else if( dir == "x" )
    {
        if( smin.x > tmin.x ) // if source is to the right of target
        {
            // cropping the right relative third of the target cloud
            pcl::PassThrough<point> passer;
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , tmax.x ); 
            passer.filter( *target_overlap );
            // limiting the x-area to be the same as the source's, plus some tolerance area
            passer.setInputCloud( target_overlap );
            passer.setFilterFieldName("y");
            passer.setFilterLimits( ( smin.y ) , ( smax.y ) );
            passer.filter( *target_overlap );
            getMinMax3D( *target_overlap, tmin, tmax );
            // cropping the bottom third of the source cloud
            passer.setInputCloud( source_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , tmax.x );
            passer.filter( *source_overlap );
        }
        else // if source is to the left of target
        {
            // cropping the left relative third of the target cloud
            pcl::PassThrough<point> passer;
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , smax.x );
            passer.filter( *target_overlap );
            // limit the y-area to match the source's, plus some tolerance area
            passer.setInputCloud( target_overlap );
            passer.setFilterFieldName("y");
            passer.setFilterLimits( ( smin.y ) , ( smax.y ) ); 
            passer.filter( *target_overlap );
            getMinMax3D( *target_overlap, tmin, tmax );
            // crop the top third of the source cloud
            passer.setInputCloud( source_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , smax.x );
            passer.filter( *source_overlap );
        }
    }

    else if( dir == "xy" )
    {
            // limit the x-area to match the source's, plus some tolerance area
            pcl::PassThrough<point> passer;
            Eigen::Vector4f sminv;
            Eigen::Vector4f smaxv;
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x, smax.x );
            passer.filter( *target_overlap );
            // limit the y-area to match the source's, plus some tolerance area
            passer.setInputCloud( target_overlap );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , smax.y );
            passer.filter( *target_overlap );
            // crop out the non-overlapping box from source cloud
            if ( 30 > abs(smax.x-tmax.x) ) // source is on the right
            {        
                sminv[0] = smin.x+abs(smax.x-smin.x)/3;
                smaxv[0] = smax.x;
            }
            else // source is on the left
            {        
                sminv[0] = smin.x;
                smaxv[0] = smax.x-abs(smax.x-smin.x)/3;
            }
            if ( 30 > abs(tmax.y-smax.y) ) // source is above
            {
                sminv[1] = smin.y+abs(smax.y-smin.y)/3;
                smaxv[1] = smax.y;
            }
            else // source is under
            {
                sminv[1] = smin.y;
                smaxv[1] = smax.y-abs(smax.y-smin.y)/3;
            }
            sminv[2] = smin.z;
            sminv[3] = 1.0;
            smaxv[2] = smax.z;
            smaxv[3] = 1.0;
            pcl::CropBox<point> cb;
            cb.setInputCloud( source_cloud );
            cb.setNegative( true );
            cb.setMin( sminv );
            cb.setMax( smaxv );
            cb.filter( *source_overlap );

    }
}

void normalSpaceSample (pointCloud::Ptr &cloud,
                        pointCloud::Ptr &ds_cloud,
                        normalCloud::Ptr &normals, float ds_factor = 0.002) 
{
    // create the nss valid, pass the cloud, the normals, and save the filtered cloud.
    pcl::NormalSpaceSampling<point,normal> nss;
    nss.setSample( cloud->size() * ds_factor ); // default: 0.2% of original points
    nss.setBins( 5, 5, 1000 ); // number of different bins on each axis. (each bin gets filled with points of a specific normal vlaue)
    nss.setSeed( 8 );
    nss.setInputCloud( cloud );
    nss.setNormals( normals );
    nss.pcl::Filter<point>::filter( *ds_cloud );
    pcl::console::print ( pcl::console::L_INFO, "Source cloud was downsampled from %d points to %d points\n", (int)cloud.get()->size(), (int)ds_cloud.get()->size() );
}

Eigen::Matrix4f findTF (pointCloud::Ptr &source_cloud, pointCloud::Ptr &target_cloud, pcl::CorrespondencesPtr &corr_list)
{
    Eigen::Matrix4f tf;
    pcl::registration::TransformationEstimationSVD<point, point> te;
    te.estimateRigidTransformation( *source_cloud, *target_cloud, *corr_list, tf );
    cout << "The approximated transfer function is: " << endl << tf << endl;
    return tf;
}

void cloudsViewer (pointCloud::Ptr &source_cloud, 
                   pointCloud::Ptr &target_cloud, 
                   normalCloud::Ptr source_normals, 
                   normalCloud::Ptr target_normals,
                   pcl::CorrespondencesPtr &corr_list,
                   bool view_normals = false) // two clouds with their correspondences and normals
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<point> source_color ( source_cloud, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<point> target_color ( target_cloud, 255, 0, 0 );
    viewer->addPointCloud<point>( source_cloud, source_color, "first" );
    viewer->addPointCloud( target_cloud, target_color, "second", 0 );
    viewer->addCorrespondences<point>( source_cloud, target_cloud, *corr_list ); // views connecting lines betwenn each point pair.
    if (view_normals == true) {
        viewer->addPointCloudNormals<point,normal>( source_cloud, source_normals, 1, 0.9, "source" );
        viewer->addPointCloudNormals<point,normal>( target_cloud, target_normals, 1, 0.9, "target" );
    }
    viewer->spin();
}

void cloudsViewer (pointCloud::Ptr &source_cloud, 
                   pointCloud::Ptr &target_cloud) // Overload, two clouds without correspondences nor normals
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<point> source_color ( source_cloud, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<point> target_color ( target_cloud, 255, 0, 0 );
    viewer->addPointCloud<point>( source_cloud, source_color, "first" );
    viewer->addPointCloud( target_cloud, target_color, "second", 0 );
    viewer->spin();
}

void cloudsViewer (pointCloud::Ptr &cloud) // Overload, one cloud without correspondences nor normals
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<point> source_color ( cloud, 0, 255, 0 );
    viewer->addPointCloud<point>( cloud, source_color, "first" );
    viewer->spin();
}

void saveFile (string file_name, pointCloud::Ptr &cloud)
{
    cout << "Saving cloud..." << endl;
    int counter = 1;
    ofstream out(file_name);
    out << "# .PCD v.7 - Point Cloud Data file\n" << "VERSION .7\n" << "FIELDS x y z\n" << "SIZE 4 4 4\n" << "TYPE F F F\n" << "COUNT 1 1 1\n"
    << "WIDTH 1\n" << "HEIGHT " << cloud->points.size() << "\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << (cloud->points.size()) << "\nDATA ascii\n";
    for(int i = 0; i < cloud->points.size(); i++) {
        out << cloud->points[i].x << " " 
        << cloud->points[i].y << " "
        << cloud->points[i].z << endl;
    }
    out.close();
    cout << "The clouds were merged and saved in " << file_name << " successfully." << endl;
}

string getDirection (pointCloud::Ptr &source_cloud, pointCloud::Ptr &target_cloud) // detects the scanning direction based on the clouds' position information
{
    cout << "Detecting scanning direction..." << endl;
    string dir;
    point tmin, tmax, smin, smax;
    getMinMax3D( *source_cloud, smin, smax );
    getMinMax3D( *target_cloud, tmin, tmax );

    if( (abs(tmin.y - smin.y) <= 30 || abs(tmax.y - smax.y) <= 30) && 
        (abs(tmin.x - smin.x) > 30 && abs(tmax.x - smax.x) > 30) ) 
    dir = "x"; 

    else if( (abs(tmin.x - smin.x) <= 30 || abs(tmax.x - smax.x) <= 30) && 
             (abs(tmin.y - smin.y) > 30 && abs(tmax.y - smax.y) > 30) )  
    dir = "y"; 
    
    else
    dir = "xy";

    //print out the scan direction -optional-
    cout << "The detected scan direction is: " << dir << endl;
    return dir;
}

pointCloud::Ptr combineClouds (pointCloud::Ptr &source_cloud, pointCloud::Ptr &target_cloud, string dir) // uses passer filters to fuse two clouds
{
    pointCloud::Ptr cloud_sum ( new pointCloud );
    cout << "Combining clouds..." << endl;
    point smin; 
    point smax;
    getMinMax3D( *source_cloud, smin, smax);
    point tmin; 
    point tmax;
    getMinMax3D( *target_cloud, tmin, tmax); 
    pcl::PassThrough<point> passer;

    // eliminate double overlap-area by cropping it out of the source cloud
    passer.setInputCloud( source_cloud );

    if( dir == "y" ) {
        //cropping
        pointCloud::Ptr xtarget (new pointCloud);
        passer.setFilterFieldName( "y" );
        if( smin.y > tmin.y ) // source is on top
        {
            passer.setFilterLimits( ( tmax.y-10 ) , ( smax.y ) );
            passer.pcl::Filter<point>::filter( *source_cloud );
            passer.setInputCloud(target_cloud);
            passer.setFilterFieldName("x");
            passer.setFilterLimits (smin.x, smax.x);
            passer.pcl::Filter<point>::filter (*xtarget);
            passer.setInputCloud (xtarget);
            passer.setFilterFieldName ("y");
            passer.setFilterLimits (tmin.y, (tmax.y-10));
            passer.pcl::Filter<point>::filter (*xtarget);
        }
        else if( smin.y < tmin.y ) // source is below
        { 
            passer.setFilterLimits( ( smin.y ) , ( tmin.y+10 ) ); // tminx+10
            passer.pcl::Filter<point>::filter( *source_cloud );
            passer.setInputCloud(target_cloud);
            passer.setFilterFieldName("x");
            passer.setFilterLimits (smin.x, smax.x);
            passer.pcl::Filter<point>::filter (*xtarget);
            passer.setNegative(true);
            passer.pcl::Filter<point>::filter (*target_cloud);
            passer.setInputCloud (xtarget);
            passer.setNegative(false);
            passer.setFilterFieldName ("y");
            passer.setFilterLimits( tmin.y+10 , tmax.y );
            passer.pcl::Filter<point>::filter( *xtarget );
        }

        //saving
       *cloud_sum = *source_cloud + *target_cloud + *xtarget;
    }

    else if( dir == "x" ) {
        //cropping
        pointCloud::Ptr ytarget (new pointCloud);
        passer.setFilterFieldName( "x" );
        if( smin.x > tmin.x ) // source is on the right
        {
            passer.setFilterLimits( ( tmax.x-10 ) , ( smax.x ) );
            passer.pcl::Filter<point>::filter( *source_cloud );
            passer.setInputCloud(target_cloud);
            passer.setFilterFieldName("y");
            passer.setFilterLimits (smin.y, smax.y);
            passer.pcl::Filter<point>::filter (*ytarget);
            passer.setInputCloud (ytarget);
            passer.setFilterFieldName ("x");
            passer.setFilterLimits (tmin.x, (tmax.x-10));
            passer.pcl::Filter<point>::filter (*ytarget);
        }
        else if( smin.x < tmin.x )
        { // source is on the left
            passer.setFilterLimits( ( smin.x ) , ( tmin.x+10 ) ); // tminx+10
            passer.pcl::Filter<point>::filter( *source_cloud );
            passer.setInputCloud(target_cloud);
            passer.setFilterFieldName("y");
            passer.setFilterLimits (smin.y, smax.y);
            passer.pcl::Filter<point>::filter (*ytarget);
            passer.setNegative(true);
            passer.pcl::Filter<point>::filter (*target_cloud);
            passer.setInputCloud (ytarget);
            passer.setNegative(false);
            passer.setFilterFieldName ("x");
            passer.setFilterLimits( tmin.x+10 , tmax.x );
            passer.pcl::Filter<point>::filter( *ytarget );
        }

        //saving
       *cloud_sum = *source_cloud + *target_cloud + *ytarget;
    }

    else if( dir == "xy" ) {
        //cropping on x-direction
        pointCloud::Ptr target_cloud_rest ( new pointCloud ); // point cloud
        if ( abs(smin.x-tmin.x) < abs(smax.x-tmax.x) ) //source is on the left
        {
            //relative source's x-segment
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , (smax.x-10) );
            passer.filter( *source_cloud );
            //non-overlapped target segment
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smax.x-10) , tmax.x );
            passer.filter( *target_cloud_rest );
            //overlapped target's x-segment
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , smax.x-10 );
            passer.filter( *target_cloud );
        }
        else //source is on the right
        {
            //relative source's x-segment
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smin.x+10) , smax.x );
            passer.filter( *source_cloud );
            //non-overlapped target segment
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , (smin.x+10) );
            passer.filter( *target_cloud_rest );
            //overlapped target's x-segment
            passer.setInputCloud( target_cloud );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smin.x+10) , tmax.x );
            passer.filter( *target_cloud );
        }

        //cropping on y-direction
        if ( abs(smin.y-tmin.y) > abs(smax.y-tmax.y) ) //source is above
        {
            //relative source's y-segment
            passer.setInputCloud( source_cloud );   
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smin.y+10) , smax.y);
            passer.filter( *source_cloud );
            //relative target's y-segment
            passer.setInputCloud( target_cloud );           
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , (smin.y+10) );
            passer.filter( *target_cloud );
        }
        else //source is under
        {
            //relative source's y-segment
            passer.setInputCloud( source_cloud );   
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , (smax.y-10) );
            passer.filter( *source_cloud );
            //relative target's y-segment
            passer.setInputCloud( target_cloud );           
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smax.y-10) , tmax.y );
            passer.filter( *target_cloud );
        }

        //saving
        *cloud_sum = *source_cloud + *target_cloud + *target_cloud_rest;
    }
    return cloud_sum;
}

// MATCHERS
void findNormalCorrespondences (pointCloud::Ptr &source_cloud, normalCloud::Ptr &source_normals,
                                pointCloud::Ptr &target_cloud, normalCloud::Ptr &target_normals,
                                pcl::CorrespondencesPtr &corr, int kfactor = 8, float dist = 10)
{
    pcl::registration::CorrespondenceEstimationNormalShooting<point, point, normal> est;
    est.setInputSource( source_cloud );
    est.setSourceNormals( source_normals );
    est.setInputTarget( target_cloud );
    pcl::search::KdTree<point>::Ptr sourceTree ( new pcl::search::KdTree<point> () );
    pcl::search::KdTree<point>::Ptr targetTree ( new pcl::search::KdTree<point> () );
    est.setSearchMethodSource(sourceTree);
    est.setSearchMethodTarget(targetTree);
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, dist );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

void findIndNormalCorrespondences (pointCloud::Ptr &source_cloud, normalCloud::Ptr &source_normals,
                                   pointCloud::Ptr &target_cloud, normalCloud::Ptr &target_normals,
                                   pcl::CorrespondencesPtr &corr,
                                   pcl::IndicesPtr &source_ind, pcl::IndicesPtr &target_ind,
                                   int kfactor = 8, float dist = 10)
{
    pcl::registration::CorrespondenceEstimationNormalShooting<point, point, normal> est;
    est.setInputSource( source_cloud );
    est.setSourceNormals( source_normals );
    est.setIndicesSource ( source_ind );
    est.setIndicesTarget( target_ind );
    est.setInputTarget( target_cloud );
    pcl::search::KdTree<point>::Ptr sourceTree ( new pcl::search::KdTree<point> () );
    pcl::search::KdTree<point>::Ptr targetTree ( new pcl::search::KdTree<point> () );
    est.setSearchMethodSource( sourceTree );
    est.setSearchMethodTarget( targetTree );
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, dist );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

// REJECTORS
void normalCorrRejector (pcl::CorrespondencesPtr &corr_list, pcl::CorrespondencesPtr &corr_out,
                         normalCloud::Ptr &source_normals, normalCloud::Ptr &target_normals,
                         pcl::IndicesPtr rej_ids, float angle_diff)
{
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejector ( new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    rejector->setThreshold( std::cos (pcl::deg2rad(angle_diff)));
    rejector->initializeDataContainer<normal, normal> ( );
    rejector->setInputSource <normal> ( source_normals );
    rejector->setInputTarget <normal> ( target_normals );
    rejector->setInputNormals <normal, normal> ( source_normals );
    rejector->setTargetNormals <normal, normal> ( target_normals );
    rejector->setInputCorrespondences( corr_list );
    rejector->getCorrespondences( *corr_out );
    rejector->getRejectedQueryIndices( *corr_out, *rej_ids );
    pcl::console::print ( pcl::console::L_INFO, "Correspondences are now down to %d after rejecting normal outliers.\n", corr_out->size() );
}

void one2oneRejector (pcl::CorrespondencesPtr &corr_list,
                      pcl::CorrespondencesPtr &corr_out)
{
    pcl::registration::CorrespondenceRejectorOneToOne rejector;
    rejector.setInputCorrespondences(corr_list);
    //rejector.setSourceNormals(source_normals);
    //rejector.setTargetNormals(target_normals);
    rejector.getCorrespondences(*corr_out);
    cout << "Correspondeces are down to " << corr_out->size() << " after removing repeating matches." << endl;
}

// FEATURE EXTRACTORS
void findNormals (pointCloud::Ptr &cloud, normalCloud::Ptr &normals)
{
    // create the ne and kdtree valids, pass the cloud, set search radius, and save the found normals.
    Eigen::Vector4f centroid;
    pcl::NormalEstimation<point, normal> ne;
    pcl::search::KdTree<point>::Ptr kdtree ( new pcl::search::KdTree<point> () );
    compute3DCentroid( *cloud, centroid );
    ne.setViewPoint( centroid[0], centroid[1], 10000.0f );
    ne.setKSearch( 8 );
    ne.setInputCloud( cloud );
    ne.setSearchMethod( kdtree );
    ne.compute( *normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, %d normals were found.\n", (int)normals->size() );
}

pcl::IndicesPtr filterByAngle (normalCloud::Ptr &cloud_normals, const Eigen::Vector3f refVector, float threshAngle, string LH)
{
    pcl::IndicesPtr indices ( new std::vector <int> );
    float angle, lenSq1, dot;

    for( int i = 0; i < cloud_normals->size(); ++i )
    {
        dot = cloud_normals->points[i].normal_z;
        lenSq1 = sqrt(pow(cloud_normals->points[i].normal_x, 2.0f) + pow(cloud_normals->points[i].normal_y, 2.0f) + pow(cloud_normals->points[i].normal_z, 2.0f));
        angle = pcl::rad2deg( acos(dot/lenSq1) );
        if( LH == "higher") 
        { 
            if ( angle >= threshAngle ) 
                indices->push_back(i);
        }

        else if( LH == "lower") 
        { 
            if ( angle <= threshAngle ) 
                indices->push_back(i);
        }            
    }

    cout << endl << indices->size() << " from " << cloud_normals->size() << " point normals are " << LH << " than " << threshAngle << "°." << endl;
    return indices;
}  

main (int argc, char **argv) 
{   
    pcl::console::TicToc time;
    time.tic();
    // Variable to store the estimated TF.
    Eigen::Matrix4f tf;
    tf << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1
    ;
    // Straight vector pointing in the z-direction used to compare with points normal angles.
    Eigen::Vector3f straight_vec;
    straight_vec(0) = 0;
    straight_vec(1) = 0;
    straight_vec(2) = 10;

    // Create pointer valids for source and target clouds, with all other variable dependencies.
    pointCloud::Ptr source_cloud (new pointCloud); // point cloud
    pointCloud::Ptr source_overlap (new pointCloud); // overlapped region
    normalCloud::Ptr source_normals (new normalCloud); // cloud normals
    pointCloud::Ptr nss_source_overlap (new pointCloud); // sampled cloud
    pcl::IndicesPtr source_ind (new std::vector <int>); // point indices of curvy points

    pointCloud::Ptr target_cloud (new pointCloud);
    pointCloud::Ptr target_overlap (new pointCloud);
    normalCloud::Ptr target_normals (new normalCloud);
    pointCloud::Ptr nss_target_overlap (new pointCloud);
    pcl::IndicesPtr target_ind (new std::vector <int>);

    pointCloud::Ptr cloud_sum (new pointCloud); // registered cloud

    pcl::CorrespondencesPtr corr_list (new pcl::Correspondences); // correspondences list
    pcl::CorrespondencesPtr corr_list_out (new pcl::Correspondences); // parallel correspondences list
    pcl::IndicesPtr rej_ids (new vector <int>); // rejected indices list
    vector<pointCloud::Ptr, Eigen::aligned_allocator<pointCloud::Ptr>> clouds_vec; // point clouds vector
    vector<vector<pair<string, float>>> scans_pos_vec; // clouds positions vector
    string dir; // scanning direction

    // Loading source and target files:
    cout << "Loading pcd files...\n";
    for (int i=1; i <= argc-2; i++)
    {
        pointCloud::Ptr cloud (new pointCloud);
        loadFile (argv[i], *cloud);
        clouds_vec.push_back (cloud);
    }
    cout << clouds_vec.size() << " clouds were loaded for registration." << endl;

    // Assigning the initial target cloud
    target_cloud = clouds_vec[0];

    // Registring two clouds at a time, iterating through the clouds_vec
    for (int i=1; i < clouds_vec.size(); i++) {
        source_cloud->clear();
        // Note: each scan has 2 positions vector, initial and final, 
        // thus len(scans_pos_vec) = 2*len(clouds_vec); explaining the i+=2.
        // Defining first key values:
        source_cloud = clouds_vec[i];

        dir = getDirection (source_cloud, target_cloud);

        // View the original input clouds -optional-
        cout << "Viewing the pre-aligned clouds. Press 'q' to continue." << endl;
        cloudsViewer (source_cloud, target_cloud);

        // Crop the overlapped areas of the point clouds
        cropClouds (source_cloud, target_cloud, source_overlap, target_overlap, dir);

        // View the resultant overlap clouds -optional-
        cout << "Viewing the overlap regions of the clouds. Press 'q' to continue." << endl;
        cloudsViewer (source_overlap, target_overlap);

        // Downsampling the original clouds:
        findNormals (source_overlap, source_normals);
        findNormals (target_overlap, target_normals);
        normalSpaceSample (source_overlap, nss_source_overlap, source_normals, 0.2); // The downsampling factor values depend on how much details are in 
        normalSpaceSample (target_overlap, nss_target_overlap, target_normals, 0.2); // the scans. The more details (grooves), the lower the ds_factor can be.
        
        // Calculating new normals of the downsampled clouds
        findNormals (nss_source_overlap, source_normals);
        findNormals (nss_target_overlap, target_normals);
        copyPointCloud (*nss_source_overlap, *source_normals);
        copyPointCloud (*nss_target_overlap, *target_normals);

        // Estimating the correspondeces using normal shooting method:
        findNormalCorrespondences (nss_source_overlap, source_normals, nss_target_overlap, target_normals, corr_list, 8, 200);
        normalCorrRejector (corr_list, corr_list_out, source_normals, target_normals, rej_ids, 35.0f); // normal-angle rejector is used to assure that only almost-vertical correspondences are considered. 
        one2oneRejector (corr_list_out, corr_list_out);
        
        // View downsampled overlap clouds before the initial transformation -optional-
        cout << "Viewing the clouds before initial ICP. Press 'q' to continue." << endl;
        cloudsViewer( nss_source_overlap, nss_target_overlap, source_normals, target_normals, corr_list_out );

        // estimate and apply the matrix on the source clouds
        tf = findTF (nss_source_overlap, nss_target_overlap, corr_list_out);
        transformPointCloud (*nss_source_overlap, *nss_source_overlap, tf);
        transformPointCloud (*source_cloud, *source_cloud, tf);

        // View downsampled overlap clouds after the initial transformation -optional-
        cout << "Viewing the clouds after initial ICP. Press 'q' to continue." << endl;
        cloudsViewer (nss_source_overlap, nss_target_overlap);

        // Filtering out points on straight areas
        source_ind = filterByAngle (source_normals, straight_vec, 45, "higher");
        target_ind = filterByAngle (target_normals, straight_vec, 45, "higher");
        
        // Iterate ICP until a convergence criteria is satisfied (aligning details)
        bool conv = false;
        int loop = 1;
        while (!conv)
        {
            // Estimate correspondences and then apply rejectors
            cout << "\nloop #" << loop << endl;
            findIndNormalCorrespondences (nss_source_overlap, source_normals, nss_target_overlap, target_normals, corr_list_out, source_ind, target_ind, 8, 10); 
            normalCorrRejector (corr_list_out, corr_list, source_normals, target_normals, rej_ids, 35.0f);
            one2oneRejector (corr_list, corr_list);

            if (loop == 1)
                cloudsViewer(nss_source_overlap, nss_target_overlap, source_normals, target_normals, corr_list);
            // View clouds iteratively before each transform -optional-
            //cloudsViewer( nss_source_overlap, nss_target_overlap, source_normals, target_normals, corr_list );
            
            // Estimate TF and transform clouds
            tf = findTF (nss_source_overlap, nss_target_overlap, corr_list);
            transformPointCloud (*nss_source_overlap, *nss_source_overlap, tf);
            transformPointCloud (*source_cloud, *source_cloud, tf);

            // Checking the convergence criteria, by comparing the last two TFs.
            pcl::registration::DefaultConvergenceCriteria<float> converge (i, tf, *corr_list);
            converge.setTranslationThreshold (0.000005); 
            converge.setRotationThreshold (cos( deg2rad(0.5f)));
            if ( converge.hasConverged() == true )
                break;
            loop++;
        } 
        
        // View the detail-alignment result -optional-
        cout << "Viewing the clouds after details-alignment loop. Press 'q' to continue." << endl;
        cloudsViewer( nss_source_overlap, nss_target_overlap );
        
        // Iterating ICP until a convergence criteria is satisfied (aligning flat surfaces):
        conv = false;
        loop = 1;
        while (!conv)
        {
            // estimate correspondences and then apply rejectors
            cout << "\nloop #" << loop << endl;
            findNormalCorrespondences (nss_source_overlap, source_normals, nss_target_overlap, target_normals, corr_list_out, 8, 2);
            normalCorrRejector (corr_list_out, corr_list, source_normals, target_normals, rej_ids, 35.0f);
            one2oneRejector (corr_list, corr_list);

            // View clouds iteratively before each transform -optional-
            //cloudsViewer( nss_source_overlap, nss_target_overlap, source_normals, target_normals, corr_list );

            // estimate TF and transform clouds
            tf = findTF( nss_source_overlap, nss_target_overlap, corr_list );
            transformPointCloud( *nss_source_overlap, *nss_source_overlap, tf );
            transformPointCloud( *source_cloud, *source_cloud, tf );

            // Checking the convergence criteria, by comparing the last two TFs.
            pcl::registration::DefaultConvergenceCriteria<float> converge (i, tf, *corr_list);
            converge.setTranslationThreshold (0.000005);
            converge.setRotationThreshold (cos( deg2rad( 0.5f )));
            if (converge.hasConverged() == true)
                break;
            loop++;
        }

        // View the registration result -optional-
        cout << "Viewing the clouds after complete registration. Press 'q' to continue." << endl;
        cloudsViewer (nss_source_overlap, nss_target_overlap);
        cloudsViewer (source_cloud, target_cloud);

        // Merging the clouds
        cloud_sum = combineClouds (source_cloud, target_cloud, dir);
        // View the new merged cloud -optional-
        cout << "Viewing the new merged cloud. Press 'q' to continue." << endl;
        cloudsViewer (cloud_sum);
        // Passing on the new target cloud data for the next loop
        target_cloud->clear();
        copyPointCloud (*cloud_sum, *target_cloud);
        cloud_sum->clear();
        
    }

    // Save the final cloud if a file name is inputted as an argument
    string file_name = argv[argc-1];
    saveFile (file_name, target_cloud);

    cout << "The registration finished in " << time.toc() / 1000 << " s." << endl;

    // View the entire final cloud
    cout << "Viewing the final cloud. Press 'q' to end." << endl;
    cloudsViewer (target_cloud);
    return 0;
}
