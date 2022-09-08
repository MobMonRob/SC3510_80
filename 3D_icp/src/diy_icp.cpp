#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/console/time.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

using namespace std;
using namespace pcl;

struct Positions {
    float xc = 0.0; // x_center 
    float x0 = 0.0;
    float x1 = 0.0;
    float y0 = 0.0;
    float y1 = 0.0;
};
void cloudsViewer(PointCloud<PointXYZ>::Ptr &cloud1, PointCloud<PointXYZ>::Ptr &cloud2);

void loadFile( const char* fileName, pcl::PointCloud<pcl::PointXYZ> &cloud ) 
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
        pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
        cout << std::string(fileName);
        pcl::console::print ( pcl::console::L_INFO, " was loaded successfully.\n" );
    }
    // remove nan points from cloud and save the resulting PC in the same cloud, and the respective indices in index.
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

void initAlign( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target,
                pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target, string dir, Positions* &target, Positions* &source )
{
    cout << "Pre-aligning clouds..." << endl;

    // constructing the new corresponding pose transfer function
    Eigen::Matrix4f tf;

    float x_dist = source->xc - target->xc;
    float y_dist = source->y0 - target->y0;

    tf << 1, 0, 0, x_dist,
          0, 1, 0, y_dist,
          0, 0, 1, 0,
          0, 0, 0, 1
    ;
    cout << "the cloud transformation function is: " << endl << tf << endl;
    transformPointCloud( *cloud_source, *cloud_source, tf );
    // view the clouds after position alignment -optional-
    //cloudsViewer( cloud_source, cloud_target );

    // calculating extreme points of the clouds after matching the sensor's position
    pcl::PointXYZ tmin;
    pcl::PointXYZ tmax;
    getMinMax3D( *cloud_target, tmin, tmax );
    pcl::PointXYZ smin;
    pcl::PointXYZ smax;
    getMinMax3D( *cloud_source, smin, smax );

    cout << "source (min), (max): " << smin << ", " << smax << endl;
    cout << "target (min), (max): " << tmin << ", " << tmax << endl;
    
    // clouds cropping
    if( dir == "y" )
    {
        if( source->y0 > target->y0 )
        {
            // cropping the top relative third of the target cloud
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmax.y-(abs(smax.y-smin.y)/3) , (tmax.y) ); 
            passer.filter( *overlap_target );
            // limiting the x-area to be the same as the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName("x");
            passer.setFilterLimits( ( smin.x ) , ( smax.x ) );
            passer.filter( *overlap_target );
            getMinMax3D( *overlap_target, tmin, tmax );
            // cropping the bottom third of the source cloud
            passer.setInputCloud( cloud_source );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , tmax.y );
            passer.filter( *overlap_source );
        }
        else
        {
            // cropping the bottom relative third of the target cloud
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (tmin.y) , tmin.y+(abs(smax.y-smin.y)/3) );
            passer.filter( *overlap_target );
            // limit the x-area to be the same as the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName("x");
            passer.setFilterLimits( ( smin.x ) , ( smax.x ) );
            passer.filter( *overlap_target );
            getMinMax3D( *overlap_target, tmin, tmax );
            // crop the top third of the source cloud
            passer.setInputCloud( cloud_source );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , tmax.y );
            passer.filter( *overlap_source );
        }
    }
    else if( dir == "x" )
    {
        if( source->xc > target->xc )
        {
            // cropping the right relative third of the target cloud
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (tmax.x)-abs(smax.x-smin.x)/3 , (tmax.x) ); 
            passer.filter( *overlap_target );
            // limiting the x-area to be the same as the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName("y");
            passer.setFilterLimits( ( smin.y ) , ( smax.y ) );
            passer.filter( *overlap_target );
            getMinMax3D( *overlap_target, tmin, tmax );
            // cropping the bottom third of the source cloud
            passer.setInputCloud( cloud_source );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , tmax.x );
            passer.filter( *overlap_source );
        }
        else
        {
            // cropping the left relative third of the target cloud
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (tmin.x) , (tmin.x)+abs(smax.x-smin.x)/3 );
            passer.filter( *overlap_target );
            // limit the y-area to match the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName("y");
            passer.setFilterLimits( ( smin.y ) , ( smax.y ) );
            passer.filter( *overlap_target );
            getMinMax3D( *overlap_target, tmin, tmax );
            // crop the top third of the source cloud
            passer.setInputCloud( cloud_source );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , tmax.x );
            passer.filter( *overlap_source );
        }
    }
    else if( dir == "yx")
    {
            // crop the relative segment of the target cloud
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , smax.y );
            passer.filter( *overlap_target );
            // limit the x-area to match the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , smax.x );  
            passer.filter( *overlap_target );
            // maintain the source cloud
            copyPointCloud( *cloud_source, *overlap_source );
    }
    else if( dir == "xy")
    {
            // limit the x-area to match the source's
            pcl::PassThrough<PointXYZ> passer;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x, smax.x );
            passer.filter( *overlap_target );
            // limit the y-area to match the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , smax.y );  
            passer.filter( *overlap_target );
            // maintain the source cloud
            copyPointCloud( *cloud_source, *overlap_source );

    }
}

void normalSpaceSample( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &nss_overlap_source,
                        pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals ) 
{
    // create the nss valid, pass the cloud, the normals, and save the filtered cloud.
    pcl::NormalSpaceSampling<pcl::PointXYZ,pcl::PointNormal> nss;
    nss.setSample( cloud_source->size() / 12 );
    nss.setBins( 30, 30, 1000 ); // number of different bins on each axis. (each bin gets filled with points of a specific normal vlaue)
    nss.setSeed( 8 );
    nss.setInputCloud( cloud_source );
    nss.setNormals( source_normals );
    nss.pcl::Filter<pcl::PointXYZ>::filter( *nss_overlap_source );
    pcl::console::print ( pcl::console::L_INFO, "Source cloud was downsampled from %d points to %d points\n", (int)cloud_source.get()->size(), (int)nss_overlap_source.get()->size() );
}

// MATCHERS
void findNormalCorrespondences( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, pcl::PointCloud<pcl::PointNormal>::Ptr &target_normals,
                                pcl::CorrespondencesPtr &corr, int kfactor = 8 )
{
    pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal> est;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setInputTarget( cloud_target );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr sourceTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr targetTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    est.setSearchMethodSource(sourceTree);
    est.setSearchMethodTarget(targetTree);
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, 10 );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

void findIndNormalCorrespondences( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, pcl::PointCloud<pcl::PointNormal>::Ptr &target_normals,
                                   pcl::CorrespondencesPtr &corr,
                                   pcl::IndicesPtr &source_ind, pcl::IndicesPtr &target_ind,
                                   int kfactor = 20 )
{
    pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal> est;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setIndicesSource ( source_ind );
    est.setIndicesTarget( target_ind );
    est.setInputTarget( cloud_target );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr sourceTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr targetTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    est.setSearchMethodSource( sourceTree );
    est.setSearchMethodTarget( targetTree );
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, 10 );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

// REJECTORS
void normalCorrRejector( pcl::CorrespondencesPtr &corr_list, pcl::CorrespondencesPtr &corr_out,
                         pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals, pcl::PointCloud<pcl::PointNormal>::Ptr &target_normals,
                         pcl::IndicesPtr rej_ids, float angle_diff )
{
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejector ( new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    rejector->setThreshold( std::cos (  pcl::deg2rad(angle_diff)  ) );
    rejector->initializeDataContainer<PointNormal, PointNormal> ( );
    rejector->setInputSource <PointNormal> ( source_normals );
    rejector->setInputTarget <PointNormal> ( target_normals );
    rejector->setInputNormals <PointNormal, PointNormal> ( source_normals );
    rejector->setTargetNormals <PointNormal, PointNormal> ( target_normals );
    rejector->setInputCorrespondences( corr_list );
    rejector->getCorrespondences( *corr_out );
    rejector->getRejectedQueryIndices( *corr_out, *rej_ids );
    pcl::console::print ( pcl::console::L_INFO, "Correspondences are now down to %d after rejecting normal outliers.\n", corr_out->size() );
}

void one2oneRejector( pcl::CorrespondencesPtr &corr_list,
                      pcl::CorrespondencesPtr &corr_out )
{
    pcl::registration::CorrespondenceRejectorOneToOne rejector;
    rejector.setInputCorrespondences(corr_list);
    //rejector.setSourceNormals(source_normals);
    //rejector.setTargetNormals(target_normals);
    rejector.getCorrespondences(*corr_out);
    cout << "Correspondeces are down to " << corr_out->size() << " after removing repeating matches." << endl;
}

void distanceRejector( pcl::CorrespondencesPtr &corr_list,
                       pcl::CorrespondencesPtr &corr_out, 
                       float max_dist = 10 )
{
    pcl::registration::CorrespondenceRejectorDistance rejector;
    rejector.setInputCorrespondences( corr_list );
    rejector.setMaximumDistance( max_dist );
    rejector.getCorrespondences( *corr_out );
    cout << "Correspondences are down to " << corr_out->size() << " after applying the distance rejector." << endl;
}

// FEATURE EXTRACTORS
void findNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals )
{
    // create the ne and kdtree valids, pass the cloud, set search radius, and save the found normals.
    Eigen::Vector4f centroid;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    compute3DCentroid( *cloud_source, centroid );
    ne.setViewPoint( centroid[0], centroid[1], 10000.0f );
    ne.setKSearch( 8 );
    ne.setInputCloud( cloud_source );
    ne.setSearchMethod( kdtree );
    ne.compute( *source_normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, %d normals were found.\n", (int)source_normals->size() );
}

void findCurvatures( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals, pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr &curves )
{
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PrincipalCurvatures> pce;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree ( new pcl::search::KdTree<pcl::PointXYZ> );
    pce.setInputCloud( cloud );
    pce.setInputNormals( normals );
    pce.setSearchMethod( tree );
    pce.setKSearch( 8 );
    pce.compute( *curves );
}

// TOOLS
Eigen::Matrix4f findTF( pcl::PointCloud<pcl::PointXYZ>::Ptr &nss_overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &nss_overlap_target, pcl::CorrespondencesPtr &corr_list )
{
    Eigen::Matrix4f tf;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
    te.estimateRigidTransformation( *nss_overlap_source, *nss_overlap_target, *corr_list, tf );
    cout << "The approximated transfer function is: " << endl << tf << endl;
    return tf;
}

void cloudsViewer( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, 
                   pcl::PointCloud<pcl::PointNormal>::Ptr source_normals, 
                   pcl::PointCloud<pcl::PointNormal>::Ptr target_normals,
                   pcl::CorrespondencesPtr &corr_list ) 
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 0, 0 );
    viewer->addPointCloud<PointXYZ>( cloud_source, source_color, "first" );
    viewer->addPointCloud( cloud_target, target_color, "second", 0 );
    viewer->addCorrespondences<pcl::PointXYZ>( cloud_source, cloud_target, *corr_list ); // views connecting lines betwenn each point pair.
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>( cloud_source, source_normals, 10, 0.2, "source" );
    viewer->addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>( cloud_target, target_normals, 10, 0.2, "target" );
    Eigen::Vector4f centroid;
    compute3DCentroid( *cloud_source, centroid );
    pcl::PointXYZ p1;
    p1.x = centroid[0];
    p1.y = centroid[1];
    p1.z = centroid[2];
    pcl::PointXYZ p2;
    p2.x = centroid[0];
    p2.y = centroid[1];
    p2.z = 10;
    viewer->addArrow(p1, p2, 0, 100, 100);
    viewer->spin();
}

void cloudsViewer( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target ) // Overload, without correspondences nor normals
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 0, 0 );
    viewer->addPointCloud<PointXYZ>( cloud_source, source_color, "first" );
    viewer->addPointCloud( cloud_target, target_color, "second", 0 );
    viewer->spin();
}

void saveFile( string file_name, pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud )
{
    int counter = 1;
    ofstream out(file_name);
    out << "# .PCD v.7 - Point Cloud Data file\n" << "VERSION .7\n" << "FIELDS x y z\n" << "SIZE 4 4 4\n" << "TYPE F F F\n" << "COUNT 1 1 1\n"
    << "WIDTH 1\n" << "HEIGHT " << ds_cloud->points.size() << "\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << (ds_cloud->points.size()) << "\nDATA ascii\n";
    for(int i = 0; i < ds_cloud->points.size(); i++) {
        out << ds_cloud->points[i].x << " " 
        << ds_cloud->points[i].y << " "
        << ds_cloud->points[i].z << endl;
    }
    out.close();
    cout << "The clouds were merged and saved in " << file_name << " successfully." << endl;
}

void combineClouds( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, 
                   pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_sum, string file_name, string dir, Positions* &source, Positions* &target )
{
    pcl::PointXYZ smin; 
    pcl::PointXYZ smax;
    getMinMax3D( *cloud_source, smin, smax);
    pcl::PointXYZ tmin; 
    pcl::PointXYZ tmax;
    getMinMax3D( *cloud_target, tmin, tmax); 
    pcl::PassThrough<PointXYZ> passer;

    // eliminate double overlap-area by cropping it out of the source cloud
    passer.setInputCloud( cloud_source );
    if( dir == "y" ) {
        //cropping
        passer.setFilterFieldName( "y" );
        if( source->y0 > target->y0 )
        { // source is on top
            passer.setFilterLimits( ( tmax.y-10 ) , ( smax.y ) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.y , (tmax.y-10) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_target );
        }
        else if( source->y0 < target->y0 )
        { // source is under
            passer.setFilterLimits( ( smin.y ) , ( tmin.y+10 ) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.y+10 , tmax.y );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_target );
        }
        
        //saving
        int total_size = cloud_source->size() + cloud_target->size();
        cloud_sum->resize( total_size );
        
        for ( int i = 0; i < cloud_target->size(); i++)
        {
            cloud_sum->points[i] = cloud_target->points[i];
        }
        for ( int i = 0; i < cloud_source->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()] = cloud_source->points[i];
        }
    }

    else if( dir == "x" ) {
        //cropping
        passer.setFilterFieldName( "x" );
        if( source->xc > target->xc ) // source is on the right
        {
            passer.setFilterLimits( ( tmax.x-10 ) , ( smax.x ) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.x , (tmax.x-10) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_target );
        }
        else if( source->xc < target->xc )
        { // source is on the left
            passer.setFilterLimits( ( smin.x ) , ( tmin.x+10 ) );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.x+10 , tmax.x );
            passer.pcl::Filter<PointXYZ>::filter( *cloud_target );
        }

        //saving
        int total_size = cloud_source->size() + cloud_target->size();
        cloud_sum->resize( total_size );
        
        for ( int i = 0; i < cloud_target->size(); i++)
        {
            cloud_sum->points[i] = cloud_target->points[i];
        }
        for ( int i = 0; i < cloud_source->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()] = cloud_source->points[i];
        }
    }

    else if( dir == "xy" ) {
        //cropping on x-direction
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_rest ( new pcl::PointCloud<pcl::PointXYZ> ); // point cloud
        if ( abs(smin.x-tmin.x) < abs(smax.x-tmax.x) ) //source is on the left
        {
            cout << "xy - left - ";
            //relative source's x-segment
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , (smax.x-10) );
            passer.filter( *cloud_source );
            //non-overlapped target segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smax.x-10) , tmax.x );
            passer.filter( *cloud_target_rest );
            //overlapped target's x-segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , smax.x-10 );
            passer.filter( *cloud_target );
        }
        else //source is on the right
        {
            //relative source's x-segment
            cout << "xy - right - ";
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smin.x+10) , smax.x );
            passer.filter( *cloud_source );
            //non-overlapped target segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , (smin.x+10) );
            passer.filter( *cloud_target_rest );
            //overlapped target's x-segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smin.x+10) , tmax.x );
            passer.filter( *cloud_target );
        }

        //cropping on y-direction
        if (source->y0 > target->y0) //source is above
        {
            cout << " up." << endl;
            //relative source's y-segment
            passer.setInputCloud( cloud_source );   
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smin.y+10) , smax.y);
            passer.filter( *cloud_source );
            //relative target's y-segment
            passer.setInputCloud( cloud_target );           
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , (smin.y+10) );
            passer.filter( *cloud_target );
        }
        else //source is under
        {
            //relative source's y-segment
            cout << " down." << endl;
            passer.setInputCloud( cloud_source );   
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , (smax.y-10) );
            passer.filter( *cloud_source );
            //relative target's y-segment
            passer.setInputCloud( cloud_target );           
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smax.y-10) , tmax.y );
            passer.filter( *cloud_target );
        }

        //saving
        int total_size = cloud_source->size() + cloud_target->size() + cloud_target_rest->size();
        cloud_sum->resize( total_size );
        
        for ( int i = 0; i < cloud_target->size(); i++)
        {
            cloud_sum->points[i] = cloud_target->points[i];
        }
        for ( int i = 0; i < cloud_source->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()] = cloud_source->points[i];
        }
        for ( int i = 0; i < cloud_target_rest->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()+cloud_source->size()] = cloud_target_rest->points[i];
        }
    }

    else if( dir == "yx" ) {
        //cropping on y-direction
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_rest ( new pcl::PointCloud<pcl::PointXYZ> ); // point cloud
        if (source->y0 < target->y0) //source is under
        {
            cout << "yx - down -";
            //relative source's x-segment
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , (smax.y-10) );
            passer.filter( *cloud_source );
            //non-overlapped target segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smax.y-10) , tmax.y );
            passer.filter( *cloud_target_rest );
            //overlapped target's y-segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , smax.y-10 );
            passer.filter( *cloud_target );
        }
        else //source is above
        {
            //relative source's y-segment
            cout << "yx - up -";
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smin.y+10) , smax.y );
            passer.filter( *cloud_source );
            //non-overlapped target segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( tmin.y , (smin.y+10) );
            passer.filter( *cloud_target_rest );
            //overlapped target's y-segment
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( (smin.y+10) , tmax.y );
            passer.filter( *cloud_target );
        }

        //cropping on x-direction
        if (abs(smin.x-tmin.x) > abs(smax.x-tmax.x)) //source in on the right 
        {
            cout << "right" << endl;
            //relative source's x-segment
            passer.setInputCloud( cloud_source );   
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smin.x+10) , smax.x);
            passer.filter( *cloud_source );
            //relative target's x-segment
            passer.setInputCloud( cloud_target );           
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( tmin.x , (smin.x+10) );
            passer.filter( *cloud_target );
        }
        else //source is to the left
        {
            //relative source's x-segment
            cout << "left" << endl;
            passer.setInputCloud( cloud_source );   
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x , (smax.x-10) );
            passer.filter( *cloud_source );
            //relative target's x-segment
            passer.setInputCloud( cloud_target );           
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( (smax.x-10) , tmax.x );
            passer.filter( *cloud_target );
        }

        //saving
        int total_size = cloud_source->size() + cloud_target->size() + cloud_target_rest->size();
        cloud_sum->resize( total_size );
        
        for ( int i = 0; i < cloud_target->size(); i++)
        {
            cloud_sum->points[i] = cloud_target->points[i];
        }
        for ( int i = 0; i < cloud_source->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()] = cloud_source->points[i];
        }
        for ( int i = 0; i < cloud_target_rest->size(); i++)
        {
            cloud_sum->points[i+cloud_target->size()+cloud_source->size()] = cloud_target_rest->points[i];
        }
    }
} 

void nullCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud )
{
    Eigen::Matrix4f tf;
    pcl::PointXYZ min;
    pcl::PointXYZ max;
    float cx (0.0); 
    float cy (0.0);

    // shift the cloud minimums to 0.
    getMinMax3D( *cloud, min, max );
    tf << 1, 0, 0, -min.x,
          0, 1, 0, -min.y,
          0, 0, 1, 0,
          0, 0, 0, 1
    ;
    transformPointCloud( *cloud, *cloud, tf );
}

main( int argc, char **argv ) 
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
    // straight vector pointing in the z-direction used to compare with points normal angles.
    Eigen::Vector3f straight_vec;
    straight_vec(0) = 0;
    straight_vec(1) = 0;
    straight_vec(2) = 1;

    // create pointer valids for source and target clouds, with all other variable dependencies.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> ); // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // overlapped region
    pcl::PointCloud<pcl::PointNormal>::Ptr source_normals ( new pcl::PointCloud<pcl::PointNormal> ); // cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // sampled cloud
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr source_curves ( new pcl::PointCloud<PrincipalCurvatures> ); // cloud curvatures
    pcl::IndicesPtr source_ind ( new std::vector <int> ); // point indices of curvy points
    Positions *source = new Positions(); // object for storing key values of the cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointNormal>::Ptr target_normals ( new pcl::PointCloud<pcl::PointNormal> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr target_curves ( new pcl::PointCloud<PrincipalCurvatures> );
    pcl::IndicesPtr target_ind ( new std::vector <int> );
    Positions *target = new Positions();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum ( new pcl::PointCloud<pcl::PointXYZ> );

    pcl::CorrespondencesPtr corr_list ( new pcl::Correspondences );  
    pcl::CorrespondencesPtr corr_list_out ( new pcl::Correspondences );
    pcl::IndicesPtr rej_ids ( new std::vector <int> );

    // Loading source and target files:
    cout << "Loading pcd files...\n";
    loadFile ( argv[1], *cloud_target );
    loadFile ( argv[2], *cloud_source );

    // Defining key values: 
    string dir = argv[3];
    
    
    if(dir == "x") { target->xc = (stof(argv[4])+stof(argv[5]))/2; source->xc = (stof(argv[6])+stof(argv[7]))/2; }
    else if(dir == "y") { target->y0 = stof(argv[4]); target->y1 = stof(argv[5]); source->y0 = stof(argv[6]); source->y1 = stof(argv[7]); }
    else if(dir == "xy" || dir == "yx") { target->xc = (stof(argv[4])+stof(argv[6]))/2; target->y0 = stof(argv[5]); target->y1 = stof(argv[7]);
                           source->xc = (stof(argv[8])+stof(argv[10]))/2; source->y0 = stof(argv[9]); source->y1 = stof(argv[11]);  }
    else { cout << "Direction is invalid." << endl; return 0; }

    // Initial aligning:
    // shift the point clouds to the robot's coordinate system
    nullCloud( cloud_target );
    nullCloud( cloud_source );
    // view the original input clouds after moving origins to 0 -optional-
    //cloudsViewer( cloud_source, cloud_target );
    // crop and pre-align point clouds
    initAlign( cloud_source, cloud_target, overlap_source, overlap_target, dir, target, source );
    // view the resultant overlap clouds -optional-
    cloudsViewer( overlap_source, overlap_target );

    // Downsampling the original clouds:
    findNormals ( overlap_source, source_normals );
    findNormals ( overlap_target, target_normals );
    normalSpaceSample ( overlap_source, nss_overlap_source, source_normals );
    normalSpaceSample ( overlap_target, nss_overlap_target, target_normals );

    // Estimating the correspondeces using normal shooting method:
    // calculating new normals of downsampled clouds
    findNormals ( nss_overlap_source, source_normals );
    findNormals ( nss_overlap_target, target_normals );
    copyPointCloud( *nss_overlap_source, *source_normals );
    copyPointCloud( *nss_overlap_target, *target_normals );
    // initiating the correspondence estimation pipeline
    findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list);
    normalCorrRejector( corr_list, corr_list_out, source_normals, target_normals, rej_ids, 5.0f ); // normal-angle rejector is used to assure that only almost-vertical correspondences are considered. 
    one2oneRejector( corr_list_out, corr_list_out );
    distanceRejector( corr_list_out, corr_list_out, 10 );

    // Initial transforming the source point cloud:
    // estimate the transformation matrix
    tf = findTF( nss_overlap_source, nss_overlap_target, corr_list_out );
    // view downsampled overlap clouds before the initial transformation -optional-
    //cout << "Clouds before initial - icp" << endl;
    //cloudsViewer( nss_overlap_source, nss_overlap_target, source_normals, target_normals, corr_list_out );
    // apply the matrix on the source clouds
    transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
    transformPointCloud( *cloud_source, *cloud_source, tf );
    // view downsampled overlap clouds after the initial transformation -optional-
    //cout << "Clouds after initial - icp" << endl;
    //cloudsViewer( nss_overlap_source, nss_overlap_target );
    
    // Iterating ICP 20 times:
    for (int i = 1; i <= 20; i++)
    {
        // use the indices of filtered points to estimate correspondences and then apply rejectors
        cout << "\nloop #" << i << endl;
        findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list_out );
        normalCorrRejector( corr_list_out, corr_list, source_normals, target_normals, rej_ids, 5.0f );
        one2oneRejector( corr_list, corr_list );
        distanceRejector( corr_list, corr_list, 2 );

        // estimate TF 
        tf = findTF( nss_overlap_source, nss_overlap_target, corr_list );
        
        // transform clouds
        transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
        transformPointCloud( *cloud_source, *cloud_source, tf );
        
        // calculate the confidence score of the transformation
        pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> valid;
        valid.setMaxRange( 10 );
        float conf_score = valid.validateTransformation( nss_overlap_source, nss_overlap_target, tf );
        cout << "Confidence score: " << conf_score << endl;
    } 
    
    // view the final result
    cloudsViewer( cloud_source, cloud_target );
    
    // save the merged clouds as one if the file name is specified.
    if ( argc == 9 )
    {
        string file_name = argv[8];
        combineClouds( cloud_source, cloud_target, cloud_sum, file_name, dir, source, target );
        saveFile( file_name, cloud_sum );
    }
    else if ( argc == 13 )
    {
        string file_name = argv[12];
        combineClouds( cloud_source, cloud_target, cloud_sum, file_name, dir, source, target );
        saveFile( file_name, cloud_sum );
    }

    pcl::PointXYZ min; 
    pcl::PointXYZ max;
    getMinMax3D( *cloud_sum, min, max);

    cout << "sum (min), (max): " << min << ", " << max << endl;
    cout << "The program took " << time.toc() << " ms to run." << endl;

    return 0;
}
    
