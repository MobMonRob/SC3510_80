#include <cmath>
#include <math.h>
#include <stdlib.h>
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
#include <pcl/filters/normal_space.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

using namespace std;
using namespace pcl;

using point = PointXYZ;
using normal = PointNormal;
using pointCloud = PointCloud<point>;
using normalCloud = PointCloud<normal>;

struct Positions {
    float x0 = 0.0;
    float x1 = 0.0;
    float y0 = 0.0;
    float y1 = 0.0;
    float z0 = 0.0;
    float z1 = 0.0;
    float xc = (x0+x1)/2; // x_center 
};
void cloudsViewer(pointCloud::Ptr &cloud1, pointCloud::Ptr &cloud2);

void loadFile( const char* fileName, pointCloud &cloud ) 
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

void preAlign(pointCloud::Ptr &cloud, Positions* &source, Positions* &target)
{
    cout << "Pre-aligning clouds..." << endl;

    // constructing the new corresponding pose transfer function
    Eigen::Matrix4f tf;

    float x_dist = source->x0 - target->x0;
    float y_dist = source->y0 - target->y0;
    float z_dist = source->z0 - target->z0;

    tf << 1, 0, 0, x_dist,
          0, 1, 0, y_dist,
          0, 0, 1, z_dist,
          0, 0, 0, 1
    ;
    cout << "the cloud transformation function is: " << endl << tf << endl;
    transformPointCloud( *cloud, *cloud, tf );
}

void cropClouds( pointCloud::Ptr &cloud_source, pointCloud::Ptr &cloud_target,
                pointCloud::Ptr &overlap_source, pointCloud::Ptr &overlap_target, string dir, Positions* &target, Positions* &source )
{
    // calculating extreme points of the clouds after matching the sensor's position
    point tmin;    
    point tmax;
    getMinMax3D( *cloud_target, tmin, tmax );
    point smin;
    point smax;
    getMinMax3D( *cloud_source, smin, smax );

    cout << "source (min), (max): " << smin << ", " << smax << endl;
    cout << "target (min), (max): " << tmin << ", " << tmax << endl;
    
    // clouds cropping (for optimal cropping results, make sure that the overlapping dimensions do not exceed 1/3 the source cloud on each direction)
    if( dir == "y" )
    {
        if( source->y0 > target->y0 )
        {
            // cropping the top relative third of the target cloud
            pcl::PassThrough<point> passer;
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
            pcl::PassThrough<point> passer;
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
            pcl::PassThrough<point> passer;
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
            pcl::PassThrough<point> passer;
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

    else if( dir == "xy" || dir == "xyz" )
    {
            // limit the x-area to match the source's
            pcl::PassThrough<point> passer;
            Eigen::Vector4f sminv;
            Eigen::Vector4f smaxv;
            passer.setInputCloud( cloud_target );
            passer.setFilterFieldName( "x" );
            passer.setFilterLimits( smin.x, smax.x );
            passer.filter( *overlap_target );
            // limit the y-area to match the source's
            passer.setInputCloud( overlap_target );
            passer.setFilterFieldName( "y" );
            passer.setFilterLimits( smin.y , smax.y );  
            passer.filter( *overlap_target );
            // crop out the non-overlapping box from source cloud
            if ( abs(source->xc-tmin.x) > abs(source->xc-tmax.x) ) // source is on the right
            {        
                sminv[0] = smin.x+abs(smax.x-smin.x)/3;
                smaxv[0] = smax.x;
            }
            else // source is on the left
            {        
                sminv[0] = smin.x;
                smaxv[0] = smax.x-abs(smax.x-smin.x)/3;
            }
            if ( source->y0 > target->y0 ) // source is above
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
            cb.setInputCloud( cloud_source );
            cb.setNegative( true );
            cb.setMin( sminv );
            cb.setMax( smaxv );
            cb.filter( *overlap_source );

    }
}

void normalSpaceSample( pointCloud::Ptr &cloud_source,
                        pointCloud::Ptr &nss_overlap_source,
                        normalCloud::Ptr &source_normals ) 
{
    // create the nss valid, pass the cloud, the normals, and save the filtered cloud.
    pcl::NormalSpaceSampling<point,normal> nss;
    nss.setSample( cloud_source->size() / 480 );
    nss.setBins( 5, 5, 1000 ); // number of different bins on each axis. (each bin gets filled with points of a specific normal vlaue)
    nss.setSeed( 8 );
    nss.setInputCloud( cloud_source );
    nss.setNormals( source_normals );
    nss.pcl::Filter<point>::filter( *nss_overlap_source );
    pcl::console::print ( pcl::console::L_INFO, "Source cloud was downsampled from %d points to %d points\n", (int)cloud_source.get()->size(), (int)nss_overlap_source.get()->size() );
}

// MATCHERS
void findNormalCorrespondences( pointCloud::Ptr &cloud_source, normalCloud::Ptr &source_normals,
                                pointCloud::Ptr &cloud_target, normalCloud::Ptr &target_normals,
                                pcl::CorrespondencesPtr &corr, int kfactor = 8, float dist = 10 )
{
    pcl::registration::CorrespondenceEstimationNormalShooting<point, point, normal> est;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setInputTarget( cloud_target );
    pcl::search::KdTree<point>::Ptr sourceTree ( new pcl::search::KdTree<point> () );
    pcl::search::KdTree<point>::Ptr targetTree ( new pcl::search::KdTree<point> () );
    est.setSearchMethodSource(sourceTree);
    est.setSearchMethodTarget(targetTree);
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, dist );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

void findIndNormalCorrespondences( pointCloud::Ptr &cloud_source, normalCloud::Ptr &source_normals,
                                   pointCloud::Ptr &cloud_target, normalCloud::Ptr &target_normals,
                                   pcl::CorrespondencesPtr &corr,
                                   pcl::IndicesPtr &source_ind, pcl::IndicesPtr &target_ind,
                                   int kfactor = 8, float dist = 10 )
{
    pcl::registration::CorrespondenceEstimationNormalShooting<point, point, normal> est;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setIndicesSource ( source_ind );
    est.setIndicesTarget( target_ind );
    est.setInputTarget( cloud_target );
    pcl::search::KdTree<point>::Ptr sourceTree ( new pcl::search::KdTree<point> () );
    pcl::search::KdTree<point>::Ptr targetTree ( new pcl::search::KdTree<point> () );
    est.setSearchMethodSource( sourceTree );
    est.setSearchMethodTarget( targetTree );
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr, dist );
    pcl::console::print ( pcl::console::L_INFO, "There were %d normal correspondences found.\n", corr->size() );
}

// REJECTORS
void normalCorrRejector( pcl::CorrespondencesPtr &corr_list, pcl::CorrespondencesPtr &corr_out,
                         normalCloud::Ptr &source_normals, normalCloud::Ptr &target_normals,
                         pcl::IndicesPtr rej_ids, float angle_diff )
{
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejector ( new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    rejector->setThreshold( std::cos (  pcl::deg2rad(angle_diff)  ) );
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

// FEATURE EXTRACTORS
void findNormals( pointCloud::Ptr &cloud_source, normalCloud::Ptr &source_normals )
{
    // create the ne and kdtree valids, pass the cloud, set search radius, and save the found normals.
    Eigen::Vector4f centroid;
    pcl::NormalEstimation<point, normal> ne;
    pcl::search::KdTree<point>::Ptr kdtree ( new pcl::search::KdTree<point> () );
    compute3DCentroid( *cloud_source, centroid );
    ne.setViewPoint( centroid[0], centroid[1], 10000.0f );
    ne.setKSearch( 8 );
    ne.setInputCloud( cloud_source );
    ne.setSearchMethod( kdtree );
    ne.compute( *source_normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, %d normals were found.\n", (int)source_normals->size() );
}

pcl::IndicesPtr filterByAngle( normalCloud::Ptr &cloud_normals, const Eigen::Vector3f requiredDirection, float AngleDeg, string LH )
{
    pcl::IndicesPtr indices ( new std::vector <int> );
    float angle, lenSq1, dot;

    for( int i = 0; i < cloud_normals->size(); ++i )
    {
        dot = cloud_normals->points[i].normal_z;
        lenSq1 = sqrt(pow(cloud_normals->points[i].normal_x, 2.0f) + pow(cloud_normals->points[i].normal_y, 2.0f) + pow(cloud_normals->points[i].normal_z, 2.0f));
        angle = pcl::rad2deg( acos(dot/lenSq1) );

        if( LH == "lower") 
        { 
            if ( angle <= AngleDeg ) 
                indices->push_back(i);
        }

        else if( LH == "higher") 
        { 
            if ( angle >= AngleDeg ) 
                indices->push_back(i);
        }            
    }

    cout << indices->size() << " from " << cloud_normals->size() << " point normals are " << LH << " than " << AngleDeg << "°." << endl;
    return indices;
}  

// TOOLS
Eigen::Matrix4f findTF( pointCloud::Ptr &nss_overlap_source, pointCloud::Ptr &nss_overlap_target, pcl::CorrespondencesPtr &corr_list )
{
    Eigen::Matrix4f tf;
    pcl::registration::TransformationEstimationSVD<point, point> te;
    te.estimateRigidTransformation( *nss_overlap_source, *nss_overlap_target, *corr_list, tf );
    cout << "The approximated transfer function is: " << endl << tf << endl;
    return tf;
}

void cloudsViewer( pointCloud::Ptr &cloud_source, 
                   pointCloud::Ptr &cloud_target, 
                   normalCloud::Ptr source_normals, 
                   normalCloud::Ptr target_normals,
                   pcl::CorrespondencesPtr &corr_list ) 
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<point> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<point> target_color ( cloud_target, 255, 0, 0 );
    viewer->addPointCloud<point>( cloud_source, source_color, "first" );
    viewer->addPointCloud( cloud_target, target_color, "second", 0 );
    viewer->addCorrespondences<point>( cloud_source, cloud_target, *corr_list ); // views connecting lines betwenn each point pair.
    viewer->addPointCloudNormals<point,normal>( cloud_source, source_normals, 10, 0.2, "source" );
    viewer->addPointCloudNormals<point,normal>( cloud_target, target_normals, 10, 0.2, "target" );
    Eigen::Vector4f centroid;
    compute3DCentroid( *cloud_source, centroid );
    viewer->spin();
}

void cloudsViewer( pointCloud::Ptr &cloud_source, 
                   pointCloud::Ptr &cloud_target ) // Overload, without correspondences nor normals
{
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<point> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<point> target_color ( cloud_target, 255, 0, 0 );
    viewer->addPointCloud<point>( cloud_source, source_color, "first" );
    viewer->addPointCloud( cloud_target, target_color, "second", 0 );
    viewer->spin();
}

void saveFile( string file_name, pointCloud::Ptr ds_cloud )
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

void combineClouds( pointCloud::Ptr &cloud_source, pointCloud::Ptr &cloud_target, 
                   pointCloud::Ptr &cloud_sum, string file_name, string dir, Positions* &source, Positions* &target )
{
    point smin; 
    point smax;
    getMinMax3D( *cloud_source, smin, smax);
    point tmin; 
    point tmax;
    getMinMax3D( *cloud_target, tmin, tmax); 
    pcl::PassThrough<point> passer;

    // eliminate double overlap-area by cropping it out of the source cloud
    passer.setInputCloud( cloud_source );
    if( dir == "y" ) {
        //cropping
        passer.setFilterFieldName( "y" );
        if( source->y0 > target->y0 )
        { // source is on top
            passer.setFilterLimits( ( tmax.y-10 ) , ( smax.y ) );
            passer.pcl::Filter<point>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.y , (tmax.y-10) );
            passer.pcl::Filter<point>::filter( *cloud_target );
        }
        else if( source->y0 < target->y0 )
        { // source is under
            passer.setFilterLimits( ( smin.y ) , ( tmin.y+10 ) );
            passer.pcl::Filter<point>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.y+10 , tmax.y );
            passer.pcl::Filter<point>::filter( *cloud_target );
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
            passer.pcl::Filter<point>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.x , (tmax.x-10) );
            passer.pcl::Filter<point>::filter( *cloud_target );
        }
        else if( source->xc < target->xc )
        { // source is on the left
            passer.setFilterLimits( ( smin.x ) , ( tmin.x+10 ) );
            passer.pcl::Filter<point>::filter( *cloud_source );
            passer.setInputCloud(cloud_target);
            passer.setFilterLimits( tmin.x+10 , tmax.x );
            passer.pcl::Filter<point>::filter( *cloud_target );
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
        pointCloud::Ptr cloud_target_rest ( new pointCloud ); // point cloud
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
} 

void nullCloud( pointCloud::Ptr &cloud )
{
    Eigen::Matrix4f tf;
    point min;
    point max;

    // shift the cloud minimums to 0.
    getMinMax3D( *cloud, min, max );
    tf << 1, 0, 0, -min.x,
          0, 1, 0, -min.y,
          0, 0, 1, -min.z,
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
    straight_vec(2) = 10;

    // create pointer valids for source and target clouds, with all other variable dependencies.
    pointCloud::Ptr cloud_source ( new pointCloud ); // point cloud
    pointCloud::Ptr overlap_source ( new pointCloud ); // overlapped region
    normalCloud::Ptr source_normals ( new normalCloud ); // cloud normals
    pointCloud::Ptr nss_overlap_source ( new pointCloud ); // sampled cloud
    pcl::IndicesPtr source_ind ( new std::vector <int> ); // point indices of curvy points
    Positions *source = new Positions(); // object for storing key values of the cloud

    pointCloud::Ptr cloud_target ( new pointCloud );
    pointCloud::Ptr overlap_target ( new pointCloud );
    normalCloud::Ptr target_normals ( new normalCloud );
    pointCloud::Ptr nss_overlap_target ( new pointCloud );
    pcl::IndicesPtr target_ind ( new std::vector <int> );
    Positions *target = new Positions();

    pointCloud::Ptr cloud_sum ( new pointCloud );

    pcl::CorrespondencesPtr corr_list ( new pcl::Correspondences );  
    pcl::CorrespondencesPtr corr_list_out ( new pcl::Correspondences );
    pcl::IndicesPtr rej_ids ( new std::vector <int> );

    // Loading source and target files:
    cout << "Loading pcd files...\n";
    loadFile ( argv[1], *cloud_target );
    loadFile ( argv[2], *cloud_source );

    // Defining key values: 
    string dir = argv[3];
    if(dir == "x") { target->x0 = stof(argv[4]); target->x1 = stof(argv[5]); source->x0 = stof(argv[6]);source->x1 = stof(argv[7]); }
    else if(dir == "y") { target->y0 = stof(argv[4]); target->y1 = stof(argv[5]); source->y0 = stof(argv[6]); source->y1 = stof(argv[7]); }
    else if(dir == "xy") { target->x0 = stof(argv[4]); target->x1 = stof(argv[6]); target->y0 = stof(argv[5]); target->y1 = stof(argv[7]);
                           source->x0 = stof(argv[8]); source->x1 = stof(argv[10]); source->y0 = stof(argv[9]); source->y1 = stof(argv[11]);  }
    else if(dir == "xyz") { target->x0 = stof(argv[4]); target->x1 = stof(argv[7]); target->y0 = stof(argv[5]); target->y1 = stof(argv[8]); target->z0 = stof(argv[6]); target->z1 = stof(argv[9]);
                            source->x0 = stof(argv[10]); source->x1 = stof(argv[13]); source->y0 = stof(argv[11]); source->y1 = stof(argv[14]); source->z0 = stof(argv[12]); source->z1 = stof(argv[15]); }
    else { cout << "Direction is invalid." << endl; return 0; }

    // shift the point clouds to the robot's coordinate system
    nullCloud( cloud_target );
    nullCloud( cloud_source );

    // view the original input clouds after moving origins to 0 -optional-
    //cloudsViewer( cloud_source, cloud_target );

    // pre-align the clouds based on the robot's positions
    preAlign( cloud_source, source, target );

    // view the original input clouds after initial alignment -optional-
    cloudsViewer( cloud_source, cloud_target );

    // crop the overlapped areas of the point clouds
    cropClouds( cloud_source, cloud_target, overlap_source, overlap_target, dir, target, source );

    // view the resultant overlap clouds -optional-
    //cloudsViewer( overlap_source, overlap_target );

    // downsampling the original clouds:
    findNormals ( overlap_source, source_normals );
    findNormals ( overlap_target, target_normals );
    normalSpaceSample ( overlap_source, nss_overlap_source, source_normals );
    normalSpaceSample ( overlap_target, nss_overlap_target, target_normals );

    
    // calculating new normals of downsampled clouds
    findNormals ( nss_overlap_source, source_normals );
    findNormals ( nss_overlap_target, target_normals );
    copyPointCloud( *nss_overlap_source, *source_normals );
    copyPointCloud( *nss_overlap_target, *target_normals );

    // estimating the correspondeces using normal shooting method:
    findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list, 8, 200);
    normalCorrRejector( corr_list, corr_list_out, source_normals, target_normals, rej_ids, 35.0f ); // normal-angle rejector is used to assure that only almost-vertical correspondences are considered. 
    one2oneRejector( corr_list_out, corr_list_out );
    
    // view downsampled overlap clouds before the initial transformation -optional-
    //cout << "Clouds before initial - icp" << endl;
    //cloudsViewer( nss_overlap_source, nss_overlap_target, source_normals, target_normals, corr_list_out );

    // estimate and apply the matrix on the source clouds
    tf = findTF( nss_overlap_source, nss_overlap_target, corr_list_out );
    transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
    transformPointCloud( *cloud_source, *cloud_source, tf );

    // view downsampled overlap clouds after the initial transformation -optional-
    //cout << "Clouds after initial - icp" << endl;
    //cloudsViewer( nss_overlap_source, nss_overlap_target );

    // filtering out points on straight areas
    source_ind = filterByAngle(source_normals, straight_vec, 10, "higher");
    target_ind = filterByAngle(target_normals, straight_vec, 10, "higher");

    // Iterate ICP until a convergence criteria is satisfied (aligning details)
    bool conv = false;
    int i = 1;
    while (!conv)
    {
        // estimate correspondences and then apply rejectors
        cout << "\nloop #" << i << endl;
        findIndNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list_out, source_ind, target_ind );
        normalCorrRejector( corr_list_out, corr_list, source_normals, target_normals, rej_ids, 35.0f );
        one2oneRejector( corr_list, corr_list );

        // view clouds iteratively before each transform -optional-
        //cloudsViewer( nss_overlap_source, nss_overlap_target, source_normals, target_normals, corr_list );
        
        // estimate TF and transform clouds
        tf = findTF( nss_overlap_source, nss_overlap_target, corr_list );
        transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
        transformPointCloud( *cloud_source, *cloud_source, tf );

        pcl::registration::DefaultConvergenceCriteria<float> converge ( i, tf, *corr_list );
        converge.setTranslationThreshold( 0.000005 );
        converge.setRotationThreshold( std::cos( pcl::deg2rad( 0.5f ) ));
        if ( converge.hasConverged() == true )
            break;
        i++;
    } 
    
    // view the detail-alignment result -optional-
    //cloudsViewer( cloud_source, cloud_target );
    
    // Iterating ICP until a convergence criteria is satisfied (aligning flat surfaces):
    conv = false;
    i = 1;
    while (!conv)
    {
        // estimate correspondences and then apply rejectors
        cout << "\nloop #" << i << endl;
        findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list_out, 8, 2 );
        normalCorrRejector( corr_list_out, corr_list, source_normals, target_normals, rej_ids, 35.0f );
        one2oneRejector( corr_list, corr_list );

        // view clouds iteratively before each transform -optional-
        //cloudsViewer( nss_overlap_source, nss_overlap_target, source_normals, target_normals, corr_list );

        // estimate TF and transform clouds
        tf = findTF( nss_overlap_source, nss_overlap_target, corr_list );
        transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
        transformPointCloud( *cloud_source, *cloud_source, tf );

        pcl::registration::DefaultConvergenceCriteria<float> converge ( i, tf, *corr_list );
        converge.setTranslationThreshold( 0.000005 );
        converge.setRotationThreshold( std::cos( pcl::deg2rad( 0.5f ) ));
        if ( converge.hasConverged() == true )
            break;
        i++;
    }

    // view the final result -optional-
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

    point min; 
    point max;
    getMinMax3D( *cloud_sum, min, max);

    cout << "sum (min), (max): " << min << ", " << max << endl;
    cout << "The program took " << time.toc() << " ms to run." << endl;

    return 0;
}
