#include <cmath>
#include <math.h>
#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/common/distances.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/normal_space.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/transformation_validation.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_validation_euclidean.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>

using namespace std;
using namespace pcl;

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
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target, double x, double y, double z )
{
    cout << "Pre-aligning clouds..." << endl;

    // constructing the new corresponding pose transfer function
    Eigen::Matrix4f tf;
    tf <<   1, 0, 0, x,
            0, 1, 0, y,
            0, 0, 1, z,
            0, 0, 0, 1
    ;
    cout << "this is the pre-aligning matrix: \n" << tf << endl;

    // calculating extreme points of the clouds
    pcl::PointXYZ tmin;
    pcl::PointXYZ tmax;
    getMinMax3D( *cloud_target, tmin, tmax );
    pcl::PointXYZ smin;
    pcl::PointXYZ smax;
    getMinMax3D( *cloud_source, smin, smax );

    // clouds cropping
    pcl::PassThrough<PointXYZ> passer;
    passer.setInputCloud( cloud_target );
    passer.setFilterFieldName( "y" );
    passer.setFilterLimits( ( tmin.y + abs(y) ) , ( tmax.y ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_target );
    passer.setInputCloud( cloud_source );
    passer.setFilterFieldName( "y" );
    passer.setFilterLimits( ( smin.y ) , ( smax.y - abs(y) ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_source );
    
    // shift on x-axis
    transformPointCloud( *overlap_source, *overlap_source, tf );
    transformPointCloud( *cloud_source, *cloud_source, tf );
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

void findBPCorrespondences( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, pcl::PointCloud<pcl::PointNormal>::Ptr &target_normals,
                            pcl::CorrespondencesPtr &corr, int kfactor = 8 ) 
{
    pcl::registration::CorrespondenceEstimationBackProjection<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr sourceTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr targetTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::PointRepresentation<pcl::PointXYZ>::Ptr pointRep;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setSearchMethodSource( sourceTree );
    est.setSearchMethodTarget( targetTree );
    est.setKSearch( kfactor );
    est.setPointRepresentation( pointRep );
    est.setInputTarget( cloud_target );
    est.setTargetNormals( target_normals );
    est.determineCorrespondences( *corr, 10 );
    pcl::console::print ( pcl::console::L_INFO, "There were %d BP correspondences found.\n", corr->size() );
}

void findIndBPCorrespondences( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, pcl::PointCloud<pcl::PointNormal>::Ptr &target_normals,
                               pcl::CorrespondencesPtr &corr,
                               pcl::IndicesPtr &source_ind, pcl::IndicesPtr &target_ind, int kfactor = 8 ) 
{
    pcl::registration::CorrespondenceEstimationBackProjection<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal> est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr sourceTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::search::KdTree<pcl::PointXYZ>::Ptr targetTree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    pcl::PointRepresentation<pcl::PointXYZ>::Ptr pointRep;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setSearchMethodSource( sourceTree );
    est.setSearchMethodTarget( targetTree );
    est.setIndicesSource( source_ind );
    est.setIndicesTarget( target_ind );
    est.setKSearch( kfactor );
    est.setPointRepresentation( pointRep );
    est.setInputTarget( cloud_target );
    est.setTargetNormals( target_normals );
    est.determineCorrespondences( *corr, 1000 );
    pcl::console::print ( pcl::console::L_INFO, "There were %d BP correspondences found.\n", corr->size() );
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

pcl::IndicesPtr filterByAngle( pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals, const Eigen::Vector3f requiredDirection, float AngleDeg, char LH )
{
    pcl::IndicesPtr indices ( new std::vector <int> );
    float angle, lenSq1, dot;

    for( int i = 0; i < cloud_normals->size(); ++i )
    {
        dot = cloud_normals->points[i].normal_z;
        lenSq1 = sqrt(pow(cloud_normals->points[i].normal_x, 2.0f) + pow(cloud_normals->points[i].normal_y, 2.0f) + pow(cloud_normals->points[i].normal_z, 2.0f));
        angle = pcl::rad2deg( acos(dot/lenSq1) );

        if( LH == 'L') { if ( angle <= AngleDeg ) { indices->push_back(i); } }

        else if( LH == 'H') { if ( angle >= AngleDeg ) { indices->push_back(i); } }
        
    }

    if( LH == 'L') { cout << indices->size() << " points are below " << AngleDeg << "°." << endl; }

    else if( LH == 'H') { cout << indices->size() << " points are over " << AngleDeg << "°." << endl; }
    
    return indices;
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

void saveCombined( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_sum, string file_name, float y )
{
    int total_size = cloud_source->size() + cloud_target->size();
    cloud_sum->resize( total_size );

    pcl::PointXYZ min;
    pcl::PointXYZ max;
    getMinMax3D( *cloud_source, min, max );
    pcl::PassThrough<PointXYZ> passer;
    passer.setInputCloud( cloud_source );
    passer.setFilterFieldName( "y" );
    passer.setFilterLimits( ( max.y - abs(y) ) , ( max.y ) );
    passer.pcl::Filter<PointXYZ>::filter( *cloud_source );

    for ( int i = 0; i < cloud_source->size(); i++)
    {
        cloud_sum->points[i] = cloud_source->points[i];
    }
    for ( int i = 0; i < cloud_target->size(); i++)
    {
        cloud_sum->points[i+cloud_source->size()-1] = cloud_target->points[i];
    }

    saveFile( file_name, cloud_sum );
}

main( int argc, char **argv ) 
{   
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

    // sensor shifting values:
    float x, z = 0.0;
    float y = 121.47;

    // create pointer valids for source and target clouds, with all other variable dependencies.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> ); // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // overlapped region
    pcl::PointCloud<pcl::PointNormal>::Ptr source_normals ( new pcl::PointCloud<pcl::PointNormal> ); // cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // sampled cloud
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr source_curves ( new pcl::PointCloud<PrincipalCurvatures> ); // cloud curvatures
    pcl::IndicesPtr source_ind ( new std::vector <int> ); // point indices of curvy points

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointNormal>::Ptr target_normals ( new pcl::PointCloud<pcl::PointNormal> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr target_curves ( new pcl::PointCloud<PrincipalCurvatures> );
    pcl::IndicesPtr target_ind ( new std::vector <int> );

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum ( new pcl::PointCloud<pcl::PointXYZ> );

    // load source and target files.
    cout << "Loading pcd files...\n";
    loadFile ( argv[1], *cloud_target );
    loadFile ( argv[2], *cloud_source );

    // crop and pre-align point clouds
    initAlign( cloud_source, cloud_target, overlap_source, overlap_target, x, y, z ); // The last three value are for the x, y, z shift values, respectively.
    
    // calculate normals and downsample the original clouds.
    findNormals ( overlap_source, source_normals );
    findNormals ( overlap_target, target_normals );

    normalSpaceSample ( overlap_source, nss_overlap_source, source_normals );
    normalSpaceSample ( overlap_target, nss_overlap_target, target_normals );

    // calculate normals of the downsampled clouds
    findNormals ( nss_overlap_source, source_normals );
    findNormals ( nss_overlap_target, target_normals );
    copyPointCloud( *nss_overlap_source, *source_normals );
    copyPointCloud( *nss_overlap_target, *target_normals );
    
    // estimate the correspondeces using normal shooting method (z-axis registration)
    pcl::CorrespondencesPtr corr_list ( new pcl::Correspondences );  
    pcl::CorrespondencesPtr corr_list_out ( new pcl::Correspondences );
    pcl::IndicesPtr rej_ids ( new std::vector <int> );

    // filters out the points of the flat regions
    source_ind = filterByAngle( source_normals, straight_vec, 10, 'L' );
    target_ind = filterByAngle( target_normals, straight_vec, 10, 'L' );

    findIndNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list, source_ind, target_ind );
    normalCorrRejector( corr_list, corr_list_out, source_normals, target_normals, rej_ids, 5 ); // normal-angle rejector is used to assure that only correspondences resulting in vertical shifitng of source are considered. 
    one2oneRejector( corr_list_out, corr_list_out );

    // estimate the transformation matrix
    tf = findTF( nss_overlap_source, nss_overlap_target, corr_list_out );
    cout << "Clouds before z - icp" << endl;
    cloudsViewer( nss_overlap_source, nss_overlap_target, source_normals, target_normals, corr_list_out );

    // apply transformation matrix on the source clouds
    transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
    transformPointCloud( *cloud_source, *cloud_source, tf );
    cout << "Clouds after z - icp\n\n" << endl;
    cloudsViewer( nss_overlap_source, nss_overlap_target );
    

    // iterate until clouds are merged
    for (int i = 1; i <= 20; i++)
    {
        // use the indices of filtered points to estimate correspondences and then apply rejectors
        cout << "loop #" << i << endl;
        findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list_out );
        normalCorrRejector( corr_list_out, corr_list, source_normals, target_normals, rej_ids, 5 );
        one2oneRejector( corr_list_out, corr_list_out );

        // estimate TF 
        tf = findTF( nss_overlap_source, nss_overlap_target, corr_list_out );
        
        // transform clouds
        transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
        transformPointCloud( *cloud_source, *cloud_source, tf );
        
        // calculate the confidence score of the transformation
        pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> valid;
        valid.setMaxRange( 10 );
        float conf_score = valid.validateTransformation( nss_overlap_source, nss_overlap_target, tf );
        cout << "Confidence score: " << conf_score << endl;
    } 
    
    if ( argc > 3 )
    {
        string file_name = argv[3];
        saveCombined( cloud_source, cloud_target, cloud_sum, file_name, y );
    }

    cloudsViewer( cloud_source, cloud_target );

    return 0;
}
    
