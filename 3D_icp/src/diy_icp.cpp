#include <iostream>
#include <cmath>
#include <math.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/distances.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_estimation_backprojection.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/transformation_validation_euclidean.h>

using namespace std;
using namespace pcl;

void loadFile( const char* fileName, pcl::PointCloud<pcl::PointXYZ> &cloud ) 
{
    // mech object to store the cloud.
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
        // store the cloud data from the mesh in the passed cloud object.
        pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
        cout << std::string(fileName);
        pcl::console::print ( pcl::console::L_INFO, " was loaded successfully.\n" );
    }
    // remove nan points from cloud and save the resulting PC in the same cloud, and the respective indices in index.
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

void initAlign( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr &overlap_target )
{
    cout << "Pre-aligning clouds..." << endl;
    // sensor displacement is experimentally calculated to be 57 in this example, however needs to be provided by other means.
    float sensor_displacement = 57.0;

    Eigen::Matrix4f tfx;
    tfx << 1, 0, 0, sensor_displacement,
          0, 1, 0, 0,
          0, 0, 1, 0,
          0, 0, 0, 1
    ;

    Eigen::Matrix4f tfz;
    tfz << 1, 0, 0, 0,
           0, 1, 0, 0,
           0, 0, 1, 1, 
           0, 0, 0, 1
    ;

    pcl::PointXYZ tmin_pt;
    pcl::PointXYZ tmax_pt;
    getMinMax3D( *cloud_target, tmin_pt, tmax_pt );
    float tmin_x = tmin_pt.x;
    float tmax_x = tmax_pt.x;
    pcl::PointXYZ smin_pt;
    pcl::PointXYZ smax_pt;
    getMinMax3D( *cloud_source, smin_pt, smax_pt );
    float smin_x = smin_pt.x;
    float smax_x = smax_pt.x;

    // shift on z-axis to have a visibile gap to work with
    transformPointCloud( *cloud_source, *cloud_source, tfz );

    // clouds cropping
    pcl::PassThrough<PointXYZ> passer;
    passer.setInputCloud( cloud_target );
    passer.setFilterFieldName( "x" );
    passer.setFilterLimits( ( tmin_x+sensor_displacement ) , ( tmax_x ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_target );
    passer.setInputCloud( cloud_source );
    passer.setFilterFieldName( "x" );
    passer.setFilterLimits( ( smin_x ) , ( smax_x-sensor_displacement ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_source );

    // shift on x-axis
    transformPointCloud( *overlap_source, *overlap_source, tfx );
    transformPointCloud( *cloud_source, *cloud_source, tfx );
}

void normalSpaceSample( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &nss_overlap_source,
                        pcl::PointCloud<pcl::PointNormal>::Ptr &source_normals ) 
{
    // create the nss object, pass the cloud, the normals, and save the filtered cloud.
    pcl::NormalSpaceSampling<pcl::PointXYZ,pcl::PointNormal> nss;
    nss.setSample( cloud_source->size() / 4 );
    nss.setBins( 10, 10, 10 );
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
    est.setIndicesSource( source_ind );
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
                         pcl::IndicesPtr rej_ids )
{
    pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr rejector ( new pcl::registration::CorrespondenceRejectorSurfaceNormal );
    rejector->setThreshold( std::cos (  pcl::deg2rad(15.0)  ) );
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
    // create the ne and kdtree objects, pass the cloud, set search radius, and save the found normals.
    Eigen::Vector4f centroid;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    compute3DCentroid( *cloud_source, centroid );
    ne.setViewPoint( centroid[0], centroid[1], 100.0f );
    ne.setKSearch( 8 );
    ne.setInputCloud( cloud_source );
    ne.setSearchMethod( kdtree );
    ne.compute( *source_normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, %d normals were found.\n", (int)source_normals->size() );
}

pcl::IndicesPtr filterByAngle( pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_normals, const Eigen::Vector3f requiredDirection, float maxAngleDeg )
{
    pcl::IndicesPtr indices ( new std::vector <int> );
    float angle, lenSq1, dot;

    for( int i = 0; i < cloud_normals->size(); ++i )
    {
        dot = cloud_normals->points[i].normal_z;
        lenSq1 = sqrt(pow(cloud_normals->points[i].normal_x, 2.0f) + pow(cloud_normals->points[i].normal_y, 2.0f) + pow(cloud_normals->points[i].normal_z, 2.0f));
        angle = pcl::rad2deg( acos(dot/lenSq1) );

        if ( angle >= maxAngleDeg ) { indices->push_back(i); }
    }

    cout << indices->size() << " points are above 45°." << endl;
    return indices;
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
    Eigen::Vector4f centroid;
    pcl::visualization::PCLVisualizer::Ptr viewer ( new pcl::visualization::PCLVisualizer ("3D Viewer") );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 0, 0 );
    compute3DCentroid( *cloud_source, centroid );
    pcl::PointXYZ p1;
    p1.x = centroid[0];
    p1.y = centroid[1];
    p1.z = centroid[2];
    pcl::PointXYZ p2;
    p2.x = centroid[0];
    p2.y = centroid[1];
    p2.z = 10;
    viewer->addPointCloud<PointXYZ>( cloud_source, source_color, "first" );
    viewer->addPointCloud( cloud_target, target_color, "second", 0 );
    viewer->addCorrespondences<pcl::PointXYZ>( cloud_source, cloud_target, *corr_list );
    viewer->addArrow(p1, p2, 0, 100, 100); // views the straight vector used to compare with the points normal vectors
    viewer->addSphere(p1, 0.5); // only for reference
    //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>( cloud_source, source_normals, 10, 0.2, "source" );
    //viewer->addPointCloudNormals<pcl::PointXYZ,pcl::PointNormal>( cloud_target, target_normals, 10, 0.2, "target" );
    viewer->spin();
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

    // create pointer objects for source and target clouds, with all other variable dependencies.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> ); // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // overlapped region
    pcl::PointCloud<pcl::PointNormal>::Ptr source_normals ( new pcl::PointCloud<pcl::PointNormal> ); // cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source ( new pcl::PointCloud<pcl::PointXYZ> ); // sampled cloud
    pcl::IndicesPtr source_ind ( new std::vector <int>  ); // point indices of curvy points

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointNormal>::Ptr target_normals ( new pcl::PointCloud<pcl::PointNormal> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_target ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::IndicesPtr target_ind ( new std::vector <int> );

    // load source and target files.
    cout << "Loading pcd files...\n";
    loadFile ( argv[1], *cloud_target );
    loadFile ( argv[2], *cloud_source );

    // crop and pre-align point clouds
    initAlign( cloud_source, cloud_target, overlap_source, overlap_target );
    
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
    findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list );
    normalCorrRejector( corr_list, corr_list_out, source_normals, target_normals, rej_ids ); // normal-angle rejector is used to assure that only correspondences resulting in vertical shifitng of source are considered. 

    // estimate the transformation matrix
    tf = findTF( nss_overlap_source, nss_overlap_target, corr_list_out );

    // apply transformation matrix on the source clouds
    transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
    transformPointCloud( *cloud_source, *cloud_source, tf );

    // filter out points with normals that have more than 30° to the "surface normal vector" 
    source_ind = filterByAngle( source_normals, straight_vec, 25 );
    target_ind = filterByAngle( target_normals, straight_vec, 25 );

    // iterate until clouds are merged
    for (int i = 1; i <= 70; i++)
    {
        // use the indices of filtered points to estimate correspondences and then apply rejectors
        cout << "loop #" << i << endl;
        findIndNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, target_normals, corr_list_out, source_ind, target_ind );
        normalCorrRejector( corr_list_out, corr_list, source_normals, target_normals, rej_ids );
        one2oneRejector( corr_list_out, corr_list_out );
        
        // estimate TF 
        tf = findTF( nss_overlap_source, nss_overlap_target, corr_list );
        
        // transform clouds
        transformPointCloud( *nss_overlap_source, *nss_overlap_source, tf );
        transformPointCloud( *cloud_source, *cloud_source, tf );
        
        // calculate the confidence score of the transformation
        pcl::registration::TransformationValidationEuclidean<pcl::PointXYZ, pcl::PointXYZ> object;
        object.setMaxRange( 10 );
        float conf_score = object.validateTransformation( nss_overlap_source, nss_overlap_target, tf );
        cout << "Confidence score: " << conf_score << endl; // the clouds seemed to be merged when the confidence score readched 0.2
    }

    cloudsViewer( cloud_source, cloud_target, source_normals, target_normals, corr_list );

    return 0;
}
