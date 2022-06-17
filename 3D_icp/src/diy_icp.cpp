#include <iostream>
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
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/icp.h>


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
        PCL_ERROR ( "Could not load %d.", (string) fileName );
        return;
    }
    else 
    {
        // store the cloud data from the mesh in the passed cloud object.
        pcl::fromPCLPointCloud2<pcl::PointXYZ> ( mesh.cloud, cloud );
        pcl::console::print ( pcl::console::L_INFO, "%d was loaded successfully.\n", (string) fileName );
    }
    // remove nan points from cloud and save the resulting PC in the same cloud, and the respective indices in index.
    std::vector<int> index;
    pcl::removeNaNFromPointCloud ( cloud, cloud, index );
}

void initAlign( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_target )
{
    // sensor displacement is calculated to be 38 in this example, however needs to be provided by other means.
    float sensor_displacement = 38.0;

    for(size_t i = 0; i < cloud_target->points.size(); ++i)
    {
        cloud_target->points[i].z += 1;
    }

    pcl::PassThrough<PointXYZ> passer;
    passer.setInputCloud( cloud_source );
    passer.setFilterFieldName( "x" );
    passer.setFilterLimits( ( cloud_source->points[0].x+sensor_displacement ) , ( cloud_source->points[cloud_source->size()-1].x ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_source );
    passer.setInputCloud( cloud_target );
    passer.setFilterFieldName( "x" );
    passer.setFilterLimits( ( cloud_target->points[0].x ) , ( cloud_target->points[cloud_target->size()-1].x-sensor_displacement ) );
    passer.pcl::Filter<PointXYZ>::filter( *overlap_target );

    // the target_overlap and cloud_target are shifted since the target was captured after moving the sensor
    for(size_t i = 0; i < overlap_target->points.size(); ++i)
    {
        overlap_target->points[i].x += sensor_displacement;
    }

    for( size_t i = 0; i < cloud_target->points.size(); ++i)
    {
        cloud_target->points[i].x += sensor_displacement;
    }

}

void transformationEstimation()
{

}

void findNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointNormal>::Ptr source_normals )
{
    // create the ne and kdtree objects, pass the cloud, set search radius, and save the found normals.
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    ne.setInputCloud( cloud_source );
    ne.setSearchMethod( kdtree );
    ne.setRadiusSearch( 0.03 );
    ne.compute( *source_normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, %d normals were found.\n", (int)source_normals.get()->size() );
}

void normalSpaceSample( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source,
                        pcl::PointCloud<pcl::PointNormal>::Ptr source_normals ) 
{
    // create the nss object, pass the cloud, the normals, and save the filtered cloud.
    pcl::NormalSpaceSampling<pcl::PointXYZ,pcl::PointNormal> nss;
    nss.setSample( cloud_source->size() / 4 );
    nss.setBins(1, 1, 1);
    nss.setSeed(0);
    nss.setInputCloud( cloud_source );
    nss.setNormals( source_normals );
    nss.pcl::Filter<pcl::PointXYZ>::filter( *nss_overlap_source );
    //nss.filter( *cloud_indices );
    pcl::console::print ( pcl::console::L_INFO, "Source cloud was downsampled from %d points to %d points\n", (int)cloud_source.get()->size(), (int)nss_overlap_source.get()->size() );
}

void findNormalCorrespondences( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source,
                                pcl::PointCloud<pcl::PointNormal>::Ptr source_normals,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                                pcl::CorrespondencesPtr corr,
                                int kfactor = 10 )
{
    pcl::registration::CorrespondenceEstimationNormalShooting<pcl::PointXYZ, pcl::PointXYZ, pcl::PointNormal> est;
    est.setInputSource( cloud_source );
    est.setSourceNormals( source_normals );
    est.setInputTarget( cloud_target );
    est.setKSearch( kfactor );
    est.determineCorrespondences( *corr );
    pcl::console::print ( pcl::console::L_INFO, "There were %d correspondences found.\n", corr->size() );
}

void rejectCorrespondences( pcl::CorrespondencesPtr corr_list,
                            pcl::CorrespondencesPtr corr_rej,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, 
                            pcl::PointCloud<pcl::PointNormal>::Ptr source_normals,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target,
                            pcl::PointCloud<pcl::PointNormal>::Ptr target_normals )
{
    pcl::registration::CorrespondenceRejectorSurfaceNormal rejector;
    rejector.setThreshold( std::acos ( deg2rad ( 0.0 ) ) );
    rejector.initializeDataContainer<PointXYZ, PointNormal> ();
    rejector.setInputSource <PointXYZ> ( cloud_source );
    rejector.setInputTarget <PointXYZ> ( cloud_target );
    rejector.setInputNormals <PointXYZ, PointNormal> ( source_normals );
    rejector.setTargetNormals <PointXYZ, PointNormal> ( target_normals );
    rejector.setInputCorrespondences( corr_list );
    pcl::console::print ( pcl::console::L_INFO, "Rejection process started with %d correspondences.\n", corr_list->size() );
    rejector.getCorrespondences( *corr_rej );
    pcl::console::print ( pcl::console::L_INFO, "Correspondences are now down to %d after rejecting outliers.\n", corr_rej->size() );
}

void findTF( pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source, pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_target, pcl::CorrespondencesPtr corr_list, Eigen::Matrix4f tf )
{
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
    te.estimateRigidTransformation( *nss_overlap_source, *nss_overlap_target, *corr_list, tf );
    cout << "The approximated transfer function is: " << endl << tf << endl;
}

void cloudsViewer( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target )
{
    pcl::visualization::PCLVisualizer viewer( "Cloud Viewer" );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color ( cloud_source, 0, 255, 0 );
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color ( cloud_target, 255, 0, 0 );
    viewer.addPointCloud( cloud_source, source_color, "first", 0 );
    viewer.addPointCloud( cloud_target, target_color, "second", 0 );
    viewer.spin();
}

main( int argc, char **argv ) 
{
    // create pointer objects for source and target clouds, with all other variable dependencies.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source ( new pcl::PointCloud<pcl::PointXYZ> () ); // point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_source ( new pcl::PointCloud<pcl::PointXYZ> () ); // overlapped region
    pcl::PointCloud<pcl::PointNormal>::Ptr source_normals ( new pcl::PointCloud<pcl::PointNormal>() ); // cloud normals
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_source ( new pcl::PointCloud<pcl::PointXYZ> () ); // sampled cloud

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointXYZ>::Ptr overlap_target ( new pcl::PointCloud<pcl::PointXYZ> () );
    pcl::PointCloud<pcl::PointNormal>::Ptr target_normals ( new pcl::PointCloud<pcl::PointNormal>() );
    pcl::PointCloud<pcl::PointXYZ>::Ptr nss_overlap_target ( new pcl::PointCloud<pcl::PointXYZ> () );



    // load source and target files.
    cout << "Loading pcd files...\n";
    loadFile ( argv[1], *cloud_source );
    loadFile ( argv[2], *cloud_target );

    // crop point clouds
    initAlign( cloud_source, cloud_target, overlap_source, overlap_target );

    // calculate normals and sample the source cloud.
    findNormals ( overlap_source, source_normals );
    findNormals ( overlap_target, target_normals );
    normalSpaceSample ( overlap_source, nss_overlap_source, source_normals );
    normalSpaceSample ( overlap_target, nss_overlap_target, target_normals );

    // finding optimal correspondences
    pcl::CorrespondencesPtr corr_list ( new pcl::Correspondences );
    pcl::CorrespondencesPtr corr_list_rej ( new pcl::Correspondences );
    findNormals ( nss_overlap_source, source_normals );
    findNormals ( nss_overlap_target, target_normals );
    findNormalCorrespondences( nss_overlap_source, source_normals, nss_overlap_target, corr_list );
    rejectCorrespondences( corr_list, corr_list_rej, nss_overlap_source, source_normals, nss_overlap_target, target_normals );

    // estimate the transformation matrix
    Eigen::Matrix4f tf;
    findTF(nss_overlap_source, nss_overlap_target, corr_list, tf);

    // apply transformation matrix on hte source cloud
    pcl::transformPointCloud( *cloud_source, *cloud_source, tf );

    // launch the pcl visualizer.
    cloudsViewer( cloud_source, cloud_target);

    return 0;

}