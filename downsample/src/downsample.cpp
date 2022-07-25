#include <iostream>
#include <pcl/PolygonMesh.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/normal_space.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/time.h>

using namespace std;
using namespace pcl;

// DOWNSAMPLING METHODS
void uniformDS( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, double search_rad, pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud )
{
    pcl::console::TicToc time;
    time.tic();
    pcl::UniformSampling<PointXYZ> uds;
    uds.setInputCloud( cloud );
    uds.setRadiusSearch( search_rad );
    uds.filter( *ds_cloud );
    cout << "Cloud was downsampled in " << time.toc() << " ms to " << ds_cloud->points.size() << " points using uniform DS method." << endl;
}

void voxelDS( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float lx, float ly, float lz, pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud )
{
    pcl::console::TicToc time;
    time.tic();
    pcl::VoxelGrid<PointXYZ> vg;
    vg.setInputCloud( cloud );
    vg.setLeafSize( lx, ly, lz);
    vg.filter( *ds_cloud );
    cout << "Cloud was downsampled in " << time.toc() << " ms to " << ds_cloud->points.size() << " points using voxel DS method." << endl;
}

void normalDS( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointNormal>::Ptr &normals,
               float ds_factor, int binx, int biny, int binz, pcl::PointCloud<pcl::PointXYZ>::Ptr &ds_cloud ) 
{
    pcl::console::TicToc time;
    time.tic();
    pcl::NormalSpaceSampling<pcl::PointXYZ,pcl::PointNormal> nss;
    nss.setSample( cloud->size() * ds_factor );
    nss.setBins( binx, biny, binz ); // number of different bins on each axis. (each bin gets filled with points of a specific normal vlaue)
    nss.setSeed( 8 );
    nss.setInputCloud( cloud );
    nss.setNormals( normals );
    nss.pcl::Filter<pcl::PointXYZ>::filter( *ds_cloud );
    cout << "Cloud was downsampled in " << time.toc() << " ms to " << ds_cloud->points.size() << " points using normal DS method." << endl;
}

// TOOLS and FEATURES
void findNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, int k, pcl::PointCloud<pcl::PointNormal>::Ptr &normals )
{
    // create the ne and kdtree objects, pass the cloud, set search radius, and save the found normals.
    Eigen::Vector4f centroid;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree ( new pcl::search::KdTree<pcl::PointXYZ> () );
    compute3DCentroid( *cloud, centroid );
    ne.setViewPoint( centroid[0], centroid[1], 100.0f );
    ne.setKSearch( k );
    ne.setInputCloud( cloud );
    ne.setSearchMethod( kdtree );
    ne.compute( *normals );
    pcl::console::print ( pcl::console::L_INFO, "Source normals calculated, there were %d normals found.\n", (int)normals->size() );
}

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
    //std::vector<int> index;
    //pcl::removeNaNFromPointCloud ( cloud, cloud, index );
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
}

int main( int argc, char **argv )
{
    if( argc < 4 )
    {
        cout << "Format: ./downsample (voxel | uniform | normal) file_to_downsample.pcd name_of_new_file.pcd" << endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointXYZ>::Ptr ds_cloud ( new pcl::PointCloud<pcl::PointXYZ> );
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals ( new pcl::PointCloud<pcl::PointNormal> );

    loadFile( argv[2], *cloud);

    if( string(argv[1]) == "uniform" )
      uniformDS( cloud, 0.5, ds_cloud ); // parameters: input_cld, search_radius in mm, output_cld (bigger radius --> less points)

    else if( string(argv[1]) == "voxel" )
      voxelDS( cloud, 0.5, 0.5, 0.5, ds_cloud ); // parameters: input_cld, LeafSize_x in mm, LS_y in mm, LS_z in mm, output_cld (bigger leafs --> less points)

    else if( string(argv[1]) == "normal" )
    {
      findNormals( cloud, 8, cloud_normals); // param: inpt_cld, K-neighbours-search #, normals_cld
      normalDS( cloud, cloud_normals, 0.25, 10, 10, 1000, ds_cloud);// param: inpt_cld, normals, downsampling factor, xbins, ybins, zbins, output_cld
    }

    else 
    {
        cout << "Please choose a valid downsampling method." << endl;
        return -1;
    }
      
    saveFile( argv[3], ds_cloud );

    return 0;
    
}