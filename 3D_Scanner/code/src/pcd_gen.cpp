#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include<vector>

#include <GenicamSystem.h>
#include <SurfaceControlDevice.h>
#include <GenicamParameterEnum.h>
#include <GenicamParameterFloat.h>
#include <GenicamParameterInt.h>
#include <fstream>

using namespace ME;
using namespace std;
using PointCloudType = PointCloud<PointXYZ<float>>;
using PointCloudTypePtr = PointCloudPtr<PointXYZ<float>>;

megc_status_t status;
GenicamDeviceVec devices;
SurfaceControlDevicePtr connected_device;
PointCloudTypePtr point_cloud;
mutex mutex_waiting;
bool data_received = false;
condition_variable event_condition;


//Used to detect available devices.
void findDevices()
{
  cout << "Finding devices...\n";

  status = GenicamSystem::findDevices();
  if (status != megc_status_t::MEGC_OK)
  {
    cerr << "Error finding devices!\n";
    return;
  }

  devices = GenicamSystem::devices();
  cout << devices->size() << " device(s) found.\n";
}

//Used to connect to available devices.
void connectSurfaceControlDevice()
{
  cout << "Connecting to surfaceCONTROL device...\n";
  int count = 0;

  status = megc_status_t::MEGC_ERR_DEVICE_NOT_CONNECTED;
  for (GenicamDevicePtr device : *devices)
  {
    if (device->type() != GenicamDevice::T_SurfaceControl)
      continue;

    count++;
    status = device->connect();
    if (status == megc_status_t::MEGC_OK)
    {
      connected_device = device->toSurfaceControlDevice();
      cout << "Device connected: Name: \"" << device->modelName() << "\" SN: \"" << device->serialNumber() << "\"\n";
      return;
    }
  }

  if (count == 0)
    cerr << "No surfaceCONTROL device found!\n";
  else
    cerr << "Connecting to surfaceCONTROL device failed!\n";
}

//Sets device parameters. More available parameters could be configured, check the documentations to find the list of parameters.
void setParameters()
{
  //Set some basic parameters
  cout << "Set exposure time to 2500 microseconds\n";
  status = connected_device->setExposureTime(2500.0f);
  if (status != megc_status_t::MEGC_OK)
    return;
}

//Decides what happens when capturing exposure is done.
static
void eventExposureEndCallback(meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //Sensor exposure finished, sensor may be moved to next measurement position
  cout << "Exposure finished, data processing...\n";
}

//Decides what happens when a frame data processing is finished.
static
void eventFrameEndCallback(meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //Data processing finished, the next measurement may be triggered
  cout << "Data processing finished, transferring data...\n";
}

//Runs when an error occurs.
static
void eventErrorCallback(int error_code, std::string error_message, meg_uint64_t /*timestamp*/, void* /*user_data*/)
{
  //A sensor error has occured
  cerr << "Error " << error_code << ": " << error_message << "\n";
}

//...
static
void pointCloudCallback(PointCloudTypePtr data, void* /*user_data*/)
{
  point_cloud = data;

  //We signal the measurement loop that data has been received
  data_received = true;
  event_condition.notify_all();
}

void setupDataTransfer()
{
  cout << "Setup data transfer of 3D data\n";
  status = connected_device->setMeasurementDataEnabled();
  if (status != megc_status_t::MEGC_OK)
    return;

  cout << "Enable events\n";
  connected_device->setEventsEnabled(SurfaceControlDevice::EV_ExposureEnd | SurfaceControlDevice::EV_FrameEnd | SurfaceControlDevice::EV_Error);

  cout << "Set callback for exposure end\n";
  connected_device->setEventExposureEndCallback(eventExposureEndCallback);

  cout << "Set callback for frame end\n";
  connected_device->setEventFrameEndCallback(eventFrameEndCallback);

  cout << "Set callback for errors\n";
  connected_device->setEventErrorCallback(eventErrorCallback);

  cout << "Set callback for 3D data\n";
  connected_device->set3dDataCallback(pointCloudCallback);

  cout << "Start image acquisition (triggered mode)\n";
  status = connected_device->acquisitionStart(GenicamDevice::AcquisitionMode::AM_Continuous, true);
  if (status != megc_status_t::MEGC_OK)
    return;
}

void processData(string filename)
{
  
  const PointCloudType::Points& points = point_cloud->points();
  int pi = 0;

  if (points.count() == 0)
  {
    cerr << "Point cloud is empty!\n";
    return;
  }

  vector<float> pointsVec;
  int valid_points = 0;
  float min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0, min_z = 0.0, max_z = 0.0;
  

  for (const PointXYZ<float>& point : points)
  {
    if (PointCloudType::isValid(point)) {
      valid_points++;
      pointsVec.push_back(point.x);
      pointsVec.push_back(point.y);
      pointsVec.push_back(point.z);
    }
    else
      continue;

    if (valid_points == 1)
    {
      //Initialize values
      min_x = point.x;
      max_x = min_x;
      min_y = point.y;
      max_y = min_y;
      min_z = point.z;
      max_z = min_z;
    }
    else if (PointCloudType::isValid(point))
    {
      if (point.x < min_x)
        min_x = point.x;
      if (point.x > max_x)
        max_x = point.x;
      if (point.y < min_y)
        min_y = point.y;
      if (point.y > max_y)
        max_y = point.y;
      if (point.z < min_z)
        min_z = point.z;
      if (point.z > max_z)
        max_z = point.z;
    }
  }

  //Printing out key parameters. (Unit is in mm)
  cout << "3D data info: Valid points: " << valid_points << " | Min x: " << min_x << " | Max x: " << max_x << " | Min y: "
  << min_y << " | Max y: " << max_y << " | Min z: " << min_z << " | Max z: " << max_z << "\n";

  //Creating .pcd file using the fstream library
  ofstream out (filename);
  out << "# .PCD v.7 - Point Cloud Data file\n" << "VERSION .7\n" << "FIELDS x y z\n" << "SIZE 4 4 4\n" << "TYPE F F F\n" << "COUNT 1 1 1\n"
  << "WIDTH 1\n" << "HEIGHT " << valid_points << "\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << (valid_points) << "\nDATA ascii\n";
  for(int i = 1; i <= pointsVec.size(); i++) {
    if(i % 3 == 0) 
      out << pointsVec[i-1] << "\n";  
    
    else 
      out << pointsVec[i-1] << " ";
  }
  out.close();
} 



void measurementLoop(string filename)
{
  char key;
  do
  {
    key = '\0';
    cout << "\n\n Press [t] + [Enter] to trigger a measurement or [a] + [Enter] to abort measurement\n";
    cin >> key;
    cin.ignore();

    if (key != 't')
      return;

    cout << "Trigger measurement\n";
    data_received = false;
    status = connected_device->triggerSoftware();
    if (status != megc_status_t::MEGC_OK)
      return;

    cout << "Measuring...\n";

    //Wait until data is received, we wait a maximum of 20 seconds
    unique_lock<mutex> lk(mutex_waiting);
    if ((event_condition.wait_for(lk, std::chrono::seconds(20), [](){return data_received; })))
      processData(filename);
    else
      cerr << "Measurement timeout! No measurement data received!\n";
  }
  while (key == 't');
}

int main(int argc, char** argv)
{
  string filename = argv[0];

  status = GenicamSystem::initialize();

  if (status != megc_status_t::MEGC_OK)
    cerr << "Error initializing GenicamSystem!\n";
  else
    findDevices();

  if (status == megc_status_t::MEGC_OK)
    connectSurfaceControlDevice();

  if (status == megc_status_t::MEGC_OK)
    setParameters();

  if (status == megc_status_t::MEGC_OK)
    setupDataTransfer();

  if (status == megc_status_t::MEGC_OK)
    measurementLoop(filename);

  if (status == megc_status_t::MEGC_OK)
    connected_device->acquisitionStop();

  if (connected_device != nullptr)
    connected_device->disconnect();

  GenicamSystem::destroy();

  cout << "Press Enter to exit\n";
  cin.ignore();

  return 0;
}
