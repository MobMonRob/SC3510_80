#ifndef SCANCONTROLDEVICE_H
#define SCANCONTROLDEVICE_H

#if defined(__MINGW32__)
#include <_mingw.h>
#endif

#include "MEDecl.h"
#include "MEGBase.h"
#include "GenicamDevice.h"
#include <PointXYZ.h>
#include <PointCloud.h>
#include <vector>
#include <memory>

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class ScanControlDevicePrivate;

  class MEGCPP_VISIBILITY ScanControlDevice : public GenicamDevice
  {
  public:
    enum Binning
    {
      Binning1 = 1,
      Binning2 = 2,
      Binning4 = 4,
      Binning8 = 8,
    };

    enum LaserPower
    {
      LP_Off,
      LP_ReducedPower,
      LP_StandardPower,
    };

    enum PeakThresholdMode
    {
      PM_Fixed,
      PM_AmbientLightSuppression,
      PM_FullDynamic
    };

    enum TriggerActivation
    {
      TA_FallingEdge,
      TA_RisingEdge,
    };

    enum EncoderOutputMode
    {
      EM_Off,
      EM_PositionUp,
      EM_PositionDown,
      EM_Motion,
    };

    enum RegionOfInterestExtended
    {
      RE_RegionOfInterest2,
      RE_RegionOfNoInterest,
      RE_AutoExposure,
    };

    enum VideoMode
    {
      VM_None,
      VM_LowRes,
      VM_HighRes
    };

    ~ScanControlDevice();

    // Basic parameters
    float exposureTime(megc_status_t* status = nullptr) const;
    bool isAutoExposureEnabled(megc_status_t* status = nullptr) const;
    float acquisitionLineRate(megc_status_t* status = nullptr) const;
    Binning binningX(megc_status_t* status = nullptr) const;
    int profileCount(megc_status_t* status = nullptr) const;
    float profileScale(megc_status_t* status = nullptr) const;
    LaserPower laserPower(megc_status_t* status = nullptr) const;

    //Peaks
    PeakThresholdMode peakThresholdMode(megc_status_t* status = nullptr) const;
    int peakThreshold(megc_status_t* status = nullptr) const;

    // Trigger / Encoder
    bool isLineTriggerEnabled(megc_status_t* status = nullptr) const;
    TriggerActivation lineTriggerActivation(megc_status_t* status = nullptr) const;
    EncoderOutputMode encoderOutputMode(megc_status_t* status = nullptr) const;
    float encoderResolution(megc_status_t* status = nullptr) const;
    int encoderDivider(megc_status_t* status = nullptr) const;

    // Measuring Field
    megc_status_t getRoi(float* offset_x, float* offset_z, float* width, float* height);
    megc_status_t getRoiExtended(RegionOfInterestExtended type, bool* is_enabled, float* offset_x, float* offset_z, float* width, float* height);

    // Images / Data to be transferred
    VideoMode cameraDataEnabled(megc_status_t* status = nullptr) const;
    int measurementDataEnabled(megc_status_t* status = nullptr) const;

    // Basic parameters
    megc_status_t setExposureTime(float value);
    megc_status_t setAutoExposureEnabled(bool enabled);
    megc_status_t setAcquisitionLineRate(float value);
    megc_status_t setBinningX(Binning binning);
    megc_status_t setProfileCount(int count);
    megc_status_t setProfileScale(float scale);
    megc_status_t setLaserPower(LaserPower power);

    //Peaks
    megc_status_t setPeakThresholdMode(PeakThresholdMode mode);
    megc_status_t setPeakThreshold(int threshold);

    // Trigger / Encoder
    megc_status_t setLineTriggerEnabled(bool enabled);
    megc_status_t setLineTriggerActivation(TriggerActivation activation);
    megc_status_t setEncoderOutputMode(EncoderOutputMode mode);
    megc_status_t setEncoderResolution(float resolution);
    megc_status_t setEncoderDivider(int value);

    // Measuring Field
    megc_status_t setRoi(float offset_x, float offset_z, float width, float height);
    megc_status_t setRoiExtended(RegionOfInterestExtended type, bool enable, float offset_x, float offset_z, float width, float height);

    // Images / Data to be transferred
    megc_status_t setCameraDataEnabled(VideoMode mode);
    megc_status_t setMeasurementDataEnabled(int _3d_mode_enabled = 1);

    // Callbacks for receiving images / 3d data
    void setCameraImageCallback(void(*callback)(const meg_image_t*, void *), void *userdata = nullptr);
    void setCameraImageCallback(std::function<void(const meg_image_t*)> callback);
    void set3dDataCallback(void(*callback)(const PointCloudPtr<PointXYZ<float>> point_cloud, void *), void *userdata = nullptr);
    void set3dDataCallback(std::function<void(const PointCloudPtr<PointXYZ<float>>)> callback);

    megc_status_t connect();

  private:
    ScanControlDevice();
    ScanControlDevice(const ScanControlDevice &) = delete;
    ScanControlDevice &operator=(const ScanControlDevice &) = delete;

    friend class GenicamSystemPrivate;
  };

  using ScanControlDevicePtr = std::shared_ptr<ScanControlDevice>;
  using ScanControlDeviceVec = std::shared_ptr<std::vector<ScanControlDevicePtr>>;
}

#endif //SCANCONTROLDEVICE_H
