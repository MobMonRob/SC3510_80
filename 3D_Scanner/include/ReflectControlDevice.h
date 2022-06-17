#pragma once

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
  class ReflectControlDevicePrivate;

  class MEGCPP_VISIBILITY ReflectControlDevice : public GenicamDevice
  {
  public:
    enum MeasurementType
    {
      MT_Amplitude,
      MT_Curvature,
      MT_Base,
    };

    enum CameraData
    {
      CD_ImageCamera0 = 0x001,
      CD_ImageCamera1 = 0x002
    };

    enum MeasurementData
    {
      MD_ImageAmplitude0 = 0x01,
      MD_ImageCurvature0 = 0x02,
      MD_ImageBase0 = 0x04,
      MD_ImageAmplitude1 = 0x08,
      MD_ImageCurvature1 = 0x10,
      MD_ImageBase1 = 0x20,
      MD_3dData = 0x40
    };

    enum Event
    {
      EV_ExposureEnd = 0x01,
      EV_FrameStart = 0x02,
      EV_FrameEnd = 0x04,
      EV_FrameTriggerMissed = 0x08,
      EV_Error = 0x10
    };

    enum TrendRemoval
    {
      TR_None,
      TR_Plane,
      TR_Custom
    };

    enum PatternType
    {
      PT_Balanced,
      PT_HighPrecision,
      PT_HighSpeed,
      PT_Custom
    };

    enum PatternDisplay
    {
      PD_Bright,
      PD_Medium,
      PD_Dark,
      PD_Pattern
    };

    enum ReduceMask
    {
      RM_Off,
      RM_3x3,
      RM_5x5,
      RM_7x7
    };

    enum MultiAreaMode
    {
      MC_OneAreaMode,
      MC_MultiAreaMode,
    };

    enum MeasurementMode
    {
      MM_Measurement,
      MM_SensorReferencing
    };

    enum ReferencingMode
    {
      RM_Planar,
      RM_Nonplanar
    };

    ~ReflectControlDevice();

    //Camera
    float exposureTime(megc_status_t* status = nullptr) const;
    float gain(megc_status_t* status = nullptr) const;
    int binning(megc_status_t* status = nullptr) const;

    //Region of interest
    int roiWidth(int camera_idx, megc_status_t* status = nullptr) const;
    int roiHeight(int camera_idx, megc_status_t* status = nullptr) const;
    int roiOffsetX(int camera_idx, megc_status_t* status = nullptr) const;
    int roiOffsetY(int camera_idx, megc_status_t* status = nullptr) const;

    //Measurement parameters
    float imageScale(int camera_idx, MeasurementType type, megc_status_t* status = nullptr) const;
    int imageOffset(int camera_idx, MeasurementType type, megc_status_t* status = nullptr) const;
    PatternType patternType(megc_status_t* status = nullptr) const;
    float patternWidth(megc_status_t* status = nullptr) const;
    int patternCount(megc_status_t* status = nullptr) const;
    int amplitudeThreshold(megc_status_t* status = nullptr) const;

    // 3D range parameters
    float coordinateScale(megc_status_t* status = nullptr) const;
    float coordinateOffsetX(megc_status_t* status = nullptr) const;
    float coordinateOffsetY(megc_status_t* status = nullptr) const;
    int pointCountX(megc_status_t* status = nullptr) const;
    int pointCountY(megc_status_t* status = nullptr) const;

    //Advanced 3D control
    TrendRemoval trendRemoval(megc_status_t* status = nullptr) const;
    int trendRemovalCustomX(megc_status_t* status = nullptr) const;
    int trendRemovalCustomY(megc_status_t* status = nullptr) const;
    ReduceMask reduceMask(megc_status_t* status = nullptr) const;
    int reconstructionGridSize(megc_status_t* status = nullptr) const;
    MultiAreaMode multiAreaMode(megc_status_t* status = nullptr) const;
    int minAreaSize(megc_status_t* status = nullptr) const;

    //Maintenance
    PatternDisplay patternDisplay(megc_status_t* status = nullptr) const;

    //Images / Data to be transferred / Events to be fired
    int cameraDataEnabled(megc_status_t* status = nullptr) const;
    int measurementDataEnabled(megc_status_t* status = nullptr) const;
    int eventsEnabled(megc_status_t* status = nullptr) const;

    //SensorCheck / SensorReferencing
    MeasurementMode measurementMode(megc_status_t* status = nullptr) const;
    ReferencingMode referencingMode(megc_status_t* status = nullptr) const;
    meg_bool_t isReferencingActive(megc_status_t* status = nullptr) const;
    meg_bool_t isReferencingContourVisible(megc_status_t* status = nullptr) const;
    int referencingBinning(megc_status_t* status = nullptr) const;
    std::string referencingTargetID(megc_status_t* status = nullptr) const;
    meg_bool_t isReferencingValid(megc_status_t* status = nullptr) const;

    //Camera
    megc_status_t setExposureTime(float value);
    megc_status_t setGain(float value);
    megc_status_t setBinning(int value);

    //Region of interest
    megc_status_t setRoiWidth(int camera_idx, int value);
    megc_status_t setRoiHeight(int camera_idx, int value);
    megc_status_t setRoiOffsetX(int camera_idx, int value);
    megc_status_t setRoiOffsetY(int camera_idx, int value);
    megc_status_t setRoi(int camera_idx, int offset_x, int offset_y, int width, int height);
    megc_status_t resetRoi(int camera_idx);

    //Measurement parmeters
    megc_status_t setImageScale(int camera_idx, MeasurementType type, float value);
    megc_status_t setImageOffset(int camera_idx, MeasurementType type, int value);
    megc_status_t setPatternType(PatternType type);
    megc_status_t setPatternWidth(float value);
    megc_status_t setPatternCount(int value);
    megc_status_t setAmplitudeThreshold(int value);

    // 3D range parameters
    megc_status_t setCoordinateScale(float value);
    megc_status_t setCoordinateOffsetX(float value);
    megc_status_t setCoordinateOffsetY(float value);
    megc_status_t setPointCountX(int value);
    megc_status_t setPointCountY(int value);

    //Advanced 3D control
    megc_status_t setTrendRemoval(TrendRemoval value);
    megc_status_t setTrendRemovalCustomX(int value);
    megc_status_t setTrendRemovalCustomY(int value);
    megc_status_t setReduceMask(ReduceMask value);
    megc_status_t setReconstructionGridSize(int value);
    megc_status_t setMultiAreaMode(MultiAreaMode value);
    megc_status_t setMinAreaSize(int value);

    //Maintenance
    megc_status_t setPatternDisplay(PatternDisplay value);

    //Images / Data to be transferred / Events to be fired
    megc_status_t setCameraDataEnabled(int components);
    megc_status_t setMeasurementDataEnabled(int components = MD_3dData);
    megc_status_t setEventsEnabled(int events);

    //SensorCheck / SensorReferencing
    megc_status_t setMeasurementMode(MeasurementMode mode);
    megc_status_t setReferencingMode(ReferencingMode mode);
    megc_status_t setReferencingActive(meg_bool_t active);
    megc_status_t setReferencingContourVisible(meg_bool_t visible);

    //Callbacks for receiving images / 3d data
    void setCameraImageCallback(void(*callback)(const std::vector<meg_image_t*>&, const std::vector<CameraData>&, void *), void *userdata = nullptr);
    void setCameraImageCallback(std::function<void(const std::vector<meg_image_t*>&, const std::vector<CameraData>&)> callback);
    void setMeasurementImageCallback(void(*callback)(const std::vector<meg_image_t*>&, const std::vector<MeasurementData>&, void *), void *userdata = nullptr);
    void setMeasurementImageCallback(std::function<void(const std::vector<meg_image_t*>&, const std::vector<MeasurementData>&)> callback);
    void set3dDataCallback(void(*callback)(const PointCloudPtr<PointXYZ<float>> point_cloud, void *), void *userdata = nullptr);
    void set3dDataCallback(std::function<void(const PointCloudPtr<PointXYZ<float>>)> callback);

    //Callbacks for events
    void setEventExposureEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr);
    void setEventExposureEndCallback(std::function<void(meg_uint64_t)> callback);
    void setEventFrameStartCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr);
    void setEventFrameStartCallback(std::function<void(meg_uint64_t)> callback);
    void setEventFrameEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr);
    void setEventFrameEndCallback(std::function<void(meg_uint64_t)> callback);
    void setEventErrorCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr);
    void setEventErrorCallback(std::function<void(int, std::string, meg_uint64_t)> callback);
    void setEventFrameTriggerMissedCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr);
    void setEventFrameTriggerMissedCallback(std::function<void(meg_uint64_t)> callback);

    megc_status_t connect();

    //deprecated functions, will be removed in subsequent releases
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventExposureEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventExposureEndCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventExposureEndCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventExposureEndCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameStartCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameStartCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameStartCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameStartCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameEndCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameEndCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameEndCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventErrorCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventErrorCallback(void(*callback)(int, std::string, void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventErrorCallback(std::function<void(int, std::string, meg_uint64_t)> callback)\" instead.") void setEventErrorCallback(std::function<void(int, std::string)> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameTriggerMissedCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameTriggerMissedCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameTriggerMissedCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameTriggerMissedCallback(std::function<void()> callback);
    ///////////////////////

  private:
    ReflectControlDevice();
    ReflectControlDevice(const ReflectControlDevice &) = delete;
    ReflectControlDevice &operator=(const ReflectControlDevice &) = delete;

    friend class GenicamSystemPrivate;
  };

  using ReflectControlDevicePtr = std::shared_ptr<ReflectControlDevice>;
  using ReflectControlDeviceVec = std::shared_ptr<std::vector<ReflectControlDevicePtr>>;
}

