#ifndef SURFACECONTROLDEVICE_H
#define SURFACECONTROLDEVICE_H

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
  class SurfaceControlDevicePrivate;

  class MEGCPP_VISIBILITY SurfaceControlDevice : public GenicamDevice
  {
  public:
    enum MultiSlopeExposureMode
    {
      MM_Off,
      MM_Soft,
      MM_Medium,
      MM_Aggressive,
      MM_Manual
    };

    enum ErroneousPointsHandling
    {
      EPH_Erase,
      EPH_StartValue
    };

    enum CoordinateAxis
    {
      CA_X,
      CA_Y,
      CA_Z
    };

    enum CoordinateTransform
    {
      CT_Translate,
      CT_Rotate
    };

    enum CameraData
    {
      CD_ImageCamera0 = 0x001,
      CD_ImageCamera1 = 0x002
    };

    enum MeasurementType
    {
      MT_Amplitude,
      MT_Gradient,
      MT_Base,
    };

    enum MeasurementData
    {
      MD_ImageAmplitude0 = 0x01,
      MD_ImageGradient0 = 0x02,
      MD_ImageBase0 = 0x04,
      MD_ImageAmplitude1 = 0x08,
      MD_ImageGradient1 = 0x10,
      MD_ImageBase1 = 0x20,
      MD_3dData = 0x40
    };

    enum Event
    {
      EV_ExposureEnd = 0x01,
      EV_FrameStart = 0x02,
      EV_FrameEnd = 0x04,
      EV_FrameTriggerMissed = 0x08,
      EV_Error = 0x10,
      EV_Warning = 0x20
    };

    enum PatternType
    {
      PT_Balanced,
      PT_HighPrecision,
      PT_HighSpeed,
      PT_Custom
    };

    ~SurfaceControlDevice();

    // Camera
    float exposureTime(megc_status_t* status = nullptr) const;
    MultiSlopeExposureMode multiSlopeExposureMode(megc_status_t* status = nullptr) const;
    float multiSlopeExposureLimit1(megc_status_t* status = nullptr) const;
    float multiSlopeExposureLimit2(megc_status_t* status = nullptr) const;

    // Measurement parameters
    float imageScale(int camera_idx, MeasurementType type, megc_status_t* status = nullptr) const;
    int imageOffset(int camera_idx, MeasurementType type, megc_status_t* status = nullptr) const;
    PatternType patternType(megc_status_t* status = nullptr) const;
    float patternWidth(megc_status_t* status = nullptr) const;
    int patternCount(megc_status_t* status = nullptr) const;
    int graycodeThreshold(megc_status_t* status = nullptr) const;
    int amplitudeThreshold(megc_status_t* status = nullptr) const;

    // 3D range parameters
    float coordinateScaleX(megc_status_t* status = nullptr) const;
    float coordinateScaleY(megc_status_t* status = nullptr) const;
    float coordinateOffsetX(megc_status_t* status = nullptr) const;
    float coordinateOffsetY(megc_status_t* status = nullptr) const;
    float lowerLimitZ(megc_status_t* status = nullptr) const;
    float upperLimitZ(megc_status_t* status = nullptr) const;
    int pointCountX(megc_status_t* status = nullptr) const;
    int pointCountY(megc_status_t* status = nullptr) const;

    // Advanced parameters
    float maximumDeviation(megc_status_t* status = nullptr) const;
    float searchStep(megc_status_t* status = nullptr) const;
    ErroneousPointsHandling erroneousPointsHandling(megc_status_t* status = nullptr) const;
    bool isPerformanceModeEnabled(megc_status_t* status = nullptr) const;

    // Light parameters
    float lightBrightness(megc_status_t* status = nullptr) const;

    // Scan 3d parameters
    float coordinateTransformValue(CoordinateTransform tr, CoordinateAxis axis, megc_status_t* status = nullptr) const;

    //Images / Data to be transferred / Events to be fired
    int cameraDataEnabled(megc_status_t* status = nullptr) const;
    int measurementDataEnabled(megc_status_t* status = nullptr) const;
    int eventsEnabled(megc_status_t* status = nullptr) const;

    // Camera
    megc_status_t setExposureTime(float value);
    megc_status_t setMultiSlopeExposureMode(MultiSlopeExposureMode mode);
    megc_status_t setMultiSlopeExposureLimit1(float limit_percentage);
    megc_status_t setMultiSlopeExposureLimit2(float limit_percentage);

    // Measurement parameters
    megc_status_t setImageScale(int camera_idx, MeasurementType type, float value);
    megc_status_t setImageOffset(int camera_idx, MeasurementType type, int value);
    megc_status_t setPatternType(PatternType type);
    megc_status_t setPatternWidth(float value);
    megc_status_t setPatternCount(int value);
    megc_status_t setGraycodeThreshold(int value);
    megc_status_t setAmplitudeThreshold(int value);
	

    // 3D range parameters
    megc_status_t setCoordinateScaleX(float value);
    megc_status_t setCoordinateScaleY(float value);
    megc_status_t setCoordinateOffsetX(float value);
    megc_status_t setCoordinateOffsetY(float value);
    megc_status_t setLowerLimitZ(float value);
    megc_status_t setUpperLimitZ(float value);
    megc_status_t setPointCountX(int value);
    megc_status_t setPointCountY(int value);

    // Advanced parameters
    megc_status_t setMaximumDeviation(float value);
    megc_status_t setSearchStep(float value);
    megc_status_t setErroneousPointsHandling(ErroneousPointsHandling value);
    megc_status_t setPerformanceModeEnabled(bool enabled);

    // Light parameters
    megc_status_t setLightBrightness(float value);

    // Scan 3d parameters
    megc_status_t setCoordinateTransformValue(CoordinateTransform tr, CoordinateAxis axis, float value); 

    //Images / Data to be transferred / Events to be fired
    megc_status_t setCameraDataEnabled(int components);
    megc_status_t setMeasurementDataEnabled(int components = MD_3dData);
    megc_status_t setEventsEnabled(int events);

    // Callbacks for receiving images / 3d data
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
    void setEventFrameTriggerMissedCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr);
    void setEventFrameTriggerMissedCallback(std::function<void(meg_uint64_t)> callback);
    void setEventWarningCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr);
    void setEventWarningCallback(std::function<void(int, std::string, meg_uint64_t)> callback);
    void setEventErrorCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr);
    void setEventErrorCallback(std::function<void(int, std::string, meg_uint64_t)> callback);

    megc_status_t connect();

    //deprecated functions, will be removed in subsequent releases
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventExposureEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventExposureEndCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventExposureEndCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventExposureEndCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameStartCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameStartCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameStartCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameStartCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameEndCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameEndCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameEndCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameEndCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameTriggerMissedCallback(void(*callback)(meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventFrameTriggerMissedCallback(void(*callback)(void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventFrameTriggerMissedCallback(std::function<void(meg_uint64_t)> callback)\" instead.") void setEventFrameTriggerMissedCallback(std::function<void()> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventWarningCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventWarningCallback(void(*callback)(int, std::string, void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventWarningCallback(std::function<void(int, std::string, meg_uint64_t)> callback)\" instead.") void setEventWarningCallback(std::function<void(int, std::string)> callback);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventErrorCallback(void(*callback)(int, std::string, meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventErrorCallback(void(*callback)(int, std::string, void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventErrorCallback(std::function<void(int, std::string, meg_uint64_t)> callback)\" instead.") void setEventErrorCallback(std::function<void(int, std::string)> callback);
    ///////////////////////

  private:
    SurfaceControlDevice();
    SurfaceControlDevice(const SurfaceControlDevice &) = delete;
    SurfaceControlDevice &operator=(const SurfaceControlDevice &) = delete;

    friend class GenicamSystemPrivate;
  };

  using SurfaceControlDevicePtr = std::shared_ptr<SurfaceControlDevice>;
  using SurfaceControlDeviceVec = std::shared_ptr<std::vector<SurfaceControlDevicePtr>>;
}

#endif //SURFACECONTROLDEVICE_H
