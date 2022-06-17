#pragma once

#include "PointCloudCommon.h"

#include <string>
#include <vector>
#include <map>
#include <memory>

#include "PointCloud.h"

namespace ME
{

  class PointCloudIOPrivate;

  class DLLExportMEPointCloud PointCloudIO
  {
  public:
    PointCloudIO();
    ~PointCloudIO();
    
    void setSensorName(const std::string& name);
    void setInfoString(const std::string& tag, const std::string& info_string);
    void clearInfo();
    void addComment(const std::string& comment_string);
    void clearComments();
    void setTimeStamp(long long timestamp);

    void reset();

    const std::string& sensorName() const;
    std::vector<std::string> infoTags() const;
    std::string infoString(const std::string& tag) const;
    std::vector<std::string> infoStrings() const;
    const std::vector<std::string>& comments() const;
    long long timeStamp() const;
    
    PointCloudBasePtr read(const std::string& filename);
    
    bool write(const PointCloudBase& point_cloud, const std::string& filename) const;
        
    template<typename POINT_TYPE>
    bool write(const PointCloud<POINT_TYPE>& point_cloud, const std::string& filename) const;

    PointCloudBasePtr read(const std::wstring& filename);
    
    bool write(const PointCloudBase& point_cloud, const std::wstring& filename) const;
    
    template<typename POINT_TYPE>
    bool write(const PointCloud<POINT_TYPE>& point_cloud, const std::wstring& filename) const;

  private:    
    PointCloudIOPrivate * const d_ptr;
    
  };

}