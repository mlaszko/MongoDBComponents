
#ifndef  ViewWriter_H__
#define  ViewWriter_H__
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <cstring>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>
#include <Types/Scene.hpp>

namespace Processors {
namespace ViewWriter {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoProxy;
using namespace MongoDB;


class ViewWriter: public Base::Component
{
public:
	/*!
	 * Constructor.
	 */
   ViewWriter(const std::string & name = "");

	/*!
	 * Destructor
	 */
	virtual ~ViewWriter();

	/*!
	 * Prepares communication interface.
	 */
	virtual void prepareInterface();

	// in_mean_viewpoint_features_number
	Base::DataStreamIn<int> in_mean_viewpoint_features_number;

   	 /// Input camera_info
   	Base::DataStreamIn <cv::string, Base::DataStreamBuffer::Newest> in_camera_info;

   	/// Input data stream -XYZ image
   	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_xyz;

   	/// Input RGB image
   	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_rgb;

  	/// Input depth image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_depth;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_intensity;

   	/// Input mask image
   	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_mask;

  	/// Input density image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_stereoL;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_stereoR;

  	/// Input density image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_stereoLTextured;

  	/// Input intensity image
  	Base::DataStreamIn <cv::Mat, Base::DataStreamBuffer::Newest> in_stereoRTextured;

  	//PCL - Point Clouds
   	/// Cloud containing points with Cartesian coordinates (XYZ).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZ>::Ptr, Base::DataStreamBuffer::Newest>  in_pc_xyz;

   	/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Base::DataStreamBuffer::Newest > in_pc_xyzrgb;

   	/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
   	Base::DataStreamIn<pcl::PointCloud<PointXYZSIFT>::Ptr, Base::DataStreamBuffer::Newest > in_pc_xyzsift;

   	/// Cloud containing points with Cartesian coordinates,  SIFT descriptor and color (XYZ + RGB + 128)
   	Base::DataStreamIn<pcl::PointCloud<PointXYZRGBSIFT>::Ptr, Base::DataStreamBuffer::Newest > in_pc_xyzrgbsift;

   	/// Cloud containing points with Cartesian coordinates and SHOT descriptor  (XYZ + SHOT).
   	Base::DataStreamIn<pcl::PointCloud<PointXYZSHOT>::Ptr, Base::DataStreamBuffer::Newest > in_pc_xyzshot;

   	/// Cloud containing points with Cartesian coordinates, colors and normals (XYZ + RGB + NORMAL).
   	Base::DataStreamIn<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr, Base::DataStreamBuffer::Newest> in_pc_xyzrgbnormal;
	shared_ptr<MongoDB::View> viewPtr;
	shared_ptr<MongoDB::Scene> scenePtr;
	string hostname;
protected:

	/*!
	 * Connects source to given device.
	 */
	bool onInit();

	/*!
	 * Disconnect source from device, closes streams, etc.
	 */
	bool onFinish();

	/*!
	 * Retrieves data from device.
	 */
	bool onStep();

	/*!
	 * Start component
	 */
	bool onStart();

	/*!
	 * Stop component
	 */
	bool onStop();


	/*!
	 * Event handler function.
	 */

	/// Event handler.


private:
	Base::Property<string> mongoDBHost;
	Base::Property<string> description;
	Base::Property<string> viewName;
	Base::Property<string> fileName;
	Base::Property<string> SensorType;
	Base::Property<string> viewsSet;
	Base::Property<string> sceneNameProp;
	Base::Property<bool> cameraInfoProp;
	Base::Property<bool> xyzProp;
	Base::Property<bool> rgbProp;
	Base::Property<bool> depthProp;
	Base::Property<bool> intensityProp;
	Base::Property<bool> maskProp;
	Base::Property<bool> stereoProp;
	Base::Property<bool> stereoTexturedProp;
	Base::Property<bool> pc_xyzProp;
	Base::Property<bool> pc_xyzrgbProp;
	Base::Property<bool> pc_xyzsiftProp;
	Base::Property<bool> pc_xyzrgbsiftProp;
	Base::Property<bool> pc_xyzshotProp;
	Base::Property<bool> pc_xyzrgbnormalProp;

//	boost::shared_ptr<std::vector<fileTypes> >requiredTypes;

	//Base::Property<int> mean_viewpoint_features_number;

	void writeData();

	void Write_xyz();
	void Write_xyzrgb();
	void Write_xyzsift();

	void test();

	void initObject();
	void cleanInputData(fileTypes & type);
	bool checkProvidedData(std::vector<fileTypes> & requiredFileTypes, bool& anyMarked);
	void writeNode2MongoDB(const string &destination, const string &option, string,  const string& fileType);
	void insert2MongoDB(const string &destination,  const string&,  const string&,  const string& fileType );
	void writeTXT2DB();
	void writeImage2DB();
	void writePCD2DB();
	void writeYAML2DB();

	void saveFile(fileTypes & fileType);
	float getFileSize(const string& fileType);
	void insertToModelOrView(const string &,const string &);
	void insertFileIntoGrid(OID&, const string&, int);
	void insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int);
	void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
	void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
	void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
	void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
	char* serializeTable( int &length, 	std::stringstream& compressedData);
};
}//: namespace ViewWriter
}//: namespace Processors

REGISTER_COMPONENT("ViewWriter", Processors::ViewWriter::ViewWriter)

#endif /* ViewWriter_H__ */
