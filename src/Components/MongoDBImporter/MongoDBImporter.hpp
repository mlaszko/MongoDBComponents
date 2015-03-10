
#ifndef  MongoDBImporter_H__
#define  MongoDBImporter_H__
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

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
#include <Types/Model.hpp>
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;
namespace Processors {
namespace MongoDBImporter {

using namespace cv;
using namespace mongo;
using namespace std;


class MongoDBImporter: public Base::Component
{
public:
        /*!
         * Constructor.
         */
	MongoDBImporter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBImporter();

        /*!
         * Prepares communication interface.
         */
        virtual void prepareInterface();

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
        Base::EventHandler <MongoDBImporter> h_readfromDB;
private:

        Base::Property<string> mongoDBHost;
		Base::Property<string> viewName;
		Base::Property<string> modelName;
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
		Base::Property<string> path;
		Base::Property<string> type;
		string dbCollectionPath;
		auto_ptr<DBClientCursor> cursorCollection;
		auto_ptr<DBClientCursor> childCursor;
		string hostname;
		shared_ptr<MongoDB::View> viewPtr;
		shared_ptr<MongoDB::Model> modelPtr;
		/// Output camera_info
		Base::DataStreamOut <cv::string> out_camera_info;

		/// Output data stream -XYZ image
		Base::DataStreamOut <cv::Mat> out_xyz;

		/// Output RGB image
		Base::DataStreamOut <cv::Mat> out_rgb;

		/// Output depth image
		Base::DataStreamOut <cv::Mat> out_depth;

		/// Output intensity image
		Base::DataStreamOut <cv::Mat> out_intensity;

		/// Output mask image
		Base::DataStreamOut <cv::Mat> out_mask;

		/// Output density image
		Base::DataStreamOut <cv::Mat> out_stereoL;

		/// Output intensity image
		Base::DataStreamOut <cv::Mat> out_stereoR;

		/// Output density image
		Base::DataStreamOut <cv::Mat> out_stereoLTextured;

		/// Input intensity image
		Base::DataStreamOut <cv::Mat> out_stereoRTextured;

		//PCL - Point Clouds
		/// Cloud containing points with Cartesian coordinates (XYZ).
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZ>::Ptr>  out_pc_xyz;

		/// Cloud containing points with Cartesian coordinates and colour (XYZ + RGB).
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_pc_xyzrgb;

		/// Cloud containing points with Cartesian coordinates and SIFT descriptor (XYZ + 128).
		Base::DataStreamOut<pcl::PointCloud<PointXYZSIFT>::Ptr> out_pc_xyzsift;

		/// Cloud containing points with Cartesian coordinates,  SIFT descriptor and color (XYZ + RGB + 128)
		Base::DataStreamOut<pcl::PointCloud<PointXYZRGBSIFT>::Ptr> out_pc_xyzrgbsift;

		/// Cloud containing points with Cartesian coordinates and SHOT descriptor  (XYZ + SHOT).
		Base::DataStreamOut<pcl::PointCloud<PointXYZSHOT>::Ptr> out_pc_xyzshot;

		/// Cloud containing points with Cartesian coordinates, colors and normals (XYZ + RGB + NORMAL).
		Base::DataStreamOut<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> out_pc_xyzrgbnormal;
		void readFromMongoDB(string&, string&, string&);
		void readfromDB();
		void readFile(const string&, const string&, const string&, const OID&);
		void getFileFromGrid(const GridFile& file, const string& modelOrViewName, const string& nodeName, const string& type);
		void run();
		void readRequiredData(std::vector<fileTypes> & requiredFileTypes);

};
}//: namespace MongoDBImporter
}//: namespace Processors

REGISTER_COMPONENT("MongoDBImporter", Processors::MongoDBImporter::MongoDBImporter)

#endif /* MongoDBImporter_H__ */
