
#ifndef  ViewRemover_H__
#define  ViewRemover_H__

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>
#include <cstdlib>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include "Logger.hpp"
#include <Types/AddVector.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>

namespace Processors {
namespace ViewRemover {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoProxy;
using namespace MongoDB;

class ViewRemover: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		ViewRemover(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ViewRemover();

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
        Base::EventHandler <ViewRemover> h_readfromDB;

private:
    shared_ptr<MongoDB::View> viewPtr;
   	string hostname;
	Base::Property<string> mongoDBHost;
	Base::Property<string> viewName;
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
	Base::Property<bool> removeAll;
	auto_ptr<DBClientCursor> childCursor;
	Base::DataStreamIn<Base::UnitType> in_trigger;


	void readFromMongoDB(const string&, const string&, const string&);
	//void ReadPCDCloud(const string&, const string&);
	void readfromDB();
	void readAllFilesTriggered();
	void addToAllChilds(std::vector<OID>&);
	void cloudEncoding(OID& oid, string& tempFileName, string & cloudType);
	unsigned long hex2int(char *a, unsigned int len);
	void readRequiredData(std::vector<fileTypes> & requiredFileTypes);
};
}//: namespace ViewRemover
}//: namespace Processors

REGISTER_COMPONENT("ViewRemover", Processors::ViewRemover::ViewRemover)

#endif /* ViewRemover_H__ */
