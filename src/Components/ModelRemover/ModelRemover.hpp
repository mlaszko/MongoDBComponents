
#ifndef  ModelRemover_H__
#define  ModelRemover_H__

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
#include <Types/Model.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>

namespace Processors {
namespace ModelRemover {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoProxy;
using namespace MongoDB;

class ModelRemover: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		ModelRemover(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ModelRemover();

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
        Base::EventHandler <ModelRemover> h_readfromDB;

private:
    shared_ptr<MongoDB::Model> modelPtr;
   	string hostname;
	Base::Property<string> mongoDBHost;
	Base::Property<string> modelName;
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
}//: namespace ModelRemover
}//: namespace Processors

REGISTER_COMPONENT("ModelRemover", Processors::ModelRemover::ModelRemover)

#endif /* ModelRemover_H__ */
