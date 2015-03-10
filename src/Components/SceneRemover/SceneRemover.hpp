
#ifndef  SceneRemover_H__
#define  SceneRemover_H__

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <cstdlib>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Types/PointXYZSIFT.hpp>

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include "Logger.hpp"

#include <Types/SIFTObjectModelFactory.hpp>
#include <Types/AddVector.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>
#include <Types/Scene.hpp>

namespace Processors {
namespace SceneRemover {

using namespace cv;
using namespace mongo;


class SceneRemover: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		SceneRemover(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~SceneRemover();

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
        Base::EventHandler <SceneRemover> h_readfromDB;

private:
        string hostname;
        Base::Property<string> sceneName;
        Base::Property<string> mongoDBHost;
        string dbCollectionPath;
        auto_ptr<DBClientCursor> cursorCollection;
        auto_ptr<DBClientCursor> childCursor;
    	std::string name_cloud_xyz;
    	std::string name_cloud_xyzrgb;
    	std::string name_cloud_xyzsift;
		// vector consisting all files OIDS
		std::vector<OID> allChildsVector;
		// position of allChildsVector
		/// Trigger - used for writing clouds
		Base::DataStreamIn<Base::UnitType> in_trigger;

        void removeSceneFromMongoDB();
        void readfromDB();
};
}//: namespace SceneRemover
}//: namespace Processors

REGISTER_COMPONENT("SceneRemover", Processors::SceneRemover::SceneRemover)

#endif /* SceneRemover_H__ */
