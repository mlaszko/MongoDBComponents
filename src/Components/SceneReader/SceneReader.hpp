
#ifndef  SceneReader_H__
#define  SceneReader_H__

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

#include "mongo/client/dbclient.h"
#include "mongo/bson/bson.h"
#include "Logger.hpp"
#include <Types/AddVector.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>
#include <Types/Scene.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>

namespace Processors {
namespace SceneReader {

using namespace cv;
using namespace mongo;


class SceneReader: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		SceneReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~SceneReader();

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


private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> sceneName;
        Base::Property<string> viewName;
        Base::Property<string> collectionName;
        Base::Property<bool> getViewFlag;
        Base::Property<bool> getScenesFromViewFlag;
        boost::shared_ptr<MongoDB::View> viewPtr;
        boost::shared_ptr<MongoDB::Scene> scenePtr;
        string hostname;
        string sn;

		string dbCollectionPath;
		auto_ptr<DBClientCursor> cursorCollection;
		auto_ptr<DBClientCursor> childCursor;

        void getView();
        void getScenes();
        void readfromDB();



};
}//: namespace SceneReader
}//: namespace Processors

REGISTER_COMPONENT("SceneReader", Processors::SceneReader::SceneReader)

#endif /* SceneReader_H__ */
