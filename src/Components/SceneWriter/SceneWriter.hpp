
#ifndef  SceneWriter_H__
#define  SceneWriter_H__
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
#include <Types/SIFTObjectModelFactory.hpp>
#include <Types/AddVector.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/View.hpp>
#include <Types/Scene.hpp>

namespace Processors {
namespace SceneWriter {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoDB;
using namespace MongoProxy;


class SceneWriter: public Base::Component
{
public:
        /*!
         * Constructor.
         */
	   SceneWriter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~SceneWriter();

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
        Base::EventHandler <SceneWriter> h_write2DB;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> viewName;
        Base::Property<string> description;
        Base::Property<string> sceneNameProp;
        string sceneName;
        string hostname;
        shared_ptr<MongoDB::View> viewPtr;
        shared_ptr<MongoDB::Scene> scenePtr;
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        void addSceneToView();
};
}//: namespace SceneWriter
}//: namespace Processors

REGISTER_COMPONENT("SceneWriter", Processors::SceneWriter::SceneWriter)

#endif /* SceneWriter_H__ */
