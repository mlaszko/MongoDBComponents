
#ifndef  MONGODBExporter_H__
#define  MONGODBExporter_H__
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
#include <Types/Scene.hpp>
#include <Types/Model.hpp>
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;
namespace Processors {
namespace MongoDBExporter {

using namespace cv;
using namespace mongo;
using namespace std;


class MongoDBExporter: public Base::Component
{
public:
        /*!
         * Constructor.
         */
	   MongoDBExporter(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~MongoDBExporter();

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
        Base::EventHandler <MongoDBExporter> h_write2DB;
private:

        Base::Property<string> mongoDBHost;
        Base::Property<string> description;
        Base::Property<string> viewsSet;
    	Base::Property<string> SensorType;
        Base::Property<string> folderName;
        Base::Property<string> sceneNameProp;
        Base::Property<int> in_mean_viewpoint_features_number;
        std::vector<std::string> fileExtensions;
        //string sceneName;
        std::vector<std::string> splitedSceneNames;
        shared_ptr<MongoDB::View> viewPtr;
        shared_ptr<MongoDB::Model> modelPtr;
    	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
    	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
    	pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT;
    	pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT;
    	string cloudType;
		cv::Mat tempImg;
		cv::Mat xyzrgbImage;
		std::string str;
		string hostname;
		shared_ptr<MongoDB::Scene> scenePtr;
		//float sizeOfCloud;


        //string dbCollectionPath;
        //void run();
        void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
        void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
        void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
        void initObject();
        float get_file_size(std::string& filename); // path to file
        void writeNode2MongoDB(const string &source, const string &destination, const string &option, string );
        void insert2MongoDB(const string &destination,  const string&,  const string& );
        void write2DB();
        void insertToModelOrView(const string &,const string &);
        vector<string> getAllFiles(const string& pattern);
        vector<string> getAllFolders(const string& directoryPath);
        void insertFileToGrid(string&, const std::vector<string>::iterator, string &, OID&, int);
        void addToObject(const Base::Property<string> & nodeNameProp, const string &);
        void writeToMemory(string& mime, string& fileName);
        void createModelOrView(const std::vector<string>::iterator, const string&, BSONArrayBuilder&);
        void insertFileIntoCollection(OID& oid, const string& fileType, string& tempFileName, int size);
        void ReadPCDCloudFromFile(const string& filename);

};
}//: namespace MongoDBExporter
}//: namespace Processors

REGISTER_COMPONENT("MongoDBExporter", Processors::MongoDBExporter::MongoDBExporter)

#endif /* MONGODBExporter_H__ */
