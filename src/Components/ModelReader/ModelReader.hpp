
#ifndef  ModelReader_H__
#define  ModelReader_H__

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

#include <Types/AddVector.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/PointXYZSHOT.hpp>
#include <Types/Model.hpp>
#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>

namespace Processors {
namespace ModelReader {

using namespace cv;
using namespace mongo;
using namespace std;
using namespace MongoProxy;
using namespace MongoDB;


class ModelReader: public Base::Component, SIFTObjectModelFactory
{
public:
        /*!
         * Constructor.
         */
		ModelReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~ModelReader();

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
        Base::EventHandler <ModelReader> h_readfromDB;

private:
        Base::Property<string> mongoDBHost;
        Base::Property<string> modelName;
        Base::Property<bool> pc_xyzProp;
		Base::Property<bool> pc_xyzrgbProp;
		Base::Property<bool> pc_xyzsiftProp;
		Base::Property<bool> pc_xyzrgbsiftProp;
		Base::Property<bool> pc_xyzshotProp;
		Base::Property<bool> pc_xyzrgbnormalProp;
		shared_ptr<MongoDB::Model> modelPtr;
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZNormal;
		pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT;
		pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT;
		pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;

		string hostname;
        string nodeType;
        string dbCollectionPath;
        auto_ptr<DBClientCursor> cursorCollection;
        auto_ptr<DBClientCursor> childCursor;
    	std::string name_cloud_xyz;
    	std::string name_cloud_xyzrgb;
    	std::string name_cloud_xyzsift;
		// vector consisting all files OIDS\
		// position of allChildsVector
		/// Trigger - used for writing clouds
		Base::DataStreamIn<Base::UnitType> in_trigger;

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
		Base::DataStreamOut<std::vector<AbstractObject*> > out_models;

        void readFromMongoDB(const string&, const string&, const string&);
        void readfromDB();
        void loadModels(fileTypes fT, string& ModelName, int meanViewpointFeaturesNumber, std::vector<AbstractObject*>& models);
        void ReadPCDCloudFromFile(const string&, const string&);

        void readFile(const OID& childOID, std::vector<AbstractObject*>& models);
        void run();
        void writeToSink(string& mime, string& tempFilename, string& fileName);
        void addToAllChilds(std::vector<OID>&);
        void readAllFilesTriggered();
    	void readRequiredData(std::vector<fileTypes> & requiredFileTypes);
};
}//: namespace ModelReader
}//: namespace Processors

REGISTER_COMPONENT("ModelReader", Processors::ModelReader::ModelReader)

#endif /* ModelReader_H__ */
