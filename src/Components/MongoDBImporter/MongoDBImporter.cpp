/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBImporter.hpp"

namespace Processors {
namespace MongoDBImporter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;

MongoDBImporter::MongoDBImporter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	viewName("ViewName", string("lab012_View")),
	modelName("ModelName", string("model1")),
	type("type", string("view")),//"model"
	path("Path", string("/home/lzmuda/DCL/MongoDB/test/")),
	cameraInfoProp("file.cameraInfo.xml", false),
	xyzProp("image.xyz", false),
	rgbProp("image.rgb", false),
	depthProp("image.density", false),
	intensityProp("image.intensity", false),
	maskProp("image.mask", false),
	stereoProp("image.stereo", false),
	stereoTexturedProp("image.stereoTextured", false),
	pc_xyzProp("PC.xyz", false),
	pc_xyzrgbProp("PC.xyzrgb", false),
	pc_xyzsiftProp("PC.xyzsift", false),
	pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
	pc_xyzshotProp("PC.xyzshot", false),
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false)
{
	registerProperty(type);
	registerProperty(mongoDBHost);
	registerProperty(viewName);
	registerProperty(modelName);
	registerProperty(cameraInfoProp);
	registerProperty(xyzProp);
	registerProperty(rgbProp);
	registerProperty(depthProp);
	registerProperty(intensityProp);
	registerProperty(maskProp);
	registerProperty(stereoProp);
	registerProperty(stereoTexturedProp);
	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);
	registerProperty(path);
	hostname = mongoDBHost;

	CLOG(LTRACE) << "Hello MongoDBExporter";
}

MongoDBImporter::~MongoDBImporter()
{
	CLOG(LTRACE) << "Good bye MongoDBImporter";
}


void MongoDBImporter::prepareInterface() {
	CLOG(LTRACE) << "MongoDBImporter::prepareInterface";
	h_readfromDB.setup(this, &MongoDBImporter::readfromDB);
	registerHandler("readFromDB", &h_readfromDB);
}

void MongoDBImporter::readfromDB()
{
	CLOG(LNOTICE) << "MongoDBImporter::readfromDB";
	MongoProxy::MongoProxy::getSingleton(hostname);
	bool exist = false;
	bool contain = false;
	if(type=="view")
	{
		string vn = string(viewName);
		viewPtr = boost::shared_ptr<View>(new View(vn,hostname));
		exist = viewPtr->checkIfExist();
	}
	else
	{
		string mn = string(modelName);
		modelPtr = boost::shared_ptr<Model>(new Model(mn,hostname));
		exist = modelPtr->checkIfExist();
	}

	if(!exist)
	{
		CLOG(LERROR)<<"View/Model doesn't exist in data base!!!, Change view/model name";
		return;
	}
	else
	{
		// get document
		if(type=="view")
			viewPtr->readViewDocument();
		else
			modelPtr->readModelDocument();
		// read all required types from GUI
		std::vector<fileTypes> requiredFileTypes;
		readRequiredData(requiredFileTypes);

		if(requiredFileTypes.size()==0)
		{
			CLOG(LERROR)<<"Please mark any checkbox";
			return;
		}
		// check if contain all required types
		if(type=="view")
			contain = viewPtr->checkIfContain(requiredFileTypes);
		else
			contain = modelPtr->checkIfContain(requiredFileTypes);

		if(!contain)
		{
			CLOG(LERROR)<<"View/Model doesn't contain all required files! BYE!";
		}
		else
		{
			CLOG(LNOTICE)<<"Read files from View/Model!";

			// read vector of files OIDs
			vector<OID> fileOIDSVector;

			string pathToFiles = path;
			if(type=="view")
			{
				viewPtr->getAllFilesOIDS(fileOIDSVector);

				// for full required files vector, read file document and check if its type is equal
				// one of requested file types

				viewPtr->getRequiredFiles(requiredFileTypes);

				//write to output
				LOG(LNOTICE)<<"WRITE FILE!!!";
				int filesNr = viewPtr->getFilesSize();
				for (int i=0; i<filesNr; i++)
				{
					// send to output
					viewPtr->getFile(i)->readFile(false, pathToFiles, true);
				}
			}//if
			if(type=="model")
			{
				modelPtr->getAllFilesOIDS(fileOIDSVector);

				// for full required files vector, read file document and check if its type is equal
				// one of requested file types

				modelPtr->readFiles(fileOIDSVector, requiredFileTypes);

				//write to output
				LOG(LNOTICE)<<"WRITE FILE!!!";
				int filesNr = modelPtr->getFilesSize();
				for (int i=0; i<filesNr; i++)
				{
					// get type
					fileTypes ft = modelPtr->getFileType(i);
					// send to output
					modelPtr->getFile(i)->readFile(false, pathToFiles, true);
				}
			}//else

		}
	}
}

void MongoDBImporter::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
{
	CLOG(LNOTICE)<<"ViewWriter::checkProvidedData";
	bool cleanBuffers = false;

	if(cameraInfoProp==true)
	{
		requiredFileTypes.push_back(FileCameraInfo);
	}
	if(xyzProp==true)
	{
		requiredFileTypes.push_back(ImageXyz);
	}
	if(rgbProp==true)
	{
		requiredFileTypes.push_back(ImageRgb);
	}
	if(depthProp==true)
	{
		requiredFileTypes.push_back(ImageDepth);
	}
	if(intensityProp==true)
	{
		requiredFileTypes.push_back(ImageIntensity);
	}
	if(maskProp==true)
	{
		requiredFileTypes.push_back(ImageMask);
	}
	if(stereoProp==true)
	{
		requiredFileTypes.push_back(StereoLeft);
		requiredFileTypes.push_back(StereoRight);
	}
	if(stereoTexturedProp==true)
	{
		requiredFileTypes.push_back(StereoLeft);
		requiredFileTypes.push_back(StereoRight);
		requiredFileTypes.push_back(StereoLeftTextured);
		requiredFileTypes.push_back(StereoRightTextured);
	}
	if(pc_xyzProp==true)
	{
		requiredFileTypes.push_back(PCXyz);
	}
	if(pc_xyzrgbProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgb);
	}
	if(pc_xyzsiftProp==true)
	{
		requiredFileTypes.push_back(PCXyzSift);
	}
	if(pc_xyzrgbsiftProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgbSift);
	}
	if(pc_xyzshotProp==true)
	{
		requiredFileTypes.push_back(PCXyzShot);
	}
	if(pc_xyzrgbnormalProp==true)
	{
		requiredFileTypes.push_back(PCXyzRgbNormal);
	}
	CLOG(LNOTICE)<<"Size of required file types: "<<requiredFileTypes.size();
}

bool MongoDBImporter::onInit()
{
	CLOG(LTRACE) << "MongoDBImporter::initialize";
	MongoProxy::MongoProxy::getSingleton(hostname);
	return true;
}

bool MongoDBImporter::onFinish()
{
        CLOG(LTRACE) << "MongoDBImporter::finish";
        return true;
}

bool MongoDBImporter::onStep()
{
        CLOG(LTRACE) << "MongoDBImporter::step";
        return true;
}

bool MongoDBImporter::onStop()
{
        return true;
}

bool MongoDBImporter::onStart()
{
        return true;
}



} //: namespace MongoDBImporter
} //: namespace Processors
