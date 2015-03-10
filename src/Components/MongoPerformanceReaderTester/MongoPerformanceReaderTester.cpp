/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoPerformanceReaderTester.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace MongoPerformanceReaderTester  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;

MongoPerformanceReaderTester::MongoPerformanceReaderTester(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	viewName("ViewName", string("hall_small_view")),
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
	iterations("iterations", int(1)),
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false)
{
	registerProperty(type);
	registerProperty(iterations);
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

MongoPerformanceReaderTester::~MongoPerformanceReaderTester()
{
	CLOG(LTRACE) << "Good bye MongoDBImporter";
}

void MongoPerformanceReaderTester::prepareInterface() {
	CLOG(LTRACE) << "MongoDBImporter::prepareInterface";
	h_readfromDB.setup(this, &MongoPerformanceReaderTester::readfromDB);
	registerHandler("readFromDB", &h_readfromDB);
}

void MongoPerformanceReaderTester::readfromDB()
{
	int i=0;
	thread_clock::time_point ItersStart;
	ofstream myfile;
	myfile.open("readLogs100Document.csv");
	myfile<<";\n";
	thread_clock::time_point start = thread_clock::now();
	while(i<iterations*10)
	{

		ItersStart = thread_clock::now();
		int iter = (int)i;
		stringstream ss;
		ss << iter;
		string strIter = ss.str();
		MongoProxy::MongoProxy::getSingleton(hostname);
		bool exist = false;
		bool contain = false;
		if(type=="view")
		{
			string vn = string(viewName);
			vn = vn+strIter;
			CLOG(LDEBUG)<<"ViewName : " << vn;
			viewPtr = boost::shared_ptr<View>(new View(vn,hostname));
			exist = viewPtr->checkIfExist();
		}
		else
		{
			string mn = string(modelName);
			mn = mn+strIter;
			CLOG(LDEBUG)<<"ModelName : " << mn;
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
				CLOG(LDEBUG)<<"Read files from View/Model!";
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
					LOG(LDEBUG)<<"WRITE FILE!!!";
					int filesNr = viewPtr->getFilesSize();
					for (int i=0; i<filesNr; i++)
					{
						// send to output
						viewPtr->getFile(i)->readFile(false, pathToFiles, false);
					}
				}//if
				if(type=="model")
				{
					modelPtr->getAllFilesOIDS(fileOIDSVector);

					// for full required files vector, read file document and check if its type is equal
					// one of requested file types
					modelPtr->readFiles(fileOIDSVector, requiredFileTypes);
					//write to output
					LOG(LDEBUG)<<"WRITE FILE!!!";
					int filesNr = modelPtr->getFilesSize();
					for (int i=0; i<filesNr; i++)
					{
						// get type
						fileTypes ft = modelPtr->getFileType(i);
						// send to output
						// we don't want to save file to disc so 3rd argument is false
						// but we want to convert files to output buffer so 1st is set to true
						modelPtr->getFile(i)->readFile(true, pathToFiles, false);
					}
				}//else
			}
		}

			thread_clock::time_point ItersStop = thread_clock::now();
			myfile<<duration_cast<milliseconds>(ItersStop - ItersStart).count()<<";\n";

		++i;
	}//while
	thread_clock::time_point stop = thread_clock::now();
	CLOG(LERROR) << "Full time :"  << duration_cast<milliseconds>(stop - start).count() << " ms\n";
	myfile<<duration_cast<milliseconds>(stop - start).count()<<"\n;";
	myfile.close();
}

void MongoPerformanceReaderTester::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
{
	CLOG(LDEBUG)<<"ViewWriter::checkProvidedData";
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
	CLOG(LDEBUG)<<"Size of required file types: "<<requiredFileTypes.size();
}

bool MongoPerformanceReaderTester::onInit()
{
	CLOG(LTRACE) << "MongoDBImporter::initialize";
	MongoProxy::MongoProxy::getSingleton(hostname);
	return true;
}

bool MongoPerformanceReaderTester::onFinish()
{
        CLOG(LTRACE) << "MongoDBImporter::finish";
        return true;
}

bool MongoPerformanceReaderTester::onStep()
{
        CLOG(LTRACE) << "MongoDBImporter::step";
        return true;
}

bool MongoPerformanceReaderTester::onStop()
{
        return true;
}

bool MongoPerformanceReaderTester::onStart()
{
        return true;
}



} //: namespace MongoPerformanceReaderTester
} //: namespace Processors
