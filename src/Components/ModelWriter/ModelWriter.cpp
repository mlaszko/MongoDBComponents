/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ModelWriter.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace ModelWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;

ModelWriter::ModelWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		description("description", string("My green coffe cup")),
		modelName("modelName", string("model1")),
		fileName("fileName", string("tempFile")),
		viewsSet("viewsSet", string("viewsSet1")),
		pc_xyzProp("PC.xyz", true),
		pc_xyzrgbProp("PC.xyzrgb", true),
		pc_xyzsiftProp("PC.xyzsift", false),
		pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
		pc_xyzshotProp("PC.xyzshot", false),
		pc_xyzrgbnormalProp("PC.xyzrgbnormal", false)
{
	registerProperty(mongoDBHost);
	registerProperty(description);
	registerProperty(modelName);
	registerProperty(fileName);
	registerProperty(viewsSet);

	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);

	//registerProperty(mean_viewpoint_features_number);

	CLOG(LTRACE) << "Hello ModelWriter";
	hostname = mongoDBHost;
}

ModelWriter::~ModelWriter()
{
	//delete(modelPtr);
    CLOG(LTRACE) << "Good bye ModelWriter";
}

void ModelWriter::prepareInterface() {
	CLOG(LTRACE) << "ModelWriter::prepareInterface";
	registerHandler("writeData", boost::bind(&ModelWriter::writeData, this));
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);
	registerStream("in_pc_xyz", &in_pc_xyz);
	registerStream("in_pc_xyzrgb", &in_pc_xyzrgb);
	registerStream("in_pc_xyzsift", &in_pc_xyzsift);
	registerStream("in_pc_xyzrgbsift", &in_pc_xyzrgbsift);
	registerStream("in_pc_xyzshot", &in_pc_xyzshot);
	registerStream("in_pc_xyzrgnormal", &in_pc_xyzrgbnormal);

	// adding dependency
	addDependency("writeData", NULL);
}

void ModelWriter::writeData()
{
	CLOG(LNOTICE) << "ModelWriter::writeData";
	MongoProxy::MongoProxy::getSingleton(hostname);
	string vn = string(modelName);
	modelPtr = boost::shared_ptr<Model>(new Model(vn,hostname));

	bool exist = modelPtr->checkIfExist();
	if(!exist)
	{
		string viewsSetList = viewsSet;
		modelPtr->setViewsSetNames(viewsSetList);
	}
	else
	{
		CLOG(LERROR)<<"Model exist in data base!!!";
		return;
	}
	std::vector<fileTypes> providedFileTypes;
	bool anyMarked = false;
	bool cleanData = checkProvidedData(providedFileTypes, anyMarked);
	if(anyMarked)
	{
		if(cleanData)
		{
			CLOG(LNOTICE)<<"Clean all data";
			for(std::vector<fileTypes>::iterator it = providedFileTypes.begin(); it != providedFileTypes.end(); ++it)
			{
				cleanInputData(*it);
			}
		}
		else if(!cleanData)
		{
			CLOG(LNOTICE)<<"Save all data";

			// save model in mongo
			modelPtr->create();

			// save all files from input to mongo
			for(std::vector<fileTypes>::iterator it = providedFileTypes.begin(); it != providedFileTypes.end(); ++it)
			{
				saveFile(*it);
			}
		}
	}
	else
	{
		CLOG(LERROR)<<"Please mark any checkbox";
	}
}

void ModelWriter::cleanInputData(fileTypes & type)
{
	CLOG(LNOTICE)<<"ViewWriter::cleanInputData";
	switch(type)
	{
		case PCXyz:
		{
			in_pc_xyz.read();
			break;
		}
		case PCXyzRgb:
		{
			in_pc_xyzrgb.read();
			break;
		}
		case PCXyzSift:
		{
			in_pc_xyzsift.read();
			break;
		}
		case PCXyzRgbSift:
		{
			in_pc_xyzrgbsift.read();
			break;
		}
		case PCXyzShot:
		{
			in_pc_xyzshot.read();
			break;
		}
		case PCXyzRgbNormal:
		{
			in_pc_xyzrgbnormal.read();
			break;
		}
		default:
			break;
	}
	return;
}

void ModelWriter::saveFile(fileTypes & fileType)
{
	CLOG(LNOTICE)<<"ViewWriter::saveFile";

	string filename = (string)fileName;
	string ModelName = modelName;
	string type = "Model";
	string empty = "";
	switch(fileType)
	{
		case PCXyz:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyz.read(), fileType, filename, ModelName, hostname));
			modelPtr->addFile(file, type, true, empty,0);
			break;
		}
		case PCXyzRgb:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgb.read(), fileType, filename, ModelName, hostname));
			modelPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzSift:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzsift.read(), fileType, filename, ModelName, hostname));
			int mean_viewpoint_features_number = in_mean_viewpoint_features_number.read();
			modelPtr->addFile(file, type, true, empty, mean_viewpoint_features_number);
			break;
		}
		case PCXyzRgbSift:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgbsift.read(), fileType, filename, ModelName, hostname));
			modelPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzShot:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzshot.read(), fileType, filename, ModelName, hostname));
			modelPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzRgbNormal:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgbnormal.read(), fileType, filename, ModelName, hostname));
			modelPtr->addFile(file, type, true, empty, 0);
			break;
		}
		default:
			break;
	}
}

bool ModelWriter::checkProvidedData(std::vector<fileTypes> & requiredFileTypes, bool& anyMarked)
{
	CLOG(LNOTICE)<<"ModelWriter::checkProvidedData";
	bool cleanBuffers = false;

	if(pc_xyzProp==true)
	{
		anyMarked=true;
		if(in_pc_xyz.fresh())
			requiredFileTypes.push_back(PCXyz);
		else
			cleanBuffers = true;
	}
	if(pc_xyzrgbProp==true)
	{
		anyMarked=true;
		if(in_pc_xyzrgb.fresh())
			requiredFileTypes.push_back(PCXyzRgb);
		else
			cleanBuffers = true;
	}
	if(pc_xyzsiftProp==true)
	{
		anyMarked=true;
		if(in_pc_xyzsift.fresh())
			requiredFileTypes.push_back(PCXyzSift);
		else
			cleanBuffers = true;
	}
	if(pc_xyzrgbsiftProp==true)
	{
		anyMarked=true;
		if(in_pc_xyzrgbsift.fresh())
			requiredFileTypes.push_back(PCXyzRgbSift);
		else
			cleanBuffers = true;
	}
	if(pc_xyzshotProp==true)
	{
		anyMarked=true;
		if(in_pc_xyzshot.fresh())
			requiredFileTypes.push_back(PCXyzShot);
		else
			cleanBuffers = true;
	}
	if(pc_xyzrgbnormalProp==true)
	{
		anyMarked=true;
		if(in_pc_xyzrgbnormal.fresh())
			requiredFileTypes.push_back(PCXyzRgbNormal);
		else
			cleanBuffers = true;
	}
	CLOG(LNOTICE)<<"Size of required file types: "<<requiredFileTypes.size();
	return cleanBuffers;
}
bool ModelWriter::onInit()
{
	CLOG(LTRACE) << "ModelWriter::initialize";
	return true;
}

bool ModelWriter::onFinish()
{
        CLOG(LTRACE) << "ModelWriter::finish";

        return true;
}

bool ModelWriter::onStep()
{
        CLOG(LTRACE) << "ModelWriter::step";
        return true;
}

bool ModelWriter::onStop()
{
        return true;
}

bool ModelWriter::onStart()
{
        return true;
}

} //: namespace ModelWriter
} //: namespace Processors
