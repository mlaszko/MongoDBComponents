/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewWriter.hpp"
#include <pcl/point_representation.h>
#include <Eigen/Core>
#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/serialization.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/assume_abstract.hpp>

namespace Processors {
namespace ViewWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;
using namespace Eigen;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;

ViewWriter::ViewWriter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	description("description", string("My green coffe cup")),
	viewName("viewName", string("lab012")),
	fileName("fileName", string("tempFile")),
	SensorType("SensorType", string("Stereo")),
	viewsSet("viewsSet", string("viewsSet1")),
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
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false),
	sceneNameProp("sceneNamesProp", string("scene1"))

	// for sift cloud:
	//mean_viewpoint_features_number("mean_viewpoint_features_number", int(12)),
{
	registerProperty(mongoDBHost);
	registerProperty(description);
	registerProperty(viewName);
	registerProperty(fileName);
	registerProperty(viewsSet);
	registerProperty(SensorType);

	registerProperty(cameraInfoProp);
	registerProperty(xyzProp);
	registerProperty(rgbProp);
	registerProperty(depthProp);
	registerProperty(intensityProp);
	registerProperty(maskProp);
	registerProperty(stereoTexturedProp);
	registerProperty(stereoProp);

	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);

	registerProperty(sceneNameProp);

	//registerProperty(mean_viewpoint_features_number);

	CLOG(LTRACE) << "Hello ViewWriter";
//	requiredTypes = boost::shared_ptr<std::vector<keyTypes> >(new std::vector<keyTypes>());

	hostname = mongoDBHost;

}

ViewWriter::~ViewWriter()
{
	CLOG(LTRACE) << "Good bye ViewWriter";
}

void ViewWriter::prepareInterface() {
	CLOG(LTRACE) << "ViewWriter::prepareInterface";
	registerHandler("writeData", boost::bind(&ViewWriter::writeData, this));

	// streams registration
	registerStream("in_mean_viewpoint_features_number", &in_mean_viewpoint_features_number);
	registerStream("in_camera_info", &in_camera_info);
	registerStream("in_xyz", &in_xyz);
	registerStream("in_rgb", &in_rgb);
	registerStream("in_density", &in_depth);
	registerStream("in_intensity", &in_intensity);
	registerStream("in_mask", &in_mask);
	registerStream("in_stereoL", &in_stereoL);
	registerStream("in_stereoR", &in_stereoR);
	registerStream("in_stereoLTextured", &in_stereoLTextured);
	registerStream("in_stereoRTextured", &in_stereoRTextured);

	// PCL
	registerStream("in_pc_xyz", &in_pc_xyz);
	registerStream("in_pc_xyzrgb", &in_pc_xyzrgb);
	registerStream("in_pc_xyzsift", &in_pc_xyzsift);
	registerStream("in_pc_xyzrgbsift", &in_pc_xyzrgbsift);
	registerStream("in_pc_xyzshot", &in_pc_xyzshot);
	registerStream("in_pc_xyzrgnormal", &in_pc_xyzrgbnormal);

	// adding dependency
	addDependency("writeData", NULL);
}

void ViewWriter::writeData()
{
	CLOG(LNOTICE) << "ViewWriter::writeData";
	MongoProxy::MongoProxy::getSingleton(hostname);
	string vn = string(viewName);
	viewPtr = boost::shared_ptr<View>(new View(vn,hostname));


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
			// save view in mongo
			bool exist = viewPtr->checkIfExist();
			//TODO sprawdzać czy istnieje viewsSet
			if(!exist)
			{
				string viewsSetList = viewsSet;
				string sceneName = sceneNameProp;
				string sensor = SensorType;
				std::vector<std::string> splitedViewsSetNames;
				boost::split(splitedViewsSetNames, viewsSetList, is_any_of(";"));
				viewPtr->setSensorType(sensor);
				viewPtr->setViewsSetNames(splitedViewsSetNames);
				scenePtr = boost::shared_ptr<Scene>(new Scene(sceneName,hostname));
				OID sceneOID;
				scenePtr->create(sceneOID);
				OID viewOID;
				viewPtr->create(sceneOID, viewOID, sceneName);
				scenePtr->addView(vn, viewOID);
			}
			else
			{
				CLOG(LERROR)<<"View exist in data base!!!";
				return;
			}
			//viewPtr->create();

			// save all files from input to mongo
			for(std::vector<fileTypes>::iterator it = providedFileTypes.begin(); it != providedFileTypes.end(); ++it)
			{
				saveFile(*it);
			}
			//viewPtr->saveAllFiles();
		}
	}
	else
	{
		CLOG(LERROR)<<"Please mark any checkbox";
	}
}

bool ViewWriter::onInit()
{
	CLOG(LTRACE) << "ViewWriter::initialize";

	return true;
}

bool ViewWriter::onFinish()
{
	CLOG(LTRACE) << "ViewWriter::finish";

	return true;
}

bool ViewWriter::onStep()
{
	CLOG(LTRACE) << "ViewWriter::step";
	return true;
}

bool ViewWriter::onStop()
{
	return true;
}

bool ViewWriter::onStart()
{
	return true;
}

void ViewWriter::cleanInputData(fileTypes & type)
{
	CLOG(LNOTICE)<<"ViewWriter::cleanInputData";
	switch(type)
	{
		case FileCameraInfo:
		{
			in_camera_info.read();
			break;
		}
		case ImageRgb:
		{
			CLOG(LNOTICE)<<"Clean from in_rgb";
			in_rgb.read();
			break;
		}
		case ImageXyz:
		{
			in_xyz.read();
			break;
		}
		case ImageDepth:
		{
			in_depth.read();
			break;
		}
		case ImageIntensity:
		{
			CLOG(LNOTICE)<<"Clean from in_intensity";
			in_intensity.read();
			break;
		}
		case ImageMask:
		{
			in_mask.read();
			break;
		}
		case StereoLeft:
		{
			in_stereoL.read();
			break;
		}
		case StereoRight:
		{
			in_stereoR.read();
			break;
		}
		case StereoLeftTextured:
		{
			in_stereoLTextured.read();
			break;
		}
		case StereoRightTextured:
		{
			in_stereoRTextured.read();
			break;
		}
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

void ViewWriter::saveFile(fileTypes & fileType)
{
	CLOG(LNOTICE)<<"ViewWriter::saveFile";

	string filename = (string)fileName;
	string type = "View";
	string ViewName = viewPtr->getViewName();
	string empty = "";
	switch(fileType)
	{
		case FileCameraInfo:
		{
			filename += ".xml";
			string cameraInfo = in_camera_info.read();
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(cameraInfo, fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty,0);
			break;
		}
		case ImageRgb:
		{
			CLOG(LNOTICE)<<"Read ImageRGB!";
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_rgb.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case ImageXyz:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_xyz.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case ImageDepth:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_depth.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case ImageIntensity:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_intensity.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case ImageMask:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_mask.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case StereoLeft:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_stereoL.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case StereoRight:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_stereoR.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case StereoLeftTextured:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_stereoLTextured.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case StereoRightTextured:
		{
			filename += ".png";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_stereoRTextured.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyz:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyz.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzRgb:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgb.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzSift:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzsift.read(), fileType, filename, ViewName, hostname));
			int in_mean_viewpnt_features_number = in_mean_viewpoint_features_number.read();
			viewPtr->addFile(file, type, true, empty, in_mean_viewpnt_features_number);
			break;
		}
		case PCXyzRgbSift:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgbsift.read(), fileType, filename, ViewName, hostname));
			int in_mean_viewpnt_features_number = in_mean_viewpoint_features_number.read();
			viewPtr->addFile(file, type, true, empty, in_mean_viewpnt_features_number);
			break;
		}
		case PCXyzShot:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzshot.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		case PCXyzRgbNormal:
		{
			filename += ".pcd";
			shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(in_pc_xyzrgbnormal.read(), fileType, filename, ViewName, hostname));
			viewPtr->addFile(file, type, true, empty, 0);
			break;
		}
		default:
			break;
	}
}
bool ViewWriter::checkProvidedData(std::vector<fileTypes> & requiredFileTypes, bool& anyMarked)
{
	CLOG(LNOTICE)<<"ViewWriter::checkProvidedData";
	bool cleanBuffers = false;

	if(cameraInfoProp==true)
	{
		anyMarked=true;
		if(in_camera_info.fresh())
			requiredFileTypes.push_back(FileCameraInfo);
		else
			cleanBuffers = true;
	}
	if(xyzProp==true)
	{
		anyMarked=true;
		if(in_xyz.fresh())
			requiredFileTypes.push_back(ImageXyz);
		else
			cleanBuffers = true;
	}
	if(rgbProp==true)
	{
		CLOG(LNOTICE)<<"ViewWriter::checkProvidedData::RGB";
		anyMarked=true;
		if(in_rgb.fresh())
			requiredFileTypes.push_back(ImageRgb);
		else
			cleanBuffers = true;
	}
	if(depthProp==true)
	{
		anyMarked=true;
		if(in_depth.fresh())
			requiredFileTypes.push_back(ImageDepth);
		else
			cleanBuffers = true;
	}
	if(intensityProp==true)
	{
		anyMarked=true;
		if(in_intensity.fresh())
			requiredFileTypes.push_back(ImageIntensity);
		else
			cleanBuffers = true;
	}
	if(maskProp==true)
	{
		anyMarked=true;
		if(in_mask.fresh())
			requiredFileTypes.push_back(ImageMask);
		else
			cleanBuffers = true;
	}
	if(stereoProp==true)
	{
		anyMarked=true;
		if(in_stereoL.fresh() )
		{
			requiredFileTypes.push_back(StereoLeft);
		}
		else
			cleanBuffers = true;
		if(in_stereoR.fresh())
		{
			requiredFileTypes.push_back(StereoRight);
		}
		else
			cleanBuffers = true;
	}
	if(stereoTexturedProp==true)
	{
		anyMarked=true;
		if(in_stereoL.fresh() )
		{
			requiredFileTypes.push_back(StereoLeft);
		}
		else
			cleanBuffers = true;
		if(in_stereoR.fresh())
		{
			requiredFileTypes.push_back(StereoRight);
		}
		else
			cleanBuffers = true;
		if(in_stereoLTextured.fresh())
		{
			requiredFileTypes.push_back(StereoLeftTextured);
		}
		else
			cleanBuffers = true;

		if(in_stereoRTextured.fresh())
		{
			requiredFileTypes.push_back(StereoRightTextured);
		}
		else
			cleanBuffers = true;
	}
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

} //: namespace ViewWriter
} //: namespace Processors
