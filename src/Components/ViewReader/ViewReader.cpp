/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewReader.hpp"


namespace Processors {
namespace ViewReader  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewReader::ViewReader(const std::string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	viewName("ViewName", string("lab012")),
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
	registerProperty(mongoDBHost);
	registerProperty(viewName);
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

	hostname = mongoDBHost;
	CLOG(LTRACE) << "Hello ViewReader";
}

ViewReader::~ViewReader()
{
        CLOG(LTRACE) << "Good bye ViewReader";
}


void ViewReader::prepareInterface() {
	CLOG(LTRACE) << "ViewReader::prepareInterface";
	h_readfromDB.setup(this, &ViewReader::readfromDB);
	registerHandler("Read", &h_readfromDB);
	registerStream("out_camera_info", &out_camera_info);
	registerStream("out_xyz", &out_xyz);
	registerStream("out_rgb", &out_rgb);
	registerStream("out_depth", &out_depth);
	registerStream("out_intensity", &out_intensity);
	registerStream("out_mask", &out_mask);

	registerStream("out_stereoL", &out_stereoL);
	registerStream("out_stereoR", &out_stereoR);
	registerStream("out_stereoLTextured", &out_stereoLTextured);
	registerStream("out_stereoRTextured", &out_stereoRTextured);

	registerStream("out_pc_xyz", &out_pc_xyz);
	registerStream("out_pc_xyzrgb", &out_pc_xyzrgb);
	registerStream("out_pc_xyzsift", &out_pc_xyzsift);
	registerStream("out_pc_xyzrgbsift", &out_pc_xyzrgbsift);
	registerStream("out_pc_xyzshot", &out_pc_xyzshot);
	registerStream("out_pc_xyzrgbnormal", &out_pc_xyzrgbnormal);

	//registerHandler("onTriggeredReadAllFiles", boost::bind(&ViewReader::readAllFilesTriggered, this));
	//addDependency("onTriggeredReadAllFiles", &in_trigger);
}

bool ViewReader::onInit()
{
        CLOG(LTRACE) << "ViewReader::initialize";
        return true;
}

bool ViewReader::onFinish()
{
        CLOG(LTRACE) << "ViewReader::finish";
        return true;
}

bool ViewReader::onStep()
{
        CLOG(LTRACE) << "ViewReader::step";
        return true;
}

bool ViewReader::onStop()
{
        return true;
}

bool ViewReader::onStart()
{
        return true;
}

void ViewReader::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ViewReader::addToAllChilds";
}

void ViewReader::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ViewReader::readAllFiles";
}


void ViewReader::readfromDB()
{
	CLOG(LNOTICE) << "ViewReader::readfromDB";
	MongoProxy::MongoProxy::getSingleton(hostname);
	string vn = string(viewName);
	viewPtr = boost::shared_ptr<View>(new View(vn,hostname));

	bool exist = viewPtr->checkIfExist();
	if(!exist)
	{
		CLOG(LERROR)<<"View doesn't exist in data base!!!, Change view name";
		return;
	}
	else
	{
		// get view document
		viewPtr->readViewDocument();

		// read all required types from GUI
		std::vector<fileTypes> requiredFileTypes;
		readRequiredData(requiredFileTypes);


		if(requiredFileTypes.size()==0)
		{
			CLOG(LERROR)<<"Please mark any checkbox";
			return;
		}
		// check if view contain all required types
		bool contain = viewPtr->checkIfContain(requiredFileTypes);

		if(!contain)
		{
			CLOG(LERROR)<<"View doesn't contain all required files! BYE!";
		}
		else
		{
			CLOG(LNOTICE)<<"Read files from View!";

			// for full required files vector, read file document and check if its type is equal
			// one of requested file types

			viewPtr->getRequiredFiles(requiredFileTypes);

			//write to output
			LOG(LNOTICE)<<"WRITE FILE!!!";
			int filesNr = viewPtr->getFilesSize();
			for (int i=0; i<filesNr; i++)
			{
				// get type
				fileTypes ft = viewPtr->getFileType(i);
				// send to output
				switch(ft)
				{
					case FileCameraInfo:
					{
						std::string str;
						viewPtr->getFile(i)->getStringData(str);
						out_camera_info.write(str);
						break;
					}
					case ImageRgb:
					{
						CLOG(LNOTICE)<<"Get CV mat data!";
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_rgb.write(img);
						break;
					}
					case ImageDepth:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_depth.write(img);
						break;
					}
					case ImageIntensity:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_intensity.write(img);
						break;
					}
					case ImageMask:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_mask.write(img);
						break;
					}
					case StereoLeft:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_stereoL.write(img);
						break;
					}
					case StereoRight:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_stereoR.write(img);
						break;
					}
					case StereoLeftTextured:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_stereoLTextured.write(img);
						break;
					}
					case StereoRightTextured:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_stereoRTextured.write(img);
						break;
					}
					case PCXyz:
					{
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ;
						viewPtr->getFile(i)->getXYZData(cloudXYZ);
						out_pc_xyz.write(cloudXYZ);
						break;
					}
					case PCXyzRgb:
					{
						pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB;
						viewPtr->getFile(i)->getXYZRGBData(cloudXYZRGB);
						out_pc_xyzrgb.write(cloudXYZRGB);
						break;
					}
					case PCXyzSift:
					{
						pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT;
						viewPtr->getFile(i)->getXYZSIFTData(cloudXYZSIFT);
						out_pc_xyzsift.write(cloudXYZSIFT);
						break;
					}
					case PCXyzRgbSift:
					{
						pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT;
						viewPtr->getFile(i)->getXYZRGBSIFTData(cloudXYZRGBSIFT);
						out_pc_xyzrgbsift.write(cloudXYZRGBSIFT);
						break;
					}
					case PCXyzShot:
					{
						pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT;
						viewPtr->getFile(i)->getXYZSHOTData(cloudXYZSHOT);
						out_pc_xyzshot.write(cloudXYZSHOT);
						break;
					}
					case PCXyzRgbNormal:
					{
						pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZNormal;
						viewPtr->getFile(i)->getXYZRGBNormalData(cloudXYZNormal);
						out_pc_xyzrgbnormal.write(cloudXYZNormal);
						break;
					}
					case ImageXyz:
					{
						cv::Mat img;
						viewPtr->getFile(i)->getCVMatData(img);
						out_xyz.write(img);
						break;
					}
				}
			}

		}
	}
}

void ViewReader::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
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
} //: namespace ViewReader
} //: namespace Processors
