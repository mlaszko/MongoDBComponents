/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ViewRemover.hpp"


namespace Processors {
namespace ViewRemover  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ViewRemover::ViewRemover(const std::string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	viewName("Name", string("lab012")),
	cameraInfoProp("file.cameraInfo.xml", false),
	xyzProp("image.xyz", false),
	rgbProp("image.rgb", false),
	depthProp("image.depth", false),
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
	removeAll("removeAll", false)
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
	registerProperty(removeAll);

	hostname = mongoDBHost;
	CLOG(LTRACE) << "Hello ViewRemover";
}

ViewRemover::~ViewRemover()
{
        CLOG(LTRACE) << "Good bye ViewRemover";
}


void ViewRemover::prepareInterface() {
	CLOG(LTRACE) << "ViewRemover::prepareInterface";
	h_readfromDB.setup(this, &ViewRemover::readfromDB);
	registerHandler("Remove", &h_readfromDB);
}

bool ViewRemover::onInit()
{
        CLOG(LTRACE) << "ViewRemover::initialize";
        MongoProxy::MongoProxy::getSingleton(hostname);
        return true;
}

bool ViewRemover::onFinish()
{
        CLOG(LTRACE) << "ViewRemover::finish";
        return true;
}

bool ViewRemover::onStep()
{
        CLOG(LTRACE) << "ViewRemover::step";
        return true;
}

bool ViewRemover::onStop()
{
        return true;
}

bool ViewRemover::onStart()
{
        return true;
}

void ViewRemover::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ViewRemover::addToAllChilds";
}

void ViewRemover::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ViewRemover::readAllFiles";
}


void ViewRemover::readfromDB()
{
	CLOG(LNOTICE) << "ViewRemover::readfromDB";

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
		BSONObj query;
		BSONObj update;
		// get view document
		viewPtr->readViewDocument();
		bool contain = true;
		CLOG(LNOTICE)<<"Read files from View!";
		if(removeAll)
		{
			viewPtr->getAllFiles();
			// remove all files
			int filesNr = viewPtr->getFilesSize();
			// remove all from view files
			for (int i=0; i<filesNr; i++)
			{
				CLOG(LERROR)<<"Removed Type: " << viewPtr->getFile(i)->getType();
				OID fileOID = viewPtr->getFile(i)->getOID();
				CLOG(LERROR)<<"Usuwamy : " << FTypes[viewPtr->getFile(i)->getType()];
				CLOG(LERROR)<<"fileOID: " << fileOID;
				query = BSON("Name"<<vn<<"Type"<<"View");
				update = BSON("$pull"<<BSON("FileTypes"<<BSON("Type"<<FTypes[viewPtr->getFile(i)->getType()])));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				update = BSON("$pull"<<BSON("fileOIDs"<<BSON("fileOID"<<fileOID.toString())));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				viewPtr->getFile(i)->removeFile();
			}
			//get scene from doc
			string sceneName;
			viewPtr->getSceneName(sceneName);
			OID viewOID;
			viewPtr->getID(viewOID);
			CLOG(LERROR)<<"Scene: " << sceneName;
			CLOG(LERROR)<<"viewOID.toString(): " << viewOID.toString();
			query = BSON("Name"<<sceneName<<"Type"<<"Scene");
			update = BSON("$pull"<<BSON("ViewsList"<<BSON("viewOID"<<viewOID.toString())));
			MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
			// remove from viewsSet
			vector<OID> viewsSetOIDS;
			string tableName = "viewSetList";
			string fieldName = "ViewsSetOID";
			viewPtr->getViewsSetOID(viewsSetOIDS, tableName, fieldName);
			for(std::vector<OID>::iterator viewsSetIter = viewsSetOIDS.begin(); viewsSetIter != viewsSetOIDS.end(); ++viewsSetIter)
			{
				query = BSON("_id" << *viewsSetIter);
				update = BSON("$pull"<<BSON("ViewsList"<<BSON("viewOID"<<viewOID.toString())));
				CLOG(LERROR)<<query.toString(false, false);
				CLOG(LERROR)<<update.toString(false, false);
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
			}
			// remove view document
			MongoProxy::MongoProxy::getSingleton(hostname).remove(viewOID);
		}
		else
		{
			// read all required types from GUI
			std::vector<fileTypes> requiredFileTypes;
			readRequiredData(requiredFileTypes);
			if(requiredFileTypes.size()==0)
			{
				CLOG(LERROR)<<"Please mark any checkbox";
				return;
			}
			// check if view contain all required types
			contain = viewPtr->checkIfContain(requiredFileTypes);
			if(!contain)
			{
				CLOG(LERROR)<<"View doesn't contain all required files! BYE!";
			}
			viewPtr->getRequiredFiles(requiredFileTypes);

			int filesNr = viewPtr->getFilesSize();
			// remove only marked files
			for (int i=0; i<filesNr; i++)
			{
				CLOG(LERROR)<<"Removed Type: " << viewPtr->getFile(i)->getType();
				OID fileOID = viewPtr->getFile(i)->getOID();
				CLOG(LERROR)<<"Usuwamy : " << FTypes[viewPtr->getFile(i)->getType()];
				CLOG(LERROR)<<"fileOID: " << fileOID;
				BSONObj query = BSON("Name"<<vn<<"Type"<<"View");
				BSONObj update = BSON("$pull"<<BSON("FileTypes"<<BSON("Type"<<FTypes[viewPtr->getFile(i)->getType()])));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				update = BSON("$pull"<<BSON("fileOIDs"<<BSON("fileOID"<<fileOID.toString())));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				viewPtr->getFile(i)->removeFile();
			}
		}
	}

}

void ViewRemover::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
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
} //: namespace ViewRemover
} //: namespace Processors
