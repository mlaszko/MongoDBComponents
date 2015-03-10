/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "ModelRemover.hpp"


namespace Processors {
namespace ModelRemover  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::posix_time;

ModelRemover::ModelRemover(const std::string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	modelName("ModelName", string("lab012")),
	pc_xyzProp("PC.xyz", false),
	pc_xyzrgbProp("PC.xyzrgb", false),
	pc_xyzsiftProp("PC.xyzsift", false),
	pc_xyzrgbsiftProp("PC.xyzrgbsift", false),
	pc_xyzshotProp("PC.xyzshot", false),
	pc_xyzrgbnormalProp("PC.xyzrgbnormal", false),
	removeAll("removeAll", false)
{
	registerProperty(mongoDBHost);
	registerProperty(modelName);
	registerProperty(pc_xyzProp);
	registerProperty(pc_xyzrgbProp);
	registerProperty(pc_xyzsiftProp);
	registerProperty(pc_xyzrgbsiftProp);
	registerProperty(pc_xyzshotProp);
	registerProperty(pc_xyzrgbnormalProp);
	registerProperty(removeAll);

	hostname = mongoDBHost;
	CLOG(LTRACE) << "Hello ModelRemover";
}

ModelRemover::~ModelRemover()
{
        CLOG(LTRACE) << "Good bye ModelRemover";
}


void ModelRemover::prepareInterface() {
	CLOG(LTRACE) << "ModelRemover::prepareInterface";
	h_readfromDB.setup(this, &ModelRemover::readfromDB);
	registerHandler("Remove", &h_readfromDB);
}

bool ModelRemover::onInit()
{
        CLOG(LTRACE) << "ModelRemover::initialize";
        MongoProxy::MongoProxy::getSingleton(hostname);
        return true;
}

bool ModelRemover::onFinish()
{
        CLOG(LTRACE) << "ModelRemover::finish";
        return true;
}

bool ModelRemover::onStep()
{
        CLOG(LTRACE) << "ModelRemover::step";
        return true;
}

bool ModelRemover::onStop()
{
        return true;
}

bool ModelRemover::onStart()
{
        return true;
}

void ModelRemover::addToAllChilds(std::vector<OID> & childsVector)
{
	CLOG(LTRACE)<<"ModelRemover::addToAllChilds";
}

void ModelRemover::readAllFilesTriggered()
{
	CLOG(LTRACE)<<"ModelRemover::readAllFiles";
}


void ModelRemover::readfromDB()
{
	CLOG(LNOTICE) << "ModelRemover::readfromDB";

	string vn = string(modelName);
	modelPtr = boost::shared_ptr<Model>(new Model(vn,hostname));

	bool exist = modelPtr->checkIfExist();
	if(!exist)
	{
		CLOG(LERROR)<<"Model doesn't exist in data base!!!, Change model name";
		return;
	}
	else
	{
		BSONObj query;
		BSONObj update;
		// get model document
		modelPtr->readModelDocument();
		bool contain = true;
		if(removeAll)
		{
			modelPtr->getAllFiles();
			// remove all files
			int filesNr = modelPtr->getFilesSize();
			// remove all from model files
			for (int i=0; i<filesNr; i++)
			{
				CLOG(LERROR)<<"Removed Type: " << modelPtr->getFile(i)->getType();
				OID fileOID = modelPtr->getFile(i)->getOID();
				CLOG(LERROR)<<"Usuwamy : " << FTypes[modelPtr->getFile(i)->getType()];
				CLOG(LERROR)<<"fileOID: " << fileOID;
				query = BSON("Name"<<vn<<"Type"<<"Model");
				update = BSON("$pull"<<BSON("FileTypes"<<BSON("Type"<<FTypes[modelPtr->getFile(i)->getType()])));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				update = BSON("$pull"<<BSON("fileOIDs"<<BSON("fileOID"<<fileOID.toString())));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				modelPtr->getFile(i)->removeFile();
			}
			OID modelOID;
			modelPtr->getID(modelOID);
			// remove from viewsSet
			vector<OID> viewsSetOIDS;
			string tableName = "viewSetList";
			string fieldName = "ViewsSetOID";
			modelPtr->getViewsSetOID(viewsSetOIDS, tableName, fieldName);

			for(std::vector<OID>::iterator viewsSetIter = viewsSetOIDS.begin(); viewsSetIter != viewsSetOIDS.end(); ++viewsSetIter)
			{
				query = BSON("_id" << *viewsSetIter);
				update = BSON("$pull"<<BSON("ModelsList"<<BSON("modelOID"<<modelOID.toString())));
				CLOG(LERROR)<<query.toString(false, false);
				CLOG(LERROR)<<update.toString(false, false);
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
			}
			// remove model document
			MongoProxy::MongoProxy::getSingleton(hostname).remove(modelOID);
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
			// check if model contain all required types
			contain = modelPtr->checkIfContain(requiredFileTypes);
			if(!contain)
			{
				CLOG(LERROR)<<"Model doesn't contain all required files! BYE!";
			}
			modelPtr->getRequiredFiles(requiredFileTypes);

			int filesNr = modelPtr->getFilesSize();
			// remove only marked files
			for (int i=0; i<filesNr; i++)
			{
				CLOG(LERROR)<<"Removed Type: " << modelPtr->getFile(i)->getType();
				OID fileOID = modelPtr->getFile(i)->getOID();
				LOG(LERROR)<<"Usuwamy : " << FTypes[modelPtr->getFile(i)->getType()];
				CLOG(LERROR)<<"fileOID: " << fileOID;
				BSONObj query = BSON("Name"<<vn<<"Type"<<"Model");
				BSONObj update = BSON("$pull"<<BSON("FileTypes"<<BSON("Type"<<FTypes[modelPtr->getFile(i)->getType()])));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				update = BSON("$pull"<<BSON("fileOIDs"<<BSON("fileOID"<<fileOID.toString())));
				MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
				modelPtr->getFile(i)->removeFile();
			}
		}
	}
}

void ModelRemover::readRequiredData(std::vector<fileTypes> & requiredFileTypes)
{
	CLOG(LNOTICE)<<"ModelRemover::readRequiredData";
	bool cleanBuffers = false;
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
} //: namespace ModelRemover
} //: namespace Processors
