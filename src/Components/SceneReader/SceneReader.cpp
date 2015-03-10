/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "SceneReader.hpp"


namespace Processors {
namespace SceneReader  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;

SceneReader::SceneReader(const std::string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		viewName("viewName", string("GreenCup")),
		sceneName("sceneName", string("scene1")),
		collectionName("collectionName", string("containers")),
		getScenesFromViewFlag("Get scenes", false),
		getViewFlag("Get view", false)
{
		registerProperty(mongoDBHost);
		registerProperty(sceneName);
		registerProperty(viewName);
		registerProperty(collectionName);
		registerProperty(getViewFlag);
		registerProperty(getScenesFromViewFlag);
        CLOG(LTRACE) << "Hello SceneReader";
}

SceneReader::~SceneReader()
{
        CLOG(LTRACE) << "Good bye SceneReader";
}

void SceneReader::readfromDB()
{
	CLOG(LNOTICE) << "SceneReader::readfromDB";
	if(getViewFlag)
	{
		CLOG(LINFO)<<"Get view";
		getView();
	}
	else if(getScenesFromViewFlag)
	{
		CLOG(LINFO)<<"Get scene";
		getScenes();
	}
}

void SceneReader::getView()
{
	sn = sceneName;
	string vn = viewName;
	viewPtr = boost::shared_ptr<View>(new View(vn,hostname));
	bool exist = viewPtr->checkIfExist();
	if(exist)
	{
		BSONObj query = BSON("Name"<<viewName<<"Type"<<"View");
		cursorCollection = MongoProxy::MongoProxy::getSingleton(hostname).query(query);
		vector<OID> childsVector;
		BSONObj sceneDocument = cursorCollection->next();

		string output = sceneDocument.getField("Name").str();
		if(output!="EOO")
		{
			CLOG(LINFO)<<"SceneName : "<< output;
			if(output==sn)
				CLOG(LINFO)<<sceneName <<"=="<<output;
		}
	}
	else
	{
		CLOG(LERROR)<<"View doesn't exist!";
		return;
	}
}

void SceneReader::getScenes()
{
	sn = sceneName;
	scenePtr = boost::shared_ptr<Scene>(new Scene(sn,hostname));
	BSONObj query = BSON("Name"<<sn<<"Type"<<"Scene");
	CLOG(LINFO)<<"Name"<<sn<<"Type"<<"Scene";
	cursorCollection = MongoProxy::MongoProxy::getSingleton(hostname).query(query);
	if (cursorCollection->more())
	{
		vector<OID> childsVector;
		BSONObj sceneDocument = cursorCollection->next();
		vector<OID> viewsList;
		int items =  MongoProxy::MongoProxy::getSingleton(hostname).getChildOIDS(sceneDocument, "ViewsList", "viewOID", viewsList);
		if(items>0)
		{
			for(std::vector<OID>::iterator viewOIDIter = viewsList.begin(); viewOIDIter != viewsList.end(); ++viewOIDIter)
			{
				BSONObj query = BSON("_id" << *viewOIDIter);
				BSONObj viewDocument = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
				string output = viewDocument.getField("Name").str();
				if(output!="EOO")
				{
					CLOG(LINFO)<<"ViewName : "<< output;
				}
				else
				{
					CLOG(LINFO)<<"No name!";
					return;
				}
			}//for
		}//if
		else
			CLOG(LINFO)<<"No View in Scene";
	}//if
	else
	{
		CLOG(LINFO)<<"No Scene founded!!!";
	}
	return;
}


void SceneReader::prepareInterface() {
        CLOG(LTRACE) << "SceneReader::prepareInterface";
        registerHandler("readData", boost::bind(&SceneReader::readfromDB, this));
        // adding dependency
        addDependency("readData", NULL);
}

bool SceneReader::onInit()
{
        CLOG(LTRACE) << "SceneReader::initialize";
        hostname = mongoDBHost;
        MongoProxy::MongoProxy::getSingleton(hostname);
		return true;
}

bool SceneReader::onFinish()
{
        CLOG(LTRACE) << "SceneReader::finish";
        return true;
}

bool SceneReader::onStep()
{
        CLOG(LTRACE) << "SceneReader::step";
        return true;
}

bool SceneReader::onStop()
{
        return true;
}

bool SceneReader::onStart()
{
        return true;
}

} //: namespace SceneReader
} //: namespace Processors
