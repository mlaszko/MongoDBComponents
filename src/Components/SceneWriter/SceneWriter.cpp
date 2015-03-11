/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "SceneWriter.hpp"
#include <stddef.h>
namespace Processors {
namespace SceneWriter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;
using namespace MongoProxy;


SceneWriter::SceneWriter(const string & name) : Base::Component(name),
		mongoDBHost("mongoDBHost", string("localhost")),
		viewName("viewName", string("032_View")),
		description("description", string("My green coffe cup")),
		sceneNameProp("sceneNameProp", string("scene3"))
{
        registerProperty(mongoDBHost);
        registerProperty(viewName);
        registerProperty(description);
        registerProperty(sceneNameProp);
        CLOG(LTRACE) << "Hello SceneWriter";
}

SceneWriter::~SceneWriter()
{
        CLOG(LTRACE) << "Good bye SceneWriter";
}
void SceneWriter::write2DB()
{
       CLOG(LNOTICE) << "SceneWriter::write2DB";
       sceneName = sceneNameProp;
       addSceneToView();
}

void SceneWriter::prepareInterface() {
        CLOG(LTRACE) << "SceneWriter::prepareInterface";
        h_write2DB.setup(this, &SceneWriter::write2DB);
        registerHandler("write2DB", &h_write2DB);
}

bool SceneWriter::onInit()
{
      CLOG(LTRACE) << "SceneWriter::initialize";
	  hostname = mongoDBHost;
	  MongoProxy::MongoProxy::getSingleton(hostname);
	  return true;
}

bool SceneWriter::onFinish()
{
        CLOG(LTRACE) << "SceneWriter::finish";
        return true;
}

bool SceneWriter::onStep()
{
        CLOG(LTRACE) << "SceneWriter::step";
        return true;
}

bool SceneWriter::onStop()
{
        return true;
}

bool SceneWriter::onStart()
{
        return true;
}

void SceneWriter::addSceneToView()
{

	CLOG(LTRACE)<<"Scene: "<<sceneName;

	// if scene exist
	string vn = viewName;
	string sn = sceneName;
	viewPtr = boost::shared_ptr<View>(new View(vn,hostname));
	scenePtr = boost::shared_ptr<Scene>(new Scene(sn,hostname));
	bool exist = viewPtr->checkIfExist();
	BSONObj sceneQuery = BSON("Name"<<sceneName<<"Type"<<"Scene");
	// no view
	if(!exist)
	{
		int items = MongoProxy::MongoProxy::getSingleton(hostname).count(sceneQuery);
		string vn = viewName;
		OID viewOID;
		OID sceneOID;
		// no scene
		if(items==0)
		{
			scenePtr = boost::shared_ptr<Scene>(new Scene(sceneName,hostname));
			scenePtr->create(sceneOID);
		}
		else
		{
			// get sceneOID
			BSONObj scene = MongoProxy::MongoProxy::getSingleton(hostname).findOne(sceneQuery);
			BSONElement sceneOI;
			scene.getObjectID(sceneOI);
			sceneOID=sceneOI.__oid();
		}
		// create, add scene to view
		viewPtr->create(sceneOID, viewOID, sn);
		// add view to scene
		scenePtr->addView(vn, viewOID);
	}
	else	// view exist
	{
		LOG(LINFO)<<("Check scene!");
		int items = MongoProxy::MongoProxy::getSingleton(hostname).count(sceneQuery);
		string vn = viewName;
		OID viewOID;
		OID sceneOID;
		// no scene
		if(items==0)
		{
			scenePtr = boost::shared_ptr<Scene>(new Scene(sceneName,hostname));
			scenePtr->create(sceneOID);
		}
		else
		{
			// get sceneOID
			BSONObj scene = MongoProxy::MongoProxy::getSingleton(hostname).findOne(sceneQuery);
			BSONElement sceneOI;
			scene.getObjectID(sceneOI);
			sceneOID=sceneOI.__oid();
		}
		//check if view contains scene and scene contains view
		BSONObj viewQuery = BSON("Name"<<viewName<<"Type"<<"View");

		BSONObj view = MongoProxy::MongoProxy::getSingleton(hostname).findOne(viewQuery);
		string output = view.getField("SceneName");
		if(output=="EOO")
		{
			// create, add scene to view
			viewPtr->addScene(sn, sceneOID);

			// add view to scene
			BSONElement viewOI;
			view.getObjectID(viewOI);
			viewOID=viewOI.__oid();
			scenePtr->addView(vn, viewOID);
		}
	}
}


} //: namespace SceneWriter
} //: namespace Processors
