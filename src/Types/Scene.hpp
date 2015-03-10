/*
 * Scene.hpp
 *
 *  Created on: Jan 11, 2015
 *      Author: lzmuda
 */

#ifndef SCENE_HPP_
#define SCENE_HPP_


#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>

#include "Logger.hpp"

#include <cstdlib>
#include <iostream>
#include <glob.h>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "Logger.hpp"
#include "mongo/client/dbclient.h"
#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"
#include <dirent.h>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/SIFTObjectModelFactory.hpp>
#include <Types/MongoProxy.hpp>
#include <Types/PrimitiveFile.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

namespace MongoDB{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace PrimitiveFile;

class Scene : Document
{
private:
	string hostname;

public:
	Scene(string& sceneName, string& host) : Document(sceneName), hostname(host)
	{

	}
	void create(OID&);
	void addView(string& ViewName, OID& viewOID);
	}; //class Scene

void Scene::addView(string& ViewName, OID& viewOID)
{
	// insert view oid to scene document
	BSONObj query = BSON("Name"<<Name<<"Type"<<"Scene");
	BSONObj update = BSON("$addToSet"<<BSON("ViewsList"<<BSON("viewOID"<<viewOID.toString())));
	MongoProxy::MongoProxy::getSingleton(hostname).update(query, update);
}

void Scene::create(OID& sceneOID)
{
	BSONObj sceneQuery= BSON("Name"<<Name<<"Type"<<"Scene");
	// check if object exist
	int items = MongoProxy::MongoProxy::getSingleton(hostname).count(sceneQuery);
	if(items==0)
	{
		BSONArrayBuilder objectArrayBuilder;
		BSONObj scene = BSONObjBuilder().genOID().append("Name", Name).append("Type","Scene").append("description", description).obj();
		// get scene oid
		BSONElement bsonElement;
		scene.getObjectID(bsonElement);
		sceneOID=bsonElement.__oid();
		MongoProxy::MongoProxy::getSingleton(hostname).insert(scene);
	}
	else
	{
		LOG(LINFO)<<"Scene exist!!!";
		BSONObj sceneObj = MongoProxy::MongoProxy::getSingleton(hostname).findOne(sceneQuery);
		BSONElement bsonElement;
		sceneObj.getObjectID(bsonElement);
		OID objectOID;
		sceneOID = bsonElement.__oid();
	}
	return;
}
};// namespace MongoDB

#endif /* SCENE_HPP_ */
