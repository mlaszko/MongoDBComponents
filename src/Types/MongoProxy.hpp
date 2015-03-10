/*
 * MongoProxy.hpp
 *
 *  Created on: Sep 23, 2014
 *      Author: lzmuda
 */

#ifndef MONGOPROXY_HPP_
#define MONGOPROXY_HPP_

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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <Types/PointXYZSIFT.hpp>
#include <Types/PointXYZRGBSIFT.hpp>
#include <Types/SIFTObjectModelFactory.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>
#include <iostream>

namespace MongoProxy {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;

boost::mutex io_mutex;

class MongoProxy {
private:
	string dbCollectionPath;
	string hostname;
    boost::shared_ptr<DBClientConnection> c;
    MongoProxy(string& hostname) : hostname(hostname)
    {
    	c = boost::shared_ptr<DBClientConnection>(new DBClientConnection());
    	mongo::client::initialize();
    	connectToMongoDB();
    	dbCollectionPath="images.containers";
    	collectionName = "containers";
    }
    MongoProxy(const MongoProxy & );
public:
   static MongoProxy & getSingleton(string& hostname)
   {
	   boost::mutex::scoped_lock lock(io_mutex);
	   static MongoProxy singleton(hostname);
	   return singleton;
   }
    string collectionName;
	void remove(OID & oid);
	void connectToMongoDB();
    void insert(BSONObj&);
    int count(BSONObj & object);
    void update(BSONObj& query, BSONObj& update);
    auto_ptr<DBClientCursor> query(BSONObj& query);
    boost::shared_ptr<DBClientConnection>  getClient();
    void index(string& field);
    BSONObj findOne(BSONObj& query);
	int getChildOIDS(BSONObj &obj, const string&,  const string&, vector<OID>&);

};

int MongoProxy::getChildOIDS(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<OID>& oidsVector)
{
	string output = obj.getField(fieldName);
	if(output!="EOO")
	{
		vector<BSONElement> v = obj.getField(fieldName).Array();
		for (unsigned int i = 0; i<v.size(); i++)
		{
			string readedOid =v[i][childfieldName].str();
			OID o = OID(readedOid);
			oidsVector.push_back(o);
		}
		return 1;
	}
	else
		return 0;
}
boost::shared_ptr<DBClientConnection>  MongoProxy::getClient()
{
	boost::shared_ptr<DBClientConnection> c2 = c;
	return c2;
}


void MongoProxy::insert(BSONObj & object)
{
	c->insert(dbCollectionPath, object);
}

void MongoProxy::remove(OID & oid)
{
	c->remove(dbCollectionPath , Query(BSON("_id" << oid)));
}

int MongoProxy::count(BSONObj & object)
{
	int options=0;
	int limit=0;
	int skip=0;
	return c->count(dbCollectionPath, object, options, limit, skip);
}

void MongoProxy::update(BSONObj& query, BSONObj& update)
{
	c->update(dbCollectionPath, Query(query), update, false, true);
}

BSONObj MongoProxy::findOne(BSONObj& query)
{
	return c->findOne(dbCollectionPath, Query(query));
}

auto_ptr<DBClientCursor> MongoProxy::query(BSONObj& query)
{
	return c->query(dbCollectionPath, (Query(query)));
}

void MongoProxy::index(string& field)
{
	c->createIndex(dbCollectionPath, BSON(field<<1));
}
//TODO to dodac do wstawiania i odczytu z GRID!!!
/*
void MongoProxy::cloudEncoding(OID& oid, string& tempFileName, string & cloudType)
{
	try{
		pcl::io::OctreePointCloudCompression<pcl::PointXYZ>* PointCloudDecoderXYZ;
		pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>* PointCloudDecoderXYZRGB;
		int QueryOptions = 0;
		const BSONObj *fieldsToReturn = 0;
		LOG(LNOTICE)<<"dbCollectionPath: "<<dbCollectionPath<<"\n\n";

		// get bson object from collection
		LOG(LNOTICE)<<oid<<endl;
		BSONObj obj = c->findOne(dbCollectionPath, Query( BSON("_id" << oid)), fieldsToReturn, QueryOptions);

		// read data to buffer
		int readSize;
		LOG(LNOTICE)<<obj<<"\n";

		const char* charPtr= (const char*)(obj[tempFileName].binData(readSize));
		LOG(LNOTICE)<<"Readed: "<<readSize<<" B.";
		LOG(LNOTICE)<<"charPtr: "<<charPtr;
		stringstream * strPtr = (stringstream*)charPtr;
		LOG(LNOTICE)<<"strPtr: "<<strPtr;
		LOG(LNOTICE)<<"charPtr[0]: "<<charPtr[0];
		LOG(LNOTICE)<<*charPtr;
		LOG(LNOTICE)<<*strPtr;
		LOG(LNOTICE)<<strPtr->str();

		if(cloudType=="xyz")
		{
			PointCloudDecoderXYZ=new pcl::io::OctreePointCloudCompression<pcl::PointXYZ>();
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			PointCloudDecoderXYZ->decodePointCloud (*strPtr, cloudXYZ);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZ, false);
		}
		else if(cloudType=="xyzrgb")
		{
			PointCloudDecoderXYZRGB=new pcl::io::OctreePointCloudCompression<pcl::PointXYZRGB>();
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
			PointCloudDecoderXYZRGB->decodePointCloud (*strPtr, cloudXYZRGB);
			pcl::io::savePCDFile("newCloud2.pcd", *cloudXYZRGB, false);
		}
		LOG(LNOTICE)<<"ViewWriter::insertFileIntoCollection: END";
	}catch(Exception &ex){LOG(LNOTICE)<<ex.what();}
}
*/
void MongoProxy::connectToMongoDB()
{
	try
	{
		LOG(LNOTICE)<<"hostname "<<hostname;
		c->connect(hostname);
		LOG(LNOTICE)<<"Connected to base\n";
	}
	catch(ConnectException& ex)
	{
		LOG(LNOTICE)<<ex.what();
	}
}
} /* namespace MongoProxy */
#endif /* MONGOPROXY_HPP_ */
