/*
 * PrimitiveFile.hpp
 *
 *  Created on: Nov 20, 2014
 *      Author: lzmuda
 */

#ifndef PRIMITIVEFILE_HPP_
#define PRIMITIVEFILE_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/variant.hpp>
#include <boost/chrono/thread_clock.hpp>
#include "Logger.hpp"
#include <iostream>
#include <cstdlib>
#include <time.h>

#include <iostream>
#include <glob.h>
#include <memory>
#include <stdio.h>
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
#include <Types/MongoProxy.hpp>
#include <Types/Document.hpp>

#include <vector>
#include <list>
#include <iostream>
#include <boost/variant.hpp>
#include <boost/lexical_cast.hpp>

#define shotPointSize 367
#define normalPointSize 4
#define siftPointSize 133
#define rgbsiftPointSize 134
#define xyzPointSize 3
#define xyzrgbPointSize 4

enum fileTypes {ImageRgb, FileCameraInfo, ImageXyz,  ImageDepth, ImageIntensity, ImageMask, StereoLeft, StereoRight, StereoLeftTextured, StereoRightTextured,
PCXyz, PCXyzRgb, PCXyzSift, PCXyzRgbSift, PCXyzShot, PCXyzRgbNormal, Stereo, StereoTextured};

const char* FTypes[] = {"ImageRgb", "FileCameraInfo", "ImageXyz",  "ImageDepth", "ImageIntensity", "ImageMask", "StereoLeft", "StereoRight", "StereoLeftTextured", "StereoRightTextured",
"PCXyz", "PCXyzRgb", "PCXyzSift", "PCXyzRgbSift", "PCXyzShot", "PCXyzRgbNormal", "Stereo", "StereoTextured"};


namespace MongoDB{
namespace PrimitiveFile{
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace boost::chrono;

class setSizeVisitor : public boost::static_visitor<float>
{
public:
	float operator()(const std::string & str) const
	{
		return str.length();
	}

	float operator()(const cv::Mat & img) const
	{
		LOG(LDEBUG)<<"operator(): cv::Mat";
		return (float)img.elemSize1()*(float)img.total();
	}

	float operator()(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ) const
	{
		LOG(LDEBUG)<<"operator(): PointXYZ";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<pcl::PointXYZ>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZ->size();
		return sizeBytes;
	}

	float operator()(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB) const
	{
		LOG(LDEBUG)<<"operator(): PointXYZRGB";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<pcl::PointXYZRGB>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGB->size();
		return sizeBytes;
	}

	float operator()(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT) const
    {
		LOG(LDEBUG)<<"operator(): PointXYZSIFT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSIFT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSIFT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT) const
    {
		LOG(LDEBUG)<<"operator(): PointXYZRGBSIFT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZRGBSIFT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZRGBSIFT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT) const
    {
		LOG(LDEBUG)<<"operator(): PointXYZSHOT";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSHOT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZSHOT->size();
		return sizeBytes;
    }

	float operator()(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal) const
    {
		LOG(LDEBUG)<<"operator(): PointXYZRGBNormal";
		float sizeBytes;
		std::vector< pcl::PCLPointField> fields;
		pcl::getFields<PointXYZSHOT>(fields);
		for(std::vector< pcl::PCLPointField>::iterator it = fields.begin(); it != fields.end(); ++it)
			sizeBytes+=(float)pcl::getFieldSize(it->datatype)*cloudXYZNormal->size();
		return sizeBytes;
    }
};

class PrimitiveFile : public Document
{
private:
	//string mongoFileName;	// file name in mongo base
	//string fileName;
	string pcdCloudType;	// sift, shot, xyz,  itd... itp...
	string place;			// {grid, document}
	float sizeBytes;
	float sizeMBytes;
	fileTypes fileType; 		// mask, rgb, depth, I, XYZ, cameraInfo, PCL
	string PCLType;			// SIFT, SHOT, XYZ, ORB, NORMALS
	string collectionName;
	boost::variant<	cv::Mat,
	string,
	pcl::PointCloud<pcl::PointXYZ>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr,
	pcl::PointCloud<PointXYZSIFT>::Ptr,
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr,
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr,
	pcl::PointCloud<PointXYZSHOT>::Ptr
	> data;
	string hostname;

//	boost::shared_ptr<MongoBase> basePtr;

public:
	PrimitiveFile(const cv::Mat& img, fileTypes& type, string& name, string& viewName, string& hostname) : data(img), fileType(type), Document(name),  hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor cv::Mat";
		LOG(LDEBUG)<<"fileType :" <<fileType;
		this->setSize();
	};
	PrimitiveFile(const std::string& str, fileTypes& type, string& name, string& viewName, string& hostname) : data(str), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor std::string";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,  fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <pcl::PointXYZ>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <pcl::PointXYZRGB>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSIFT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <PointXYZSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <PointXYZRGBSIFT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<PointXYZSHOT>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <PointXYZSHOT>";
		this->setSize();
	};
	PrimitiveFile(const pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloud, fileTypes& type, string& name, string& viewName, string& hostname) : data(cloud), fileType(type), Document(name), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor <pcl::PointXYZRGBNormal>";
		this->setSize();
	};

	///////////////////////////////////////////////////
	PrimitiveFile(fileTypes& type, string& hostname, mongo::OID& fileOID) : fileType(type), hostname(hostname), Document(fileOID), sizeMBytes(0), sizeBytes(0)
	{
		string noName =  "NoName";
		Name = noName;
		LOG(LDEBUG)<<"Constructor PrimitiveFile";
		LOG(LDEBUG)<<"fileType :" <<fileType;
	};
	PrimitiveFile(string& fileName, fileTypes type, string& hostname) : Document(fileName), fileType(type), hostname(hostname), sizeMBytes(0), sizeBytes(0)
	{
		LOG(LDEBUG)<<"Constructor PrimitiveFile with fileName";
	};

	/////////////////////////////////////////////////////////
	string getFileName(){return Name;}
	void saveToDisc(cv::Mat& image, string& pathToFiles, bool saveToDiscFlag);
	void copyData(int height, int channels, float* dataXYZ, float* xyz_p, int i);
	void saveIntoDisc();
	void saveIntoMongoBase(string& type, string& name, bool dataInBuffer, string& path, OID& oid, int mean_viewpoint_features_number);
	void insertFileIntoGrid(OID& oid, bool dataInBuffer, string& path, string& type, string& name, int mean_viewpoint_features_number);
	void insertFileIntoDocument(OID& oid, string& name, string& type, int mean_viewpoint_features_number);
	void convertToBuffer();
	void getMime();
	void readFromMongoDB();
	void readFromGrid();
	void readFromDocument();
	void getTimestamp(string&);
	void saveImageOnDisc();
	void getCVMatData(cv::Mat&);
	void removeDocument();
	void saveToDisc(bool suffix, bool binary);
	void readXYZMatFromDocument(bool saveToDiscFlag, string& pathToFiles);
	void getFileFromGrid(const GridFile& file, string& path);
	void writeToSinkFromFile(string& path);
	void copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const;
	void copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const;
	void copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const;
	void copyXYZRGBNormalPointToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const;
	void copyXYZSHOTPointToFloatArray (const PointXYZSHOT &p, float * out) const;
	void readPointCloudFromDocument(bool saveToDiscFlag, string& pathToFiles);
	void savePCxyzFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSIFTFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBSIFTFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzRGBNormalFileToDisc(bool suffix, bool binary, std::string& fn);
	void savePCxyzSHOTFileToDisc(bool suffix, bool binary, std::string& fn);
	void setMime(const fileTypes type,  string& mime);
	void readTextFileFromDocument(bool saveToDiscFlag, string& pathToFiles);
	void readFile(bool file2Memory, string& pathToFiles, bool saveToDiscFlag);
	void readImageFromDocument(bool saveToDiscFlag, string& pathToFiles);
	void ReadPCDCloud(const string& filename);
	void getStringData(std::string& str);
	void getXYZData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ);
	void getXYZRGBData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB);
	void getXYZSIFTData(pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT);
	void getXYZRGBSIFTData(pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT);
	void getXYZSHOTData(pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT);
	void getXYZRGBNormalData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal);
	OID& getOID();
	void removeFile();
	void setType(fileTypes type)
	{
		fileType = type;
	};
	int getChildOIDS(BSONObj &obj, const string & fieldName, const string & childfieldName, vector<OID>& oidsVector)
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
	std::string getPlace()
	{
		return place;
	};

	fileTypes getType()
	{
		return fileType;
	};

	float getSize(){return sizeMBytes;};
	void saveImage(OID& oid, string& name, string& type);
	void setSize()
	{
		LOG(LDEBUG)<<"Set size";
		// get size of data in Bytes
		sizeBytes = boost::apply_visitor(setSizeVisitor(), data);
		// convert to MBytes
		sizeMBytes = sizeBytes/(1024*1024);

		LOG(LDEBUG)<< "sizeBytes: "<<(int)sizeBytes;
		LOG(LDEBUG)<< "sizeMBytes: "<<sizeMBytes;

	};
	void setSize(float size)
	{
		LOG(LDEBUG)<<"Set size";
		// get size of data in Bytes
		sizeBytes = size;
		// convert to MBytes
		sizeMBytes = sizeBytes/(1024*1024);

		LOG(LDEBUG)<< "sizeBytes: "<<(int)sizeBytes;
		LOG(LDEBUG)<< "sizeMBytes: "<<sizeMBytes;

	};
};

void PrimitiveFile::readTextFileFromDocument(bool saveToDiscFlag, string& pathToFiles)
{
	LOG(LERROR)<<"Read text\n";

	const char *buffer;
	int size = (int)sizeBytes;
	// get data to buffer
	buffer = BSONDocument[Name].binData(size);
	for (int i=0; i<size;i++)
		LOG(LERROR)<<buffer[i];
	LOG(LERROR)<<*buffer;
	LOG(LERROR)<<"size: "<<size;
	string fromDB(buffer,size-1);
	LOG(LERROR)<<"ReadedFile: \n"<<fromDB;
	LOG(LERROR)<<"Save to sink";
	data = fromDB;
	if(saveToDiscFlag)
	{
		//const char *fn;
		std::ofstream out(Name.c_str());
		out << fromDB;
		out.close();
	}
	//cipFileOut.write(fromDB);
}

void PrimitiveFile::readXYZMatFromDocument(bool saveToDiscFlag, string& pathToFiles)
{
	LOG(LERROR)<<"Read XYZ Mat\n";
	int len;
	float *dataXYZ = (float*)BSONDocument[Name].binData(len);
	int width = BSONDocument.getField("width").Int();//480
	int height = BSONDocument.getField("height").Int();//640
	int channels = BSONDocument.getField("channels").Int();
	cv::Mat image3c(width, height, CV_32FC3);
	cv::Mat image4c(width, height, CV_32FC4);
	vector<float> img;
	float img_ptr[channels];
	for (int i = 0; i < width; ++i)
	{
		if(channels==3)
		{
			float* xyz_p = image3c.ptr <float> (i);
			copyData(height, channels, dataXYZ, xyz_p, i);
			data = image3c;
			saveToDisc(image3c, pathToFiles,saveToDiscFlag);

		}
		else if(channels==4)
		{
			float* xyz_p = image4c.ptr <float> (i);
			copyData(height, channels, dataXYZ, xyz_p, i);
			data = image4c;
			saveToDisc(image4c, pathToFiles,saveToDiscFlag);
		}
		else
		{
			LOG(LERROR)<<"Unsupported! Please add few lines of code :)";
		}
	}
}
void PrimitiveFile::copyData(int height, int channels, float* dataXYZ, float* xyz_p, int i)
{
	for (int j = 0; j < height*channels; j+=channels)
	{
		for(int k=0; k<=channels; k++)
			xyz_p[k+j]=dataXYZ[i*height*channels+j+k];

	}
}

void PrimitiveFile::saveToDisc(cv::Mat& image, string& pathToFiles, bool saveToDiscFlag)
{
	if(saveToDiscFlag)
	{
		string name = pathToFiles+"/"+Name;
		cv::FileStorage fs(name, cv::FileStorage::WRITE);
		fs << "img" << image;
		fs.release();
	}
}

void PrimitiveFile::saveIntoMongoBase(string& type, string& name, bool dataInBuffer, string& path, OID& oid, int mean_viewpoint_features_number)
{
	//add timestamp to Name
	string timestamp;
	getTimestamp(timestamp);
	Name =timestamp+"_"+Name;
	if(sizeMBytes<15)
		insertFileIntoDocument(oid, name, type, mean_viewpoint_features_number);
	else if(sizeMBytes>=15)
		insertFileIntoGrid(oid, dataInBuffer, path, type, name, mean_viewpoint_features_number);
}

void PrimitiveFile::readImageFromDocument(bool saveToDiscFlag, string& pathToFiles)
{
	LOG(LNOTICE)<<"Read image\n";
	int len;
	LOG(LNOTICE)<<"Read image 2\n";
	LOG(LNOTICE)<<"Name: " << Name;
	LOG(LNOTICE)<<"BSONDocument: " << BSONDocument;
	uchar *dataImage = (uchar*)BSONDocument[Name].binData(len);
	LOG(LNOTICE)<<"Read image 3\n";
	std::vector<uchar> v(dataImage, dataImage+len);
	LOG(LNOTICE)<<"Read image 4\n";
	cv::Mat image = cv::imdecode(cv::Mat(v), -1);
	LOG(LNOTICE)<<"Read image 5\n";

	//out_img.write(image);
	data = image;

	// save to disc
	if(saveToDiscFlag)
	{
		LOG(LDEBUG)<<"Save to disc: " <<pathToFiles+"/"+Name;
		imwrite(pathToFiles+"/"+Name, image);
	}
}

void PrimitiveFile::removeDocument()
{
	MongoProxy::MongoProxy::getSingleton(hostname).remove(oid);
}

OID& PrimitiveFile::getOID()
{
	return oid;
}


void PrimitiveFile::readFile(bool file2Memory, string& pathToFiles, bool saveToDiscFlag)
{
//	if(file2Memory!=true)
//		LOG(LWARNING)<<"file2Memory is not set! Method writeToSinkFromFile will not be invoked!!!";
	LOG(LTRACE)<<"MongoProxy::readFile";
	// get bson object from collection
	BSONObj query = BSON("_id" << oid);
	BSONObj obj = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);
	LOG(LDEBUG)<<"obj: "<<obj<<", fileOID: "<<oid.toString();

	place = obj.getField("place").str();

	sizeBytes = (float)obj.getField("size").Int();

	Name = obj.getField("Name").str();
	LOG(LNOTICE)<<"place: "<<place<<" size: "<<(int)sizeBytes<<", fileName: "<<Name<< ", fileType: " << fileType;

	if(place=="document")
	{
		LOG(LDEBUG)<<"Read from document";

		BSONObj query = BSON("_id" << oid);
		BSONDocument = MongoProxy::MongoProxy::getSingleton(hostname).findOne(query);

		if(fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask
				|| fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured
				|| fileType==StereoRightTextured)
		{
			readImageFromDocument(saveToDiscFlag, pathToFiles);
		}

		else if(fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift
				|| fileType==PCXyzShot || fileType==PCXyzRgbNormal)
		{
			readPointCloudFromDocument(saveToDiscFlag, pathToFiles);
		}
		else if(fileType==ImageXyz)
		{
			readXYZMatFromDocument(saveToDiscFlag, pathToFiles);
		}
		else if(fileType==FileCameraInfo)
		{
			readTextFileFromDocument(saveToDiscFlag, pathToFiles);
		}
	}
	else
	{
		LOG(LDEBUG)<<"Read from GRIDFS!";
		// if saved in grid
		boost::shared_ptr<DBClientConnection>  c = MongoProxy::MongoProxy::getSingleton(hostname).getClient();
		// create GridFS client
		GridFS fs(*c, MongoProxy::MongoProxy::getSingleton(hostname).collectionName);

		LOG(LTRACE)<<"_id"<<oid;
		GridFile file = fs.findFile(Query(BSON("_id" << oid)));

		if (!file.exists())
		{
			LOG(LERROR) << "File not found in grid";
		}
		else
		{
			// get filename
			//string filename = file.getFileField("filename").str();
			//LOG(LNOTICE)<<"filename: "<<filename;
			//LOG(LNOTICE)<<"Name: "<<file.getFileField("Name").str();;
			LOG(LNOTICE)<<"Name: "<<Name;
			getFileFromGrid(file, pathToFiles);
			string empty = "";
			if(file2Memory)
				writeToSinkFromFile(Name);
				//writeToSinkFromFile(empty);
			else
				LOG(LDEBUG)<<"file2Memory=" <<file2Memory;
		}

	}
}
void PrimitiveFile::removeFile()
{
	boost::shared_ptr<DBClientConnection>  c = MongoProxy::MongoProxy::getSingleton(hostname).getClient();
	GridFS fs(*c, MongoProxy::MongoProxy::getSingleton(hostname).collectionName);

	// remove from grid
	if(place=="grid")
	{
		//TODO check if it works!!!
		LOG(LDEBUG)<<"Remove : " << Name;
		fs.removeFile(Name);
	}
	this->removeDocument();
}

void PrimitiveFile::ReadPCDCloud(const string& filename)
{
	LOG(LDEBUG) << "PrimitiveFile::ReadPCDCloud";
	LOG(LDEBUG) << "filename::"<<filename;
	// Try to read the cloud of XYZRGB points.
	if(filename.find("xyzrgb")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyzrgb)!=string::npos";
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud_xyzrgb) == -1){
			LOG(LERROR) <<"Cannot read PointXYZRGB cloud from "<<Name;
			return;
		}else{
			//out_cloud_xyzrgb.write(cloud_xyzrgb);
			data = cloud_xyzrgb;
			LOG(LINFO) <<"PointXYZRGB cloud loaded properly from "<<Name;
		}
	}
	else if(filename.find("xyzsift.pcd")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyzsift)!=string::npos";
		pcl::PointCloud<PointXYZSIFT>::Ptr cloud_xyzsift (new pcl::PointCloud<PointXYZSIFT>);
		if (pcl::io::loadPCDFile<PointXYZSIFT> (filename, *cloud_xyzsift) == -1){
			LOG(LERROR) <<"Cannot read PointXYZSIFT cloud from "<<Name;
			return;
		}else{
			data = cloud_xyzsift;
		//	out_cloud_xyzsift.write(cloud_xyzsift);
			LOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<Name;
		}
	}
	else if(filename.find("xyzrgbsift.pcd")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyzrgbsift)!=string::npos";
		pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloud_xyzrgbsift (new pcl::PointCloud<PointXYZRGBSIFT>);
		if (pcl::io::loadPCDFile<PointXYZRGBSIFT> (filename, *cloud_xyzrgbsift) == -1){
			LOG(LERROR) <<"Cannot read PointXYZSIFT cloud from "<<Name;
			return;
		}else{
			data = cloud_xyzrgbsift;
		//	out_cloud_xyzsift.write(cloud_xyzsift);
			LOG(LINFO) <<"PointXYZSIFT cloud loaded properly from "<<Name;
		}
	}
	// SHOT
	else if(filename.find("xyzshot")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyzshot)!=string::npos";
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<PointXYZSHOT>::Ptr cloud_xyzshot (new pcl::PointCloud<PointXYZSHOT>);
		if (pcl::io::loadPCDFile<PointXYZSHOT> (filename, *cloud_xyzshot) == -1){
			LOG(LERROR) <<"Cannot read PointXYZSHOT cloud from "<<Name;
			return;
		}else{
			data = cloud_xyzshot;
		//	out_cloud_xyz.write(cloud_xyz);
			LOG(LINFO) <<"PointXYZSHOT cloud loaded properly from "<<Name;
		}
	}
	//NORMALS
	else if(filename.find("xyzrgbnnormal")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyzrgbnnormal)!=string::npos";
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_xyzrgbnnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filename, *cloud_xyzrgbnnormal) == -1){
			LOG(LERROR) <<"Cannot read PointXYZRGBNormal cloud from "<<Name;
			return;
		}else{
			data = cloud_xyzrgbnnormal;
		//	out_cloud_xyz.write(cloud_xyz);
			LOG(LINFO) <<"PointXYZRGBNormal cloud loaded properly from "<<Name;
		}
	}
	else if(filename.find("xyz")!=string::npos)
	{
		LOG(LDEBUG)<<"filename.find(xyz)!=string::npos";
		// Try to read the cloud of XYZ points.
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_xyz) == -1){
			LOG(LERROR) <<"Cannot read PointXYZ cloud from "<<Name;
			return;
		}else{
			data = cloud_xyz;
		//	out_cloud_xyz.write(cloud_xyz);
			LOG(LINFO) <<"PointXYZ cloud loaded properly from "<<Name;
		}
	}
	else
	{
		LOG(LERROR) <<"Unsupported cloud type! Please add few lines of the code";
		exit(1);
	}
}

void PrimitiveFile::writeToSinkFromFile(string& path)
{
	LOG(LDEBUG)<<"PrimitiveFile::writeToSink";
	LOG(LDEBUG)<<"fileType : "<<fileType;
	string fileNm = path;
	if(fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask
		|| fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured
		|| fileType==StereoRightTextured)
	{
		LOG(LNOTICE)<<"Insert to data!!!";
		LOG(LNOTICE)<<"fileNm :"<< fileNm;
		cv::Mat image = imread(fileNm, CV_LOAD_IMAGE_UNCHANGED);
		data = image;
	}
	else if(fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift
				|| fileType==PCXyzShot || fileType==PCXyzRgbNormal)
	{
		LOG(LDEBUG)<<"ReadPCDCloud";
		ReadPCDCloud(fileNm);
	}
	else if(fileType==ImageXyz)
	{
		LOG(LDEBUG)<<"fileType==ImageXyz";
		FileStorage fs2(fileNm, FileStorage::READ);
		cv::Mat imageXYZ;
		fs2["img"] >> imageXYZ;
		data = imageXYZ;
	}
	else if(fileType==FileCameraInfo)
	{
		LOG(LDEBUG)<<"xml :)";
		string CIPFile;
		char const* charFileName = fileNm.c_str();
		LOG(LDEBUG)<<fileNm;
		std::ifstream t(charFileName);
		std::stringstream buffer;
		buffer << t.rdbuf();
		CIPFile = buffer.str();
	//	cipFileOut.write(CIPFile);
		data = CIPFile;
		LOG(LDEBUG)<<CIPFile;
	}
	else
	{
		LOG(LERROR)<<"Unsupported file type!";
		exit(1);
	}
}

void PrimitiveFile::getFileFromGrid(const GridFile& file, string& path)
{
	LOG(LDEBUG)<<"PrimitiveFile::getFileFromGrid";
	LOG(LDEBUG)<<"path : "<<path;
	stringstream ss;
	string str = ss.str();
	string name;
	if(path=="")
		name = Name;
	else
		name = path+"/"+Name;

	LOG(LNOTICE)<<"name : "<<name;
	ofstream ofs((char*)name.c_str());
	gridfs_offset off = file.write(ofs);
	if (off != file.getContentLength())
	{
		LOG(LNOTICE) << "\nFailed to read a file from mongoDB\n";
	}
	else
	{
		LOG(LNOTICE) << "\nSuccess read a file from mongoDB\n";
	}
}

void PrimitiveFile::readPointCloudFromDocument(bool saveToDiscFlag, string& pathToFiles)
{
	LOG(LNOTICE)<<"Read Point Cloud";
	LOG(LNOTICE)<<"Name: " << Name;
	try
	{
		if(fileType==PCXyz)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)BSONDocument[Name].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LDEBUG)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointXYZ pt;
			for(int i=0; i<totalSize/(xyzPointSize*sizeof(float)); i++)
			{
				pt.x=newBuffer[i*xyzPointSize];
				pt.y=newBuffer[i*xyzPointSize+1];
				pt.z=newBuffer[i*xyzPointSize+2];
				cloudXYZ->push_back(pt);
			}
			data = cloudXYZ;
			if(saveToDiscFlag)
				pcl::io::savePCDFile(pathToFiles+"/"+Name, *cloudXYZ, false);
		}
		else if(fileType==PCXyzRgb)
		{
			// read data to buffer
			int totalSize;
			LOG(LNOTICE)<<"Name: " << Name;
			float* buffer = (float*)BSONDocument[Name].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LDEBUG)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB (new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointXYZRGB pt;
			for(int i=0; i<totalSize/(xyzrgbPointSize*sizeof(float)); i++)
			{
				pt.x=newBuffer[i*xyzrgbPointSize];
				pt.y=newBuffer[i*xyzrgbPointSize+1];
				pt.z=newBuffer[i*xyzrgbPointSize+2];
				pt.rgb=newBuffer[i*xyzrgbPointSize+3];
				cloudXYZRGB->push_back(pt);
			}
			data = cloudXYZRGB;
			if(saveToDiscFlag)
				pcl::io::savePCDFile(pathToFiles+"/"+Name, *cloudXYZRGB, false);
		}
		else if(fileType==PCXyzSift)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)BSONDocument[Name].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LDEBUG)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			// for sift row size in float is equal 128+5 =133  floats
			pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT (new pcl::PointCloud<PointXYZSIFT>);
			PointXYZSIFT pt;
			for(int i=0; i<totalSize/(siftPointSize*sizeof(float)); i++)
			{
				Eigen::Vector3f pointCoordinates;
				pointCoordinates[0]=newBuffer[i*siftPointSize];
				pointCoordinates[1]=newBuffer[i*siftPointSize+1];
				pointCoordinates[2]=newBuffer[i*siftPointSize+2];
				memcpy(&pt.multiplicity, &newBuffer[3+i*siftPointSize], sizeof(int)); // 4 bytes
				memcpy(&pt.pointId, &newBuffer[4+i*siftPointSize], sizeof(int));	// 4 bytes
				memcpy(&pt.descriptor, &newBuffer[5+i*siftPointSize], 128*sizeof(float)); // 128 * 4 bytes = 512 bytes

				pt.getVector3fMap() = pointCoordinates;
				cloudXYZSIFT->push_back(pt);
			}
			data = cloudXYZSIFT;
			if(saveToDiscFlag)
				pcl::io::savePCDFile(pathToFiles+"/"+Name, *cloudXYZSIFT, false);
		}
		else if(fileType==PCXyzRgbSift)
		{
			LOG(LERROR)<<"Usupported! Please add few lines of the code";
			exit(1);
		}
		else if(fileType==PCXyzShot)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)BSONDocument[Name].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LDEBUG)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);

			pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT (new pcl::PointCloud<PointXYZSHOT>);
			PointXYZSHOT pt;
			for(int i=0; i<totalSize/(shotPointSize*sizeof(float)); i++)
			{
				Eigen::Vector3f pointCoordinates;
				pointCoordinates[0]=newBuffer[i*shotPointSize];
				pointCoordinates[1]=newBuffer[i*shotPointSize+1];
				pointCoordinates[2]=newBuffer[i*shotPointSize+2];
				memcpy(&pt.descriptor, &newBuffer[3+i*shotPointSize], 352*sizeof(float));
				memcpy(&pt.rf, &newBuffer[4+i*shotPointSize], 9*sizeof(float));
				memcpy(&pt.multiplicity, &newBuffer[5+i*shotPointSize], sizeof(int));
				memcpy(&pt.pointId, &newBuffer[6+i*shotPointSize], sizeof(int));

				pt.getVector3fMap() = pointCoordinates;
				cloudXYZSHOT->push_back(pt);
			}
			data = cloudXYZSHOT;
			if(saveToDiscFlag)
				pcl::io::savePCDFile(pathToFiles+"/"+Name, *cloudXYZSHOT, false);
		}
		else if(fileType==PCXyzRgbNormal)
		{
			// read data to buffer
			int totalSize;
			float* buffer = (float*)BSONDocument[Name].binData(totalSize);
			int bufferSize = totalSize/sizeof(float);
			LOG(LDEBUG)<<"bufferSize: "<<bufferSize;
			float newBuffer[bufferSize];
			memcpy(newBuffer, buffer, totalSize);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZRGBNormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::PointXYZRGBNormal pt;
			for(int i=0; i<totalSize/(normalPointSize*sizeof(float)); i++)
			{
				pt.x=newBuffer[i*normalPointSize];
				pt.y=newBuffer[i*normalPointSize+1];
				pt.z=newBuffer[i*normalPointSize+2];
				pt.data[3]=newBuffer[i*normalPointSize+3];
				cloudXYZRGBNormal->push_back(pt);
			}
			data = cloudXYZRGBNormal;
			if(saveToDiscFlag)
				pcl::io::savePCDFile(pathToFiles+"/"+Name, *cloudXYZRGBNormal, false);
		}
	}catch(Exception &ex)
	{
		LOG(LERROR)<<ex.what();
	}
}


void PrimitiveFile::setMime( const fileTypes fileType,  string& mime)
{
	LOG(LDEBUG)<<"fileType : "<<fileType<<std::endl;

	if (fileType==ImageRgb || fileType==ImageDepth || fileType==ImageIntensity || fileType==ImageMask || fileType==StereoLeft || fileType==StereoRight || fileType==StereoLeftTextured || fileType==StereoRightTextured)
		mime="image/png";
	else if(fileType==FileCameraInfo || fileType==PCXyz || fileType==PCXyzRgb || fileType==PCXyzSift || fileType==PCXyzRgbSift || fileType==PCXyzShot ||fileType==PCXyzRgbNormal)
		mime="text/plain";
	else if(fileType==ImageXyz )
			mime="text/plain";
	else
	{
		LOG(LERROR) <<"I don't know such typeFile! Please add extension to the `if` statement from http://www.sitepoint.com/web-foundations/mime-types-complete-list/";
		return;
	}
}

void PrimitiveFile::getCVMatData(cv::Mat& img)
{
	img =  boost::get<cv::Mat>(data);
}

void PrimitiveFile::getStringData(std::string& str)
{
	str =  boost::get<std::string>(data);
}

void PrimitiveFile::getXYZData(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudXYZ)
{
	cloudXYZ = boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBData(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloudXYZRGB)
{
	LOG(LDEBUG)<<"getXYZRGBData";
	cloudXYZRGB = boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
}

void PrimitiveFile::getXYZSIFTData(pcl::PointCloud<PointXYZSIFT>::Ptr& cloudXYZSIFT)
{
	cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBSIFTData(pcl::PointCloud<PointXYZRGBSIFT>::Ptr& cloudXYZRGBSIFT)
{
	cloudXYZRGBSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);
}

void PrimitiveFile::getXYZSHOTData(pcl::PointCloud<PointXYZSHOT>::Ptr& cloudXYZSHOT)
{
	cloudXYZSHOT = boost::get<pcl::PointCloud<PointXYZSHOT>::Ptr>(data);
}

void PrimitiveFile::getXYZRGBNormalData(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& cloudXYZNormal)
{
	cloudXYZNormal = boost::get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(data);
}

void PrimitiveFile::saveImage(OID& oid, string& name, string& type)
{
	LOG(LDEBUG)<<"Insert image";
	cv::Mat* img =  boost::get<cv::Mat>(&data);
	std::vector<uchar> buf;
	std::vector<int> params(2);

	if(Name.find(".jpg")!=string::npos)
	{
		params[0] = CV_IMWRITE_JPEG_QUALITY;
		params[1] = 95;
		cv::imencode(".jpg", *img, buf, params);
	}
	else if(Name.find(".png")!=string::npos)
	{
		params[0] = CV_IMWRITE_PNG_COMPRESSION;
		params[1] = 9;
		cv::imencode(".png", *img, buf, params);
	}
	else
	{
		LOG(LERROR)<<"File extension is unsupported!";
		exit(1);
	}
	BSONObj b=BSONObjBuilder().genOID().appendBinData(Name, buf.size(), mongo::BinDataGeneral, &buf[0]).append("Name", Name).append("Type", "File").append(type+"Name", name).append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
	BSONElement bsonElement;
	b.getObjectID(bsonElement);
	oid=bsonElement.__oid();
	MongoProxy::MongoProxy::getSingleton(hostname).insert(b);

}
void PrimitiveFile::copyXYZPointToFloatArray (const pcl::PointXYZ &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
}

void PrimitiveFile::copyXYZRGBPointToFloatArray (const pcl::PointXYZRGB &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.rgb;	// 4 bytes
}

void PrimitiveFile::copyXYZSiftPointToFloatArray (const PointXYZSIFT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	memcpy(&out[3], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[4], &p.pointId, sizeof(int));	// 4 bytes
	memcpy(&out[5], &p.descriptor, 128*sizeof(float)); // 128 * 4 bytes = 512 bytes
}

//TODO check this method
void PrimitiveFile::copyXYZRGBNormalPointToFloatArray (const pcl::PointXYZRGBNormal &p, float * out) const
{
	out[0] = p.x;	// 4 bytes
	out[1] = p.y;	// 4 bytes
	out[2] = p.z;	// 4 bytes
	out[3] = p.data[3];	// 4 bytes
	/*
	out[4] = p.normal_x;	// 4 bytes
	out[5] = p.normal_y;	// 4 bytes
	out[6] = p.normal_z;	// 4 bytes
	out[7] = p.data_n[3];	// 4 bytes
	out[8] = p.curvature;	// 4 bytes
	out[9] = p.rgba;	// 4 bytes
	*/
}

void PrimitiveFile::copyXYZSHOTPointToFloatArray (const PointXYZSHOT &p, float * out) const
{
	Eigen::Vector3f outCoordinates = p.getArray3fMap();
	out[0] = outCoordinates[0];	// 4 bytes
	out[1] = outCoordinates[1];	// 4 bytes
	out[2] = outCoordinates[2];	// 4 bytes
	memcpy(&out[3], &p.descriptor, 352*sizeof(float)); // 352 * 4 bytes = 1408 bytes
	memcpy(&out[4], &p.rf, 9*sizeof(float));//9*4=36 bytes
	memcpy(&out[5], &p.multiplicity, sizeof(int)); // 4 bytes
	memcpy(&out[6], &p.pointId, sizeof(int));	// 4 bytes

}

void PrimitiveFile::insertFileIntoDocument(OID& oid, string& name, string& type, int mean_viewpoint_features_number)
{
	// use variant here...
	LOG(LDEBUG)<<"insertFileIntoDocument";
	BSONObjBuilder builder;
	BSONObj b;
	BSONElement bsonElement;

	switch(fileType)
	{
		case FileCameraInfo:
		{
			LOG(LDEBUG)<<"xml file";
			string* input =  boost::get<std::string>(&data);
			LOG(LDEBUG)<<"Input:"<< input;
			// convert to char*
			char const *cipCharTable = input->c_str();
			LOG(LDEBUG)<<string(cipCharTable);
			// create bson object
			b = BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, BinDataGeneral,  cipCharTable).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();

			// insert object into collection
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);

			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			break;
		}
		case ImageRgb:
		{
			saveImage(oid, name, type);
			break;
		}
		case ImageDepth:
		{
			saveImage(oid, name, type);
			break;
		}
		case ImageIntensity:
		{
			saveImage(oid, name, type);
			break;
		}
		case ImageMask:
		{
			saveImage(oid, name, type);
			break;
		}
		case StereoLeft:
		{
			saveImage(oid, name, type);
			break;
		}
		case StereoRight:
		{
			saveImage(oid, name, type);
			break;
		}
		case StereoLeftTextured:
		{
			saveImage(oid, name, type);
			break;
		}
		case StereoRightTextured:
		{
			saveImage(oid, name, type);
			break;
		}
		case PCXyz:
		{
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ =  boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr >(data);
			int cloudSize = cloudXYZ->size();
			int fieldsNr = xyzPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZ p = cloudXYZ->points[iter];
				copyXYZPointToFloatArray (p, &buff[xyzPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buff[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzRgb:
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB =  boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
			int cloudSize = cloudXYZRGB->size();
			int fieldsNr = xyzrgbPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZRGB p = cloudXYZRGB->points[iter];
				copyXYZRGBPointToFloatArray (p, &buff[xyzrgbPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buff[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzSift:
		{
			pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);

			int cloudSize = cloudXYZSIFT->size();
			int fieldsNr = siftPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];
			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const PointXYZSIFT p = cloudXYZSIFT->points[iter];
				copyXYZSiftPointToFloatArray (p, &buff[siftPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buff[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzRgbSift:
		{
			LOG(LDEBUG)<<"ERROR: I don't know what to do with pc_xyzrgbsift";
			exit(1);
			//pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRGBSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);;
			break;
		}
		case PCXyzShot:
		{
			pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT = boost::get<pcl::PointCloud<PointXYZSHOT>::Ptr>(data);

			int cloudSize = cloudXYZSHOT->size();
			int fieldsNr = shotPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];

			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const PointXYZSHOT p = cloudXYZSHOT->points[iter];
				copyXYZSHOTPointToFloatArray(p, &buff[shotPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buff[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case PCXyzRgbNormal:
		{
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZRGBNormal = boost::get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(data);
			int cloudSize = cloudXYZRGBNormal->size();
			int fieldsNr = normalPointSize*cloudSize;
			//int totalSize = fieldsNr*sizeof(float);
			float buff[fieldsNr];
			// copy all points to memory
			for(int iter=0; iter<cloudSize; iter++)
			{
				const pcl::PointXYZRGBNormal p = cloudXYZRGBNormal->points[iter];
				copyXYZRGBNormalPointToFloatArray(p, &buff[normalPointSize*iter]);
			}
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buff[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).obj();
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
		case ImageXyz:
		{
			cv::Mat* xyzimage = boost::get<cv::Mat>(&data);
			//int bufSize = xyzimage.total()*xyzimage.channels()*4;
			int width = xyzimage->size().width;
			int height = xyzimage->size().height;
			int channels = xyzimage->channels();
			float* buf;
			buf = (float*)xyzimage->data;
			LOG(LDEBUG)<<"Set buffer YAML";
			b=BSONObjBuilder().genOID().appendBinData(Name, (int)sizeBytes, mongo::BinDataGeneral, &buf[0]).append("Name", Name).append(type+"Name", name).append("Type", "File").append("size", (int)sizeBytes).append("place", "document").append("fileType", FTypes[fileType]).append("width", width).append("height", height).append("channels", channels).obj();
			LOG(LDEBUG)<<"YAML inserted";
			b.getObjectID(bsonElement);
			oid=bsonElement.__oid();
			MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
			break;
		}
	}
}

void PrimitiveFile::savePCxyzRGBNormalFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudXYZNormal = boost::get<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgbNormal.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZNormal, binary);
}


void PrimitiveFile::savePCxyzSHOTFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<PointXYZSHOT>::Ptr cloudXYZSHOT = boost::get<pcl::PointCloud<PointXYZSHOT>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgbshot.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSHOT, binary);
}

void PrimitiveFile::savePCxyzFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudXYZ = boost::get<pcl::PointCloud<pcl::PointXYZ>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyz.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZ, binary);
}

void PrimitiveFile::savePCxyzRGBFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudXYZRGB = boost::get<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgb.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRGB, binary);
}

void PrimitiveFile::savePCxyzSIFTFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<PointXYZSIFT>::Ptr cloudXYZSIFT = boost::get<pcl::PointCloud<PointXYZSIFT>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzsift.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZSIFT, binary);
}

void PrimitiveFile::savePCxyzRGBSIFTFileToDisc(bool suffix, bool binary, std::string& fn)
{
	pcl::PointCloud<PointXYZRGBSIFT>::Ptr cloudXYZRBGSIFT = boost::get<pcl::PointCloud<PointXYZRGBSIFT>::Ptr>(data);
	if(suffix)
	{
		size_t f = fn.find(".pcd");
		if(f != std::string::npos)
		{
			fn.erase(f);
		}
		fn = std::string(fn) + std::string("_xyzrgbsift.pcd");
	}
	LOG(LDEBUG) <<"FileName:"<<fn;
	pcl::io::savePCDFile(fn, *cloudXYZRBGSIFT, binary);
}

void PrimitiveFile::saveImageOnDisc()
{
	LOG(LTRACE)<<"Save image to disc";
	cv::Mat* img = boost::get<cv::Mat>(&data);
	cv::imwrite(Name, *img);
}

void PrimitiveFile::saveToDisc(bool suffix, bool binary)
{
	switch(fileType)
	{
		case FileCameraInfo:
		{
			std::string* cameraMatrix = boost::get<std::string>(&data);
			char const* ca = Name.c_str();
			std::ofstream out(ca);
			out<<*cameraMatrix;
			out.close();
			break;
		}
		case ImageRgb:
		{
			LOG(LTRACE)<<"RGB";
			saveImageOnDisc();
			break;
		}
		case ImageDepth:
		{
			saveImageOnDisc();
			break;
		}
		case ImageIntensity:
		{
			saveImageOnDisc();
			break;
		}
		case ImageMask:
		{
			saveImageOnDisc();
			break;
		}
		case StereoLeft:
		{
			saveImageOnDisc();
			break;
		}
		case StereoRight:
		{
			saveImageOnDisc();
			break;
		}
		case StereoLeftTextured:
		{
			saveImageOnDisc();
			break;
		}
		case StereoRightTextured:
		{
			saveImageOnDisc();
			break;
		}
		case PCXyz:
		{
			savePCxyzFileToDisc(suffix, binary, Name);
			break;
		}
		case PCXyzRgb:
		{
			savePCxyzRGBFileToDisc(suffix, binary, Name);
			break;
		}
		case PCXyzSift:
		{
			savePCxyzSIFTFileToDisc(suffix, binary, Name);
			break;
		}
		case PCXyzRgbSift:
		{
			savePCxyzRGBSIFTFileToDisc(suffix, binary, Name);
			break;
		}
		case PCXyzShot:
		{
			savePCxyzSHOTFileToDisc(suffix, binary, Name);
			break;
		}
		case PCXyzRgbNormal:
		{
			savePCxyzRGBNormalFileToDisc(suffix, binary, Name);
			break;
		}
		case ImageXyz:
		{
			cv::Mat* xyzimage = boost::get<cv::Mat>(&data);
			cv::FileStorage fs(Name, cv::FileStorage::WRITE);
			fs << "img" << *xyzimage;
			fs.release();
			break;
		}
		default:
		{
			LOG(LERROR)<<"I don't know such file type :( \nBye!";
			exit(1);
		}
	}
}
void PrimitiveFile::getTimestamp(string& timestamp)
{
	time_t tempTime;
	time(&tempTime);
	struct timeval tv;
	gettimeofday(&tv, NULL);
	std::ostringstream oss;
	oss << tv.tv_sec;
	oss << tv.tv_usec;
	timestamp =  oss.str();
	LOG(LDEBUG)<<"timestamp : "<<timestamp;
	return;
}

void PrimitiveFile::insertFileIntoGrid(OID& oid, bool dataInBuffer, string& path, string& type, string& name, int mean_viewpoint_features_number)
{
	LOG(LDEBUG)<<"Writting to GRIDFS!";
	try{
		BSONObj object;
		BSONElement bsonElement;
		string mime="";
		setMime(fileType, mime);
		bool suffix = true;
		bool binary = false;
		if(dataInBuffer)
			saveToDisc(suffix, binary);

		if(path=="")
			path=Name;
		boost::shared_ptr<DBClientConnection>  c = MongoProxy::MongoProxy::getSingleton(hostname).getClient();
		// create GridFS client
		GridFS fs(*c, MongoProxy::MongoProxy::getSingleton(hostname).collectionName);

		// save in grid
		object = fs.storeFile(path, Name, mime);

		BSONObj b;
		LOG(LDEBUG)<<"fileType: "<<fileType;
		if(type=="View")
			b = BSONObjBuilder().appendElements(object).append("ViewName", name).append("Name", Name).append("fileType", FTypes[fileType]).append("Type", "File").append("size", (int)sizeBytes).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		else if(type=="Model")
			b = BSONObjBuilder().appendElements(object).append("ModelName", name).append("Name", Name).append("fileType", FTypes[fileType]).append("Type", "File").append("size", (int)sizeBytes).append("place", "grid").append("mean_viewpoint_features_number", mean_viewpoint_features_number).obj();
		else
		{
			LOG(LERROR)<<"Couldn't update type: "<<type<<" !!!";
		}
		MongoProxy::MongoProxy::getSingleton(hostname).insert(b);
		b.getObjectID(bsonElement);
		oid=bsonElement.__oid();
		LOG(LDEBUG)<<"OID :"<<oid;
		string field = "Name";
		MongoProxy::MongoProxy::getSingleton(hostname).index(field);
	}catch(DBException &e)
	{
		LOG(LERROR) <<"Something goes wrong... :<\nBye!";
		//LOG(LDEBUG) <<c->getLastError();
		LOG(LERROR) << e.what();
	}
	const char* file_name = Name.c_str();
	std::remove(file_name);
}

}//namespace File
}//MongoBase


#endif /* PRIMITIVEFILE_HPP_ */
