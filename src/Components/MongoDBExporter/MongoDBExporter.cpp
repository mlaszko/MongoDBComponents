/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoDBExporter.hpp"

namespace Processors {
namespace MongoDBExporter  {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;

MongoDBExporter::MongoDBExporter(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	description("description", string("My green coffe cup")),
	viewsSet("viewsSet", string("viewsSet1")),
	SensorType("SensorType", string("Stereo")),
	folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial")),
	in_mean_viewpoint_features_number("in_mean_viewpoint_features_number", int(10)),
	sceneNameProp("sceneNamesProp", string("scene1"))
{
	registerProperty(mongoDBHost);
	registerProperty(in_mean_viewpoint_features_number);
	registerProperty(description);
	registerProperty(viewsSet);
	registerProperty(SensorType);
	registerProperty(folderName);
	registerProperty(sceneNameProp);
	fileExtensions.push_back("*.png");
	fileExtensions.push_back("*.jpg");
	fileExtensions.push_back("*.txt");
	fileExtensions.push_back("*.pcd");
	fileExtensions.push_back("*.yaml");
	fileExtensions.push_back("*.yml");
	hostname = mongoDBHost;

	CLOG(LTRACE) << "Hello MongoDBExporter";
}

MongoDBExporter::~MongoDBExporter()
{
	CLOG(LTRACE) << "Good bye MongoDBExporter";
}

vector<string> MongoDBExporter::getAllFiles(const string& pattern)
{
	glob_t glob_result;
	glob(pattern.c_str(),GLOB_TILDE,NULL,&glob_result);
	vector<string> files;
	for(unsigned int i=0;i<glob_result.gl_pathc;++i){
	    files.push_back(string(glob_result.gl_pathv[i]));
	}
	globfree(&glob_result);
	return files;
}

vector<string> MongoDBExporter::getAllFolders(const string& directoryPath)
{
	vector<string> directories;
	const char *cstr = directoryPath.c_str();
	DIR *dir = opendir(cstr);
    struct dirent *entry = readdir(dir);
    while (entry != NULL)
    {
    	if (entry->d_type == DT_DIR)
    		directories.push_back(entry->d_name);
    	entry = readdir(dir);
    }
    closedir(dir);
    return directories;
}


void MongoDBExporter::write2DB()
{
	CLOG(LNOTICE) << "MongoDBExporter::write2DB";
	string type;// model or view
	// set source folder name
	string directory = folderName;
	string viewFolders;
	std::vector<std::string> folders = getAllFolders(directory);
	std::string sceneName = sceneNameProp;
	for(std::vector<string>::iterator itfolders = folders.begin(); itfolders != folders.end(); ++itfolders)
	{
		if(itfolders->find(".") != std::string::npos || itfolders->find("..") != std::string::npos)
			continue;
		else if (itfolders->find("view") != std::string::npos || itfolders->find("View") != std::string::npos )
		{
			type = "View";
			string vn = string(*itfolders);
			viewPtr = boost::shared_ptr<View>(new View(vn,hostname));
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
				continue;
			}
		}
		else if (itfolders->find("model") != std::string::npos || itfolders->find("Model") != std::string::npos )
		{
			type = "Model";
			string vn = string(*itfolders);
			modelPtr = boost::shared_ptr<Model>(new Model(vn,hostname));

			bool exist = modelPtr->checkIfExist();
			if(!exist)
			{
				string viewsSetName = string(viewsSet);
				modelPtr->setViewsSetNames(viewsSetName);
			}
			else
			{
				CLOG(LERROR)<<"Model exist in data base!!!";
				continue;
			}
			modelPtr->create();
		}
		else
		{
			CLOG(LINFO)<<"Folder: "<<directory+"/" + *itfolders<<" will be ommited!";
			continue;
		}
		string source  = directory+"/"+ *itfolders+"/";
		CLOG(LINFO) <<"source : "<<source;
		// get files
		CLOG(LINFO) <<"size of extensions vector:  "<<fileExtensions.size();

		for(std::vector<string>::iterator itExtension = fileExtensions.begin(); itExtension != fileExtensions.end(); ++itExtension)
		{
			CLOG(LINFO) <<"std::vector<string>::iterator itExtension = fileExtensions.begin();";
			string source  = directory+"/"+ *itfolders+"/";
			CLOG(LTRACE) <<"source+*itExtension "<<source+*itExtension;
			vector<string> files = getAllFiles(source+*itExtension);
			CLOG(LINFO) <<"size of files vector:  "<<files.size();
			for(std::vector<string>::iterator it = files.begin(); it != files.end(); ++it)
			{
				string fileName = *it;
				string newFileName;
				string fileNameTemp = fileName;

				const size_t last_slash_idx = fileName.find_last_of("/");
				if (std::string::npos != last_slash_idx)
				{
					newFileName = fileName.erase(0, last_slash_idx + 1);
				}
				LOG(LNOTICE)<<"READ FILE FROM DISC!!!";
				fileTypes ft;
				// get file type
				if (newFileName.find(".xml") != std::string::npos)
				{
					ft = FileCameraInfo;
				}
				else if (newFileName.find(".yaml") != std::string::npos || newFileName.find(".yml") != std::string::npos)
				{
					ft = ImageXyz;
				}
				else if (newFileName.find("_rgb.png") != std::string::npos || newFileName.find("_rgb.jpg") != std::string::npos)
				{
					ft = ImageRgb;
				}
				else if (newFileName.find("_depth.png") != std::string::npos || newFileName.find("_depth.jpg") != std::string::npos)
				{
					ft = ImageDepth;
				}
				else if (newFileName.find("_intensity.png") != std::string::npos || newFileName.find("_intensity.jpg") != std::string::npos)
				{
					ft = ImageIntensity;
				}
				else if (newFileName.find("_mask.png") != std::string::npos || newFileName.find("_mask.jpg") != std::string::npos)
				{
					ft = ImageMask;
				}
				else if (newFileName.find("_stereoL.png") != std::string::npos || newFileName.find("_stereoL.jpg") != std::string::npos)
				{
					ft = StereoLeft;
				}
				else if (newFileName.find("_stereoR.png") != std::string::npos || newFileName.find("_stereoR.jpg") != std::string::npos)
				{
					ft = StereoRight;
				}
				else if (newFileName.find("_stereoLTXD.png") != std::string::npos || newFileName.find("_stereoLTXD.jpg") != std::string::npos)
				{
					ft = StereoLeftTextured;
				}
				else if (newFileName.find("_stereoRTXD.png") != std::string::npos || newFileName.find("_stereoRTXD.jpg") != std::string::npos)
				{
					ft = StereoRightTextured;
				}
				else if (newFileName.find("_xyz.pcd") != std::string::npos)
				{
					ft = PCXyz;
				}
				else if (newFileName.find("_xyzrgb.pcd") != std::string::npos)
				{
					ft = PCXyzRgb;
				}
				else if (newFileName.find("_xyzsift.pcd") != std::string::npos)
				{
					ft = PCXyzSift;
				}
				else if (newFileName.find("_xyzrgbsift.pcd") != std::string::npos)
				{
					ft = PCXyzRgbSift;
				}
				else if (newFileName.find("_xyzshot.pcd") != std::string::npos)
				{
					ft = PCXyzShot;
				}
				else if (newFileName.find("_xyzrgbnormal") != std::string::npos)
				{
					ft = PCXyzRgbNormal;
				}
				else
				{
					CLOG(LERROR)<<"Unsupported file : " << newFileName<<" !!!";
					continue;
				}
				CLOG(LNOTICE)<<"ft : "<< ft;
				shared_ptr<PrimitiveFile::PrimitiveFile> file(new PrimitiveFile::PrimitiveFile(newFileName, ft, hostname));
				float size = get_file_size(fileNameTemp);
				file->setSize(size);

				if(file->getSize()<15)
				{
					file->writeToSinkFromFile(fileNameTemp);
				}
				if(type == "Model")
					modelPtr->addFile(file, type, false, fileNameTemp, in_mean_viewpoint_features_number);
				else if(type == "View")
					viewPtr->addFile(file, type, false, fileNameTemp, in_mean_viewpoint_features_number);
				else
				{
					CLOG(LERROR) <<"Not view or model!!!";
					exit(1);
				}
			}//for
		}//for
	}//for
}

void MongoDBExporter::prepareInterface() {
	CLOG(LTRACE) << "MongoDBExporter::prepareInterface";
	h_write2DB.setup(this, &MongoDBExporter::write2DB);
	registerHandler("write2DB", &h_write2DB);
}

bool MongoDBExporter::onInit()
{
	CLOG(LTRACE) << "MongoDBExporter::initialize";
	MongoProxy::MongoProxy::getSingleton(hostname);
	return true;
}

bool MongoDBExporter::onFinish()
{
        CLOG(LTRACE) << "MongoDBExporter::finish";
        return true;
}

bool MongoDBExporter::onStep()
{
        CLOG(LTRACE) << "MongoDBExporter::step";
        return true;
}

bool MongoDBExporter::onStop()
{
        return true;
}

bool MongoDBExporter::onStart()
{
        return true;
}

float MongoDBExporter::get_file_size(std::string& filename) // path to file
{
    FILE *p_file = NULL;
    float size;
    try{
    	p_file = fopen(filename.c_str(),"rb");
    	fseek(p_file,0,SEEK_END);
    	size = (float)ftell(p_file);
    	fclose(p_file);
    }
    catch(Exception & ex){ex.what();}
    return size;
}

} //: namespace MongoDBExporter
} //: namespace Processors
