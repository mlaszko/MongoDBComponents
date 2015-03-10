/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */


#include "MongoPerformanceTester.hpp"

#define siftPointSize 133
#define xyzPointSize 3
#define xyzrgbPointSize 4

namespace Processors {
namespace MongoPerformanceTester {
using namespace cv;
using namespace mongo;
using namespace std;
using namespace boost;
using namespace MongoDB;
using namespace MongoProxy;
using namespace PrimitiveFile;
using namespace boost::chrono;

MongoPerformanceTester::MongoPerformanceTester(const string & name) : Base::Component(name),
	mongoDBHost("mongoDBHost", string("localhost")),
	description("description", string("My green coffe cup")),
	viewsSet("viewsSet", string("viewsSet1")),
	SensorType("SensorType", string("Stereo")),
	folderName("folderName", string("/home/lzmuda/mongo_driver_tutorial")),
	iterations("iterations", int(1)),
	sceneNameProp("sceneNamesProp", string("scene1"))
{
	registerProperty(mongoDBHost);
	registerProperty(description);
	registerProperty(viewsSet);
	registerProperty(SensorType);
	registerProperty(folderName);
	registerProperty(sceneNameProp);
	registerProperty(iterations);
	fileExtensions.push_back("*.png");
	fileExtensions.push_back("*.jpg");
	fileExtensions.push_back("*.txt");
	fileExtensions.push_back("*.pcd");
	fileExtensions.push_back("*.yaml");
	fileExtensions.push_back("*.yml");
	hostname = mongoDBHost;

	CLOG(LTRACE) << "Hello MongoPerformanceTester";
}

MongoPerformanceTester::~MongoPerformanceTester()
{
	CLOG(LTRACE) << "Good bye MongoPerformanceTester";
}

vector<string> MongoPerformanceTester::getAllFiles(const string& pattern)
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

vector<string> MongoPerformanceTester::getAllFolders(const string& directoryPath)
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


void MongoPerformanceTester::write2DB()
{

	int i=0;
	thread_clock::time_point ItersStart;
	ofstream myfile;
	myfile.open ("results.csv");
	myfile<<";\n";
	thread_clock::time_point start = thread_clock::now();
	while(i<iterations*100)
	{

		ItersStart = thread_clock::now();
		int iter = (int)i;
		stringstream ss;
		ss << iter;
		string strIter = ss.str();
		CLOG(LNOTICE) << "MongoPerformanceTester::write2DB";
		CLOG(LNOTICE) << "Iterations: "<<iterations;
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
				int iter = (int)iterations;
				vn = vn + strIter;
				CLOG(LNOTICE)<<"ViewName: " << vn;
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
				string mn = string(*itfolders);
				mn = mn + strIter;
				CLOG(LNOTICE)<<"ModelName: " << mn;
				modelPtr = boost::shared_ptr<Model>(new Model(mn,hostname));

				bool exist = modelPtr->checkIfExist();
				if(!exist)
				{
					//std::vector<std::string> splitedViewsSetNames;
					//boost::split(splitedViewsSetNames, viewsSetList, is_any_of(";"));
					//modelPtr->setViewsSetNames(splitedViewsSetNames);
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
			CLOG(LDEBUG)<<"type : "<<type;
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
					CLOG(LDEBUG)<<"fileName : "<<fileName;
					string fileNameTemp = fileName;

					const size_t last_slash_idx = fileName.find_last_of("/");
					if (std::string::npos != last_slash_idx)
					{
						newFileName = fileName.erase(0, last_slash_idx + 1);
					}
					LOG(LDEBUG)<<"READ FILE FROM DISC!!!";
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
					CLOG(LDEBUG)<<"fileNameTemp : "<<fileNameTemp;
					float size = get_file_size(fileNameTemp);
					file->setSize(size);

					if(file->getSize()<15)
					{
						file->writeToSinkFromFile(fileNameTemp);
					}
					string name = *itfolders+strIter;
					if(type == "Model")
						modelPtr->addFile(file, type, false, fileNameTemp, 0);
					else if(type == "View")
						viewPtr->addFile(file, type, false, fileNameTemp, 0);
					else
					{
						CLOG(LERROR) <<"Not view or model!!!";
						exit(1);
					}
				}//for
			}//for
		}//for
		//CLOG(LERROR)<<"i: "<<i;
	//	if(i!=0 && i%10==0)
	//	{
			thread_clock::time_point ItersStop = thread_clock::now();
			//CLOG(LERROR) << "10 Iterations time :"  << duration_cast<milliseconds>(Iters10Stop - Iters10Start).count() << " ms\n";
			myfile<<duration_cast<milliseconds>(ItersStop - ItersStart).count()<<";\n";
	//	}
		++i;
	}//	while(i<iterations)
	thread_clock::time_point stop = thread_clock::now();
	CLOG(LERROR) << "Full time :"  << duration_cast<milliseconds>(stop - start).count() << " ms\n";
	myfile<<duration_cast<milliseconds>(stop - start).count()<<"\n;";
	myfile.close();
}

void MongoPerformanceTester::prepareInterface() {
	CLOG(LTRACE) << "MongoPerformanceTester::prepareInterface";
	h_write2DB.setup(this, &MongoPerformanceTester::write2DB);
	registerHandler("write2DB", &h_write2DB);
}

bool MongoPerformanceTester::onInit()
{
	CLOG(LTRACE) << "MongoPerformanceTester::initialize";
	MongoProxy::MongoProxy::getSingleton(hostname);
	return true;
}

bool MongoPerformanceTester::onFinish()
{
        CLOG(LTRACE) << "MongoPerformanceTester::finish";
        return true;
}

bool MongoPerformanceTester::onStep()
{
        CLOG(LTRACE) << "MongoPerformanceTester::step";
        return true;
}

bool MongoPerformanceTester::onStop()
{
        return true;
}

bool MongoPerformanceTester::onStart()
{
        return true;
}

float MongoPerformanceTester::get_file_size(std::string& filename) // path to file
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

} //: namespace MongoPerformanceTester
} //: namespace Processors
