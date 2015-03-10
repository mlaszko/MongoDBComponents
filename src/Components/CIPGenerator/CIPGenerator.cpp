/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "CIPGenerator.hpp"
// ogarnac yaml i zrobic stworzyc sinka bedacego yamlem

namespace Processors {
namespace CIPGenerator  {
using namespace cv;

CIPGenerator::CIPGenerator(const std::string & name) : Base::Component(name)
{
        CLOG(LTRACE) << "Hello CIPGenerator";
}

CIPGenerator::~CIPGenerator()
{
        CLOG(LTRACE) << "Good bye CIPGenerator";
}

void CIPGenerator::prepareInterface() {
        CLOG(LTRACE) << "SceneReader::prepareInterface";

        h_generateCIP.setup(this, &CIPGenerator::generateCIP);
        registerHandler("generateCIP", &h_generateCIP);
        registerStream("CIPString", &CIPString);
}

bool CIPGenerator::onInit()
{
        CLOG(LTRACE) << "CIPGenerator::initialize";
		return true;
}

bool CIPGenerator::onFinish()
{
        CLOG(LTRACE) << "CIPGenerator::finish";
        return true;
}

bool CIPGenerator::onStep()
{
        CLOG(LTRACE) << "CIPGenerator::step";
        return true;
}

bool CIPGenerator::onStop()
{
        return true;
}

bool CIPGenerator::onStart()
{
        return true;
}

void CIPGenerator::generateCIP()
{
	//YAML::Node node;// = YAML::LoadFile("config.yaml");
	//config.push_back("Matrix");
	cv::Mat projectionMatrix = cv::Mat::eye(4, 4, CV_32FC1);
	cv::Size size = projectionMatrix.size();
	int rows = size.height;
	int cols = size.width;
	std::ostringstream buff;
	string stringToOutput = "";
	CLOG(LINFO)<<"COLS: "<<cols<<" Rows: "<<rows;
	stringToOutput = "Projection Matrix\n";
	float number;
	for(int i=0; i<cols;i++)
	{
		for(int j=0; j<rows; j++)
		{
			CLOG(LINFO)<<projectionMatrix.at<float>(j,i);
			number=(float)projectionMatrix.at<float>(j,i);
			buff.str("");
			buff<<number;
			stringToOutput+=buff.str();
			CLOG(LINFO)<<stringToOutput;

			if(j!=rows-1)
				stringToOutput+=",";
		}
		if(i!=cols-1)
			stringToOutput+=";";
	}
	CLOG(LINFO)<<stringToOutput;

	//node["Matrix"] = stringToOutput;  // it now is a map node
	//"{Matrix:"+stringToOutput+"}"
	//node= YAML::Load("{Matrix:dupa}");
	CIPString.write(stringToOutput);
}

} //: namespace SceneReader
} //: namespace Processors
