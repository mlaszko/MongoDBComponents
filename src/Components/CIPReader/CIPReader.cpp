/*!
 * \file
 * \brief
 * \author Lukasz Zmuda
 */

#include "CIPReader.hpp"

namespace Processors {
namespace CIPReader  {
using namespace cv;

CIPReader::CIPReader(const std::string & name) : Base::Component(name)
{
        CLOG(LTRACE) << "Hello CIPReader";
}

CIPReader::~CIPReader()
{
        CLOG(LTRACE) << "Good bye CIPReader";
}

void CIPReader::prepareInterface() {
        CLOG(LTRACE) << "SceneReader::prepareInterface";

        h_generateCIP.setup(this, &CIPReader::readCIP);
        registerHandler("generateCIP", &h_generateCIP);
        registerStream("CIPFileIn", &CIPFileIn);
}

bool CIPReader::onInit()
{
        CLOG(LTRACE) << "CIPReader::initialize";
		return true;
}

bool CIPReader::onFinish()
{
        CLOG(LTRACE) << "CIPReader::finish";
        return true;
}

bool CIPReader::onStep()
{
        CLOG(LTRACE) << "CIPReader::step";
        return true;
}

bool CIPReader::onStop()
{
        return true;
}

bool CIPReader::onStart()
{
        return true;
}

void CIPReader::readCIP()
{
	string stringToOutput = CIPFileIn.read();
	CLOG(LINFO)<<stringToOutput;
}

} //: namespace SceneReader
} //: namespace Processors
