
#ifndef  CIPGenerator_H__
#define  CIPGenerator_H__

#include "Component_Aux.hpp"
#include "Component.hpp"
#include "DataStream.hpp"
#include "Property.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "Logger.hpp"
#include <sstream>
//#include <yaml-cpp/yaml.h>

namespace Processors {
namespace CIPGenerator {

using namespace cv;


class CIPGenerator: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		CIPGenerator(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~CIPGenerator();

        /*!
         * Prepares communication interface.
         */
        virtual void prepareInterface();
        Base::DataStreamOut<std::string> CIPString;

        void generateCIP();

protected:

        /*!
         * Connects source to given device.
         */
        bool onInit();

        /*!
         * Disconnect source from device, closes streams, etc.
         */
        bool onFinish();

        /*!
         * Retrieves data from device.
         */
        bool onStep();

        /*!
         * Start component
         */
        bool onStart();

        /*!
         * Stop component
         */
        bool onStop();


        /*!
         * Event handler function.
         */

        /// Event handler.
        Base::EventHandler <CIPGenerator> h_generateCIP;

private:

};
}//: namespace SceneReader
}//: namespace Processors

REGISTER_COMPONENT("CIPGenerator", Processors::CIPGenerator::CIPGenerator)

#endif /* CIPGenerator_H__ */
