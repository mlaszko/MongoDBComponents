
#ifndef  CIPReader_H__
#define  CIPReader_H__

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
namespace CIPReader {

using namespace cv;


class CIPReader: public Base::Component
{
public:
        /*!
         * Constructor.
         */
		CIPReader(const std::string & name = "");

        /*!
         * Destructor
         */
        virtual ~CIPReader();

        /*!
         * Prepares communication interface.
         */
        virtual void prepareInterface();
        Base::DataStreamIn<std::string> CIPFileIn;

        void readCIP();

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
        Base::EventHandler <CIPReader> h_generateCIP;

private:

};
}//: namespace SceneReader
}//: namespace Processors

REGISTER_COMPONENT("CIPReader", Processors::CIPReader::CIPReader)

#endif /* CIPReader_H__ */
