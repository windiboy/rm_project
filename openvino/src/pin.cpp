//C++
#include <string>
#include <vector>
#include <iostream>
//openvino
#include <functional>
#include <iostream>
#include <fstream>
#include <random>
#include <memory>
#include <chrono>
#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

#include <opencv2/core/core.hpp>

#include <ngraph/ngraph.hpp>
#include <inference_engine.hpp>
#include <ngraph/ngraph.hpp>
#include "../monitors/presenter.h"
#include "ocv_common.hpp"
#include "common.hpp"
#include <opencv2/imgproc/imgproc.hpp>

//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

using namespace InferenceEngine;
cv::Mat Color_pic;
bool init_flag = false;

void frameToBlob(const cv::Mat& frame,InferRequest::Ptr& inferRequest,const std::string& inputName)
{
        Blob::Ptr frameBlob = inferRequest->GetBlob(inputName);
        matU8ToBlob<uint8_t>(frame, frameBlob);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
   cv_bridge::CvImagePtr cam_img;


   try {
       cam_img = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
   }catch(cv_bridge::Exception& e){
       ROS_ERROR("cv_bridge exception: %s",e.what());
       return;
   }
   if(!cam_img->image.empty())
   {
       Color_pic = cam_img->image.clone();
       //std::cout<<"received pic!"<<std::endl;

   }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "object_detection");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber img_sub = it.subscribe("/camera/color/image_raw",1,imageCallback);
    image_transport::Publisher img_pub = it.advertise("object_detection",1);
    std::cout << "InferenceEngine: " << GetInferenceEngineVersion() << std::endl;

    std::vector<std::string> labels;
    labels.reserve(1);

    labels.push_back("pin_NG");

    cv::Mat frame ;//= Color_pic.clone();
    cv::Mat next_frame;

    int WID = 640;
    int HEI = 480;
    const size_t width  = (size_t) WID;
    const size_t height = (size_t) HEI;

    Core ie;
    CNNNetwork cnnNetwork;

    cnnNetwork = ie.ReadNetwork("/home/bcsh/workspace/src/openvino/pin_defects_model/pin_defects_detector.xml");
    cnnNetwork.setBatchSize(1);

    InputsDataMap inputInfo(cnnNetwork.getInputsInfo());

    std::string imageInputName, imageInfoInputName;
    size_t netInputHeight, netInputWidth;

    for (const auto & inputInfoItem : inputInfo) {
        if (inputInfoItem.second->getTensorDesc().getDims().size() == 4) {  // first input contains images
            imageInputName = inputInfoItem.first;
            inputInfoItem.second->setPrecision(Precision::U8);
            inputInfoItem.second->getInputData()->setLayout(Layout::NCHW);
            const TensorDesc& inputDesc = inputInfoItem.second->getTensorDesc();
            netInputHeight = getTensorHeight(inputDesc);
            netInputWidth = getTensorWidth(inputDesc);
        } else if (inputInfoItem.second->getTensorDesc().getDims().size() == 2) {  // second input contains image info
            imageInfoInputName = inputInfoItem.first;
            inputInfoItem.second->setPrecision(Precision::FP32);
        } else {
            throw std::logic_error("Unsupported " +
                                   std::to_string(inputInfoItem.second->getTensorDesc().getDims().size()) + "D "
                                   "input layer '" + inputInfoItem.first + "'. "
                                   "Only 2D and 4D input layers are supported");
        }
    }

    OutputsDataMap outputInfo(cnnNetwork.getOutputsInfo());
    if (outputInfo.size() != 1) {
        throw std::logic_error("This demo accepts networks having only one output");
    }
    DataPtr& output = outputInfo.begin()->second;
    auto outputName = outputInfo.begin()->first;

    int num_classes = 0;

    if (auto ngraphFunction = cnnNetwork.getFunction()) {
        for (const auto op : ngraphFunction->get_ops()) {
            if (op->get_friendly_name() == outputName) {
                auto detOutput = std::dynamic_pointer_cast<ngraph::op::DetectionOutput>(op);
                if (!detOutput) {
                    THROW_IE_EXCEPTION << "Object Detection network output layer(" + op->get_friendly_name() +
                        ") should be DetectionOutput, but was " +  op->get_type_info().name;
                }

                num_classes = detOutput->get_attrs().num_classes;
                break;
            }
        }
    }
    const SizeVector outputDims = output->getTensorDesc().getDims();
    const int maxProposalCount = outputDims[2];
    const int objectSize = outputDims[3];
    if (objectSize != 7) {
        throw std::logic_error("Output should have 7 as a last dimension");
    }
    if (outputDims.size() != 4) {
        throw std::logic_error("Incorrect output dimensions for SSD");
    }
    output->setPrecision(Precision::FP32);
    output->setLayout(Layout::NCHW);
    ExecutableNetwork network = ie.LoadNetwork(cnnNetwork, "CPU");

    InferRequest::Ptr async_infer_request_curr = network.CreateInferRequestPtr();

    if (!imageInfoInputName.empty()) {
        auto setImgInfoBlob = [&](const InferRequest::Ptr &inferReq) {
            auto blob = inferReq->GetBlob(imageInfoInputName);
            LockedMemory<void> blobMapped = as<MemoryBlob>(blob)->wmap();
            auto data = blobMapped.as<float *>();
            data[0] = static_cast<float>(netInputHeight);  // height
            data[1] = static_cast<float>(netInputWidth);  // width
            data[2] = 1;
        };
        setImgInfoBlob(async_infer_request_curr);
    }
    bool isAsyncMode = false;

    typedef std::chrono::duration<double, std::ratio<1, 1000>> ms;
    auto total_t0 = std::chrono::high_resolution_clock::now();
    auto wallclock = std::chrono::high_resolution_clock::now();
    double ocv_render_time = 0;

    cv::Size graphSize{(int)640 / 4, 60};
    Presenter presenter("", (int)480 - graphSize.height - 10, graphSize);



    ros::Rate loop_rate(30);
    cv::namedWindow("Object_Detection");

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
        if(Color_pic.empty()) {
            std::cout<<"pic is empty!"<<std::endl;
            continue;
        }
        if(!init_flag)
            {
               init_flag =true;
            }
        else
            {
                 frame = Color_pic.clone();
                 auto t0 = std::chrono::high_resolution_clock::now();

                 frameToBlob(frame, async_infer_request_curr, imageInputName);

                 auto t1 = std::chrono::high_resolution_clock::now();
                 double ocv_decode_time = std::chrono::duration_cast<ms>(t1 - t0).count();

                 t0 = std::chrono::high_resolution_clock::now();

                 async_infer_request_curr->StartAsync();
                 if (OK == async_infer_request_curr->Wait(IInferRequest::WaitMode::RESULT_READY)) {
                     t1 = std::chrono::high_resolution_clock::now();
                     ms detection = std::chrono::duration_cast<ms>(t1 - t0);

                     t0 = std::chrono::high_resolution_clock::now();
                     ms wall = std::chrono::duration_cast<ms>(t0 - wallclock);
                     wallclock = t0;

                     t0 = std::chrono::high_resolution_clock::now();

                     presenter.drawGraphs(frame);
                     std::ostringstream out;
                     out << "OpenCV cap/render time: " << std::fixed << std::setprecision(2)
                         << (ocv_decode_time + ocv_render_time) << " ms";
                     cv::putText(frame, out.str(), cv::Point2f(0, 25), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 255, 0));
                     out.str("");
                     out << "Wallclock time " << (isAsyncMode ? "(TRUE ASYNC):      " : "(SYNC, press Tab): ");
                     out << std::fixed << std::setprecision(2) << wall.count() << " ms (" << 1000.f / wall.count() << " fps)";
                     cv::putText(frame, out.str(), cv::Point2f(0, 50), cv::FONT_HERSHEY_TRIPLEX, 0.6, cv::Scalar(0, 0, 255));

                     out.str("");
                     out << "Detection time  : " << std::fixed << std::setprecision(2) << detection.count()
                         << " ms ("
                         << 1000.f / detection.count() << " fps)";
                     cv::putText(frame, out.str(), cv::Point2f(0, 75), cv::FONT_HERSHEY_TRIPLEX, 0.6,
                                 cv::Scalar(255, 0, 0));

                     LockedMemory<const void> outputMapped = as<MemoryBlob>(
                         async_infer_request_curr->GetBlob(outputName))->rmap();
                     const float *detections = outputMapped.as<float*>();
                     for (int i = 0; i < maxProposalCount; i++) {
                         float image_id = detections[i * objectSize + 0];
                         if (image_id < 0) {
                             break;
                         }

                         float confidence = detections[i * objectSize + 2];
                         auto label = detections[i * objectSize + 1] -1 ;                         
                         float xmin = detections[i * objectSize + 3] * width;
                         float ymin = detections[i * objectSize + 4] * height;
                         float xmax = detections[i * objectSize + 5] * width;
                         float ymax = detections[i * objectSize + 6] * height;

                         if (confidence > 0.5) {
                             std::ostringstream conf;
                             //std::cout<< detections[i * objectSize+0] << "  "<< detections[i * objectSize+1] <<std::endl;
                             conf << ":" << std::fixed << std::setprecision(3) << confidence;
                             cv::putText(frame,
                                         (!labels.empty() ? labels[label] : std::string("label #") + std::to_string(label)) + conf.str(),
                                         cv::Point2f(xmin, ymin - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1,
                                         cv::Scalar(0, 0, 255));
                             cv::rectangle(frame, cv::Point2f(xmin, ymin), cv::Point2f(xmax, ymax), cv::Scalar(0, 0, 255));
                         }
                     }

                 }
                 cv::imshow("Detection results", frame);
                 cv::waitKey(1);

            }



        /*
        if(Color_pic.empty())
        {
            //cv::imshow("1",Color_pic);
            //cv::waitKey(3);
            std::cout<<"pic is empty!"<<std::endl;

        }
        ros::spinOnce();
        loop_rate.sleep();*/
    }


    return 0;

}
