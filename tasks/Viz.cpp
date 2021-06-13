/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */


#include <boost/filesystem.hpp>
#include <frame_helper/FrameHelper.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include "Viz.hpp"

using namespace dsec2pocolog;
namespace fs = boost::filesystem;

std::string type2str(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
};

Viz::Viz(std::string const& name)
    : VizBase(name)
{
    /** Set the img member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

}

Viz::~Viz()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Viz.hpp for more detailed
// documentation about them.

bool Viz::configureHook()
{
    if (! VizBase::configureHook())
        return false;

    this->calib_file = _calib_file.value();

    /* read the calibration files **/
    fs::path calib_fname = fs::path(this->calib_file);
    this->event_cam_calib = Task::readCameraInfo(calib_fname.string(),_event_camera_idx.value());
    std::cout<<"EVENT CAMERA\nD:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"distortion model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Image Size ["<<this->event_cam_calib.height<<" x "<<this->event_cam_calib.width<<"]"<<std::endl;
    std::cout<<"K:\n"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"Kr:\n"<<this->event_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:\n"<<this->event_cam_calib.Rr<<std::endl;
    std::cout<<"Q:\n"<<this->event_cam_calib.Q<<std::endl;
    std::cout<<"Tij:\n"<<this->event_cam_calib.Tij<<std::endl;
 
    return true;
}

bool Viz::startHook()
{
    if (! VizBase::startHook())
        return false;
    return true;
}

void Viz::updateHook()
{
    VizBase::updateHook();

    /** Read the events **/
    ::base::samples::EventArray events;
    if (_events.read(events, false) == RTT::NewData)
    {

        std::cout<<"events size: "<<events.events.size()<<std::endl;

        std::vector<cv::Point2f> coord, coord_rect;
        std::vector<uint8_t> p;
        for (auto it = events.events.begin(); it != events.events.end(); ++it)
        {
            coord.push_back(cv::Point2f(it->x, it->y));
            p.push_back(it->polarity);
        }

        /* Undistort events pixel coordinates**/
        cv::undistortPoints(coord, coord_rect, this->event_cam_calib.K, this->event_cam_calib.D,
                            this->event_cam_calib.Rr, this->event_cam_calib.Kr);

        std::cout<<"coord: " << coord[0] <<" coord_rect: "<< coord_rect[0] <<std::endl;

        /** Create the Frame **/
        cv::Mat img = this->createFrame(this->frame, coord, p);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *img_msg_ptr = this->img_msg.write_access();
        img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(img, *img_msg_ptr);

        /** Write the frame in the output port **/
        img_msg_ptr->time = events.time;
        img_msg_ptr->received_time = events.time;
        this->img_msg.reset(img_msg_ptr);
        _img.write(this->img_msg);
    }

    /** Read the image **/
    if (_frame.read(this->frame_ptr, false) == RTT::NewData)
    {
        this->frame = frame_helper::FrameHelper::convertToCvMat(*frame_ptr);
    }

}

void Viz::errorHook()
{
    VizBase::errorHook();
}

void Viz::stopHook()
{
    VizBase::stopHook();
}

void Viz::cleanupHook()
{
    VizBase::cleanupHook();
}

cv::Mat Viz::createFrame (cv::Mat &frame, std::vector<cv::Point2f> &coord, std::vector<uint8_t> &p)
{
    cv::Size s = frame.size();
    if (s.height == 0 || s.width == 0)
        return frame;

    /** Output image **/    
    cv::Mat out_img; frame.copyTo(out_img);

    /** Event colors **/
    cv::Vec3b color_positive, color_negative;
    if (_color_encoding.value() == dsec2pocolog::BLUE_RED)
    {
        color_positive = cv::Vec3b(255.0, 0.0, 0.0); //BGR
        color_negative = cv::Vec3b(0.0, 0.0, 255.0);
    }
    else if (_color_encoding.value() == dsec2pocolog::GREEN_RED)
    {
        color_positive = cv::Vec3b(0.0, 255.0, 0.0);
        color_negative = cv::Vec3b(0.0, 0.0, 255.0);

    }
    else if (_color_encoding.value() == dsec2pocolog::BLUE_BLACK)
    {
        color_positive = cv::Vec3b(255.0, 0.0, 0.0);
        color_negative = cv::Vec3b(0.0, 0.0, 0.0);
    }

    /** Draw the events **/
    auto it_x = coord.begin();
    auto it_p = p.begin();
    while(it_x != coord.end() && it_p != p.end())
    {
        cv::Point2i x_int = *it_x;
        if ((x_int.x > -1 && x_int.x < s.width) && (x_int.y > -1 && x_int.y < s.height))
        {
            if ((*it_p))
            {
                out_img.at<cv::Vec3b>(x_int) = color_positive;
            }
            else
            {
                out_img.at<cv::Vec3b>(x_int) = color_negative;
            }
        }

        ++it_x;
        ++it_p;
    }

    return out_img;
}

