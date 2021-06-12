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

    cv::Size s = this->frame.size();
    std::cout<<s<<std::endl;
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
    this->rgb_cam_calib = Task::readCameraInfo(calib_fname.string(),_rgb_camera_idx.value() );

    std::cout<<"EVENT CAMERA\nD:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"distortion model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Image Size ["<<this->event_cam_calib.height<<" x "<<this->event_cam_calib.width<<"]"<<std::endl;
    std::cout<<"K:\n"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"Kr:\n"<<this->event_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:\n"<<this->event_cam_calib.Rr<<std::endl;
    std::cout<<"Q:\n"<<this->event_cam_calib.Q<<std::endl;
    std::cout<<"Tij:\n"<<this->event_cam_calib.Tij<<std::endl;
 
    std::cout<<"RGB CAMERA\nD:"<<this->rgb_cam_calib.D<<std::endl;
    std::cout<<"distortion model:"<<this->rgb_cam_calib.distortion_model<<std::endl;
    std::cout<<"Image Size ["<<this->rgb_cam_calib.height<<" x "<<this->rgb_cam_calib.width<<"]"<<std::endl;
    std::cout<<"K:\n"<<this->rgb_cam_calib.K<<std::endl;
    std::cout<<"Kr:\n"<<this->rgb_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:\n"<<this->rgb_cam_calib.Rr<<std::endl;
    std::cout<<"Q:\n"<<this->rgb_cam_calib.Q<<std::endl;
    std::cout<<"Tij:\n"<<this->rgb_cam_calib.Tij<<std::endl;

    /** RGB camera is downscale to meet Event camera resolution **/
    //float scale_x =  this->rgb_cam_calib.width / this->event_cam_calib.width;
    //float scale_y =  this->rgb_cam_calib.height / this->event_cam_calib.height;
    //this->rgb_cam_calib.K.at<double>(0,0) /=  scale_x;
    //this->rgb_cam_calib.K.at<double>(0,2) /=  scale_x;
    //this->rgb_cam_calib.K.at<double>(1,1) /=  scale_y;
    //this->rgb_cam_calib.K.at<double>(1,2) /=  scale_y;
    //this->rgb_cam_calib.Kr.at<double>(0,0) /=  scale_x;
    //this->rgb_cam_calib.Kr.at<double>(0,2) /=  scale_x;
    //this->rgb_cam_calib.Kr.at<double>(1,1) /=  scale_y;
    //this->rgb_cam_calib.Kr.at<double>(1,2) /=  scale_y;

    //std::cout<<"RGB (RESCALED) CAMERA\nD:"<<this->rgb_cam_calib.D<<std::endl;
    //std::cout<<"distortion model:"<<this->rgb_cam_calib.distortion_model<<std::endl;
    //std::cout<<"Image Size ["<<this->rgb_cam_calib.height<<" x "<<this->event_cam_calib.width<<"]"<<std::endl;
    //std::cout<<"K:\n"<<this->rgb_cam_calib.K<<std::endl;
    //std::cout<<"Kr:\n"<<this->rgb_cam_calib.Kr<<std::endl;
    //std::cout<<"Rr:\n"<<this->rgb_cam_calib.Rr<<std::endl;
    //std::cout<<"Q:\n"<<this->rgb_cam_calib.Q<<std::endl;
    //std::cout<<"Tij:\n"<<this->rgb_cam_calib.Tij<<std::endl;

    /** Get the projection matrix **/
    cv::Mat R  = this->event_cam_calib.Tij(cv::Rect(0,0,3,3)).clone();
    std::cout<<"R matrix T[3:3]:\n"<< R <<std::endl;
    this->P = this->rgb_cam_calib.Kr * this->rgb_cam_calib.Rr * R * this->event_cam_calib.Rr.t() * this->event_cam_calib.Kr.inv();
    std::cout<<"Projection:\n"<< this->P <<std::endl;
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
        cv::Mat img = this->createFrame(this->frame, events.height, events.width, coord, p);

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

cv::Mat Viz::createFrame (cv::Mat &frame,  unsigned int &height, unsigned int &width,  std::vector<cv::Point2f> &coord, std::vector<uint8_t> &p)
{
    cv::Size s = frame.size();
    if (s.height == 0 || s.width == 0)
        return frame;

    /** Output image **/    
    cv::Mat out_img (cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
    std::cout<<"Out image "<<out_img.size()<<" TYPE: "<<type2str(out_img.type())<<std::endl;

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

    /** RGB image in event camera (backward warping ) **/
    cv::Mat map (height, width, CV_32FC2); //event frame size -> rgb camera size
    std::cout<<"Frame size: "<<frame.size()<<std::endl;
    for (int y=0; y<height; ++y)
    {
        for(int x=0; x<width; ++x)
        {
            cv::Point3d u_hom(x, y, 1.0);
            cv::Mat_<double> u_hat = this->P * cv::Mat(u_hom, false);
            //cv::Vec3b value = frame.at<cv::Vec3b>(floor(u_hom.y), floor(u_hom.x));
            //out_img.at<cv::Vec3b>(cv::Point(x, y)) = value;
            map.at<cv::Point2f>(y, x) = cv::Point2f(u_hat(0, 0), u_hat(0, 1));
        }
    }
    std::cout<<"Map size: "<<map.size()<<std::endl;
    cv::remap(frame, out_img, map, cv::Mat(), cv::INTER_LINEAR);
    std::cout<<"Out image "<<out_img.size()<<" TYPE: "<<type2str(out_img.type())<<std::endl;

    /** Draw the events **/
    auto it_x = coord.begin();
    auto it_p = p.begin();
    while(it_x != coord.end() && it_p != p.end())
    {
        cv::Point2i x_int = *it_x;
        if ((x_int.x > -1 && x_int.x < width) && (x_int.y > -1 && x_int.y < height))
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

void Viz::eventsToRGBCamera (const cv::Mat &P, const std::vector<cv::Point2f> &u, std::vector<cv::Point2f> &u_p)
{
    u_p.clear();

    for (auto it=u.begin(); it!=u.end(); ++it)
    {
        cv::Point3d u_hom(it->x, it->y, 1.0);
        //std::cout<<"u_hom: "<<u_hom<<std::endl;
        cv::Mat_<double> u_hat = P * cv::Mat(u_hom, false);
        //std::cout<<"u_hat: "<<u_hat<<std::endl;
        u_p.push_back(cv::Point2f(u_hat(0,0), u_hat(1,0)));
    }
}
