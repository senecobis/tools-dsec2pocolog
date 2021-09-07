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

cv::Mat drawValuesPoints(const std::vector<cv::Point2f> &points, const std::vector<int8_t> &values, const int height, const int width, const std::string &method, const float s)
{
    /** Asertion only in debug mode **/
    assert(height > 0);
    assert(width > 0);
    assert(values.size() == points.size());
    assert((method.compare("nn") == 0) || (method.compare("bilinear") == 0));

    /** Mat image **/
    cv::Mat img = cv::Mat(height, width, CV_64FC1, cv::Scalar(0));
    cv::Size size = img.size();

    auto clip = [](const int n, const int lower, const int upper)
    {
        return std::max(lower, std::min(n, upper));
    };

    auto it_x = points.begin();
    auto it_p = values.begin();
    while(it_x != points.end() && it_p != values.end())
    {
        if (method.compare("nn") == 0)
        {
            cv::Point2i x_int = *it_x;
            //std::cout<<*it_x<<std::endl;
            x_int.x = clip(x_int.x, 0, size.width - 1);
            x_int.y = clip(x_int.y, 0, size.height - 1);

            img.at<double>(x_int) += (*it_p);

        }
        else if (method.compare("bilinear") == 0)
        {
            int x0 = floor(it_x->x);
            int y0 = floor(it_x->y);
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            //std::cout<<"x0: "<< x0<<" y0: "<<y0<<" x1: "<< x1<<" y1: "<<y1 <<" pol: "<<std::to_string(*it_p)<<std::endl;
            /** compute the voting weights. Note: assign weight 0 if the point is out of the image **/
            double wa, wb, wc, wd;
            wa = ((x0 < size.width) && (y0 < size.height) && (x0 >= 0) && (y0 >= 0))? (x1 - it_x->x) * (y1 - it_x->y) : 0.0;
            wb = ((x0 < size.width) && (y1 < size.height) && (x0 >= 0) && (y1 >= 0))? (x1 - it_x->x) * (it_x->y - y0) : 0.0;
            wc = ((x1 < size.width) && (y0 < size.height) && (x1 >= 0) && (y0 >= 0))? (it_x->x - x0) * (y1 - it_x->y) : 0.0;
            wd = ((x1 < size.width) && (y1 < size.height) && (x1 >= 0) && (y1 >= 0))? (it_x->x - x0) * (it_x->y - y0) : 0.0;

            x0 = clip(x0, 0, size.width - 1);
            x1 = clip(x1, 0, size.width - 1);
            y0 = clip(y0, 0, size.height - 1);
            y1 = clip(y1, 0, size.height - 1);
            //std::cout<<"wa: "<<wa<<" wb: "<<wb<<" wc: "<<wc<<" wd: "<<wd<<std::endl;

            img.at<double>(y0, x0) += wa * (*it_p);
            img.at<double>(y1, x0) += wb * (*it_p);
            img.at<double>(y0, x1) += wc * (*it_p);
            img.at<double>(y1, x1) += wd * (*it_p);
        }
        ++it_x;
        ++it_p;
    }

    if(s > 0)
        cv::GaussianBlur(img, img, cv::Size(3, 3), s, s);

    return img;
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
    ::base::samples::EventArray events_sample;
    if (_events.read(events_sample, false) == RTT::NewData)
    {
        /** Insert the events in the buffer **/
        this->events.insert(this->events.end(), events_sample.events.begin(), events_sample.events.end());
        //std::cout<<"[UPDATE_HOOK] events buffer size: "<<this->events.size()<<std::endl;
    }

    /** Read the image **/
    if (_frame.read(this->frame_ptr, false) == RTT::NewData)
    {
        this->frame = frame_helper::FrameHelper::convertToCvMat(*frame_ptr);

        /** Get the array of coordinates and polarities (-1, 1) **/
        std::vector<cv::Point2f> coord;
        std::vector<int8_t> p;
        for (auto it = this->events.begin(); it != this->events.end(); ++it)
        {
            coord.push_back(cv::Point2f(it->x, it->y));
            p.push_back((2 * (uint8_t)it->polarity) -1);
        }

        /** Undistort events pixel coordinates**/
        std::vector<cv::Point2f> coord_rect;
        cv::undistortPoints(coord, coord_rect, this->event_cam_calib.K, this->event_cam_calib.D,
                            this->event_cam_calib.Rr, this->event_cam_calib.Kr);

        //std::cout<<"[UPDATE_HOOK] coord: " << coord[0] <<" coord_rect: "<< coord_rect[0] <<std::endl;

        /** Draw the event frame **/
        std::cout<<"[UPDATE_HOOK] create image with " <<this->events.size()<<" events"<<std::endl;
        cv::Mat event_frame = drawValuesPoints(coord_rect, p, this->frame.rows, this->frame.cols, "bilinear", 0.0);

        /** Create the Frame **/
        cv::Mat img = this->createFrame(this->frame, event_frame);
        std::cout<<"image "<<img.size()<<" TYPE: "<<type2str(img.type())<<std::endl;

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *img_msg_ptr = this->img_msg.write_access();
        img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(img, *img_msg_ptr);

        /** Write the frame in the output port **/
        img_msg_ptr->time = frame_ptr->time;
        img_msg_ptr->received_time = frame_ptr->time;
        this->img_msg.reset(img_msg_ptr);
        _img.write(this->img_msg);

        /** Clean the event buffer **/
        this->events.clear();
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


cv::Mat Viz::createFrame (cv::Mat &img_frame, cv::Mat &event_frame)
{
    assert(img_frame.size() == event_frame.size());
    std::cout<<"img_frame "<<img_frame.size()<<" TYPE: "<<type2str(img_frame.type())<<std::endl;
    std::cout<<"event_frame "<<event_frame.size()<<" TYPE: "<<type2str(event_frame.type())<<std::endl;

    cv::Size s = img_frame.size();
    if (s.height == 0 || s.width == 0)
        return img_frame;

    /** Output image **/    
    cv::Mat out_img; img_frame.copyTo(out_img);
    std::cout<<"out_img "<<out_img.size()<<" TYPE: "<<type2str(out_img.type())<<std::endl;

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

    /** Normalize the Event frame between 0-1 **/
    double min, max; cv::minMaxLoc(event_frame, &min, &max);
    std::cout<<"MIN EVENT: "<<min<<" MAX EVENT: "<<max<<std::endl;

    /** Draw the events **/
    for (size_t x=0; x<out_img.cols; ++x)
    {
        for(size_t y=0; y<out_img.rows; ++y)
        {
            double p = event_frame.at<double>(y, x);
            //double p_norm = (p - min)/(max-min);
            //std::cout<<"event integration ["<<y<<","<<x<< "]: "<<p<<" norm: "<<p_norm<<std::endl;
            if (p>1.0)
            {
                out_img.at<cv::Vec3b>(y, x) = color_positive;
            }
            else if (p<-1.0)
            {
                out_img.at<cv::Vec3b>(y, x) = color_negative;
            }
        }
    }

    return out_img;
}



