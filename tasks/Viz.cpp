/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Viz.hpp"

using namespace dsec2pocolog;

Viz::Viz(std::string const& name)
    : VizBase(name)
{
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

    /* Undistort events pixel coordinates**/
    //std::vector<cv::Point2f> coord, coord_rect;
    //coord.push_back(cv::Point2f(this->events.x[i], this->events.y[i]));
    //cv::undistortPoints(coord, coord_rect, this->event_cam_calib.K, this->event_cam_calib.D,
    //                    this->event_cam_calib.Rect, this->event_cam_calib.K_);

    //std::cout<<"coord: " << coord <<"rect coord: "<<coord_rect<<std::endl;

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
