/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "PathDrawer.hpp"
#include <base/Logging.hpp>

using namespace mars_addons;

PathDrawer::PathDrawer(std::string const& name)
    : PathDrawerBase(name)
{
}

PathDrawer::PathDrawer(std::string const& name, RTT::ExecutionEngine* engine)
    : PathDrawerBase(name, engine)
{
}

PathDrawer::~PathDrawer()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See PathDrawer.hpp for more detailed
// documentation about them.
bool PathDrawer::configureHook()
{
    if (! mars::Plugin::configureHook())
        return false;
    
    // init the line handle
    lines = 0;
    updated_lines = 0;
    point = 0;

    return true;
}

bool PathDrawer::startHook()
{
    if (! mars::Plugin::startHook())
        return false;
    if(control->graphics){
        control->graphics->addGraphicsUpdateInterface(this);
    }
    return true;
}

void PathDrawer::updateHook()
{
    mars::Plugin::updateHook();

    generate_3d_path();
}

void PathDrawer::errorHook()
{
    mars::Plugin::errorHook();
}


void PathDrawer::stopHook()
{
    mars::Plugin::stopHook();
}


void PathDrawer::cleanupHook()
{
    mars::Plugin::cleanupHook();
}


mars::interfaces::sReal PathDrawer::getHeightFromScene(mars::interfaces::sReal x, mars::interfaces::sReal y)
{
    mars::interfaces::PhysicsInterface* physics = control->sim->getPhysics();
    mars::interfaces::sReal z = 10.0;
    const mars::utils::Vector ray_origin(x, y, z);
    const mars::utils::Vector ray_vector(0.0, 0.0, -20);
    mars::interfaces::sReal value = z - physics->getVectorCollision(ray_origin, ray_vector);

    return value;
}


void PathDrawer::postGraphicsUpdate(void )
{
    osg_lines::LinesFactory lF;
    if(lines != updated_lines){
        // clear old path
        control->graphics->removeOSGNode(lines->getOSGNode());
        delete lines;
        lines = updated_lines;

        // draw it
        control->graphics->addOSGNode(lines->getOSGNode());
    }

    // get the current waypoint and draw it
    base::Waypoint waypoint;
    if(_current_waypoint.read(waypoint) == RTT::NewData){
        // clear old waypoint
        if(point == 0){
            point = lF.createLines();
            control->graphics->addOSGNode(point->getOSGNode());
        }else{
            // clear all points
            point->setData(std::list<osg_lines::Vector>());
        }
        point->appendData(osg_lines::Vector(waypoint.position[0], waypoint.position[1], waypoint.position[2]));
        point->appendData(osg_lines::Vector(waypoint.position[0], waypoint.position[1], waypoint.position[2]+0.5));
        point->setColor(osg_lines::Color(1.0, 0.0, 0.0, 1.0));
        point->setLineWidth(16);
    }

    // send the 3d trajectory when triggered
    _trajectories_3d.write(trajectories_3d);

}

void PathDrawer::generate_3d_path()
{
   osg_lines::LinesFactory lF;
    // read the current trajectory
    std::vector<base::Trajectory> trajectories_2d;
    if(_trajectories_2d.read(trajectories_2d) == RTT::NewData){
        // clear old path
        if(updated_lines != lines)
        {
            delete updated_lines;
        }
        updated_lines = lF.createLines();

        // get each xy coordinate
        std::vector<base::Trajectory>::iterator it;
        for(it = trajectories_2d.begin(); it != trajectories_2d.end(); ++it){

            // get the dimension
            int dim = it->spline.getDimension();
            if(dim < 2 || dim > 3){
                LOG_WARN_S << "2d or 3d (where z-component will be neglected) trajectory needed";
                continue;
            }

            // get the coordinates and generate 3d coordinates
            std::vector<double> path = it->spline.getCoordinates();
            base::Vector3d v;
            std::vector<base::geometry::SplineBase::CoordinateType> coord_types;
            std::vector<base::Vector3d> waypoints;
            std::vector<double>::iterator val;
            for(val = path.begin(); val != path.end(); val += dim){
                v[0] = *val;
                v[1] = *(val+1);
                v[2] = getHeightFromScene(v[0], v[1]) + _distance_to_ground.get();
                //printf("adding point %g / %g\n", v[0], v[1]);
                updated_lines->appendData(osg_lines::Vector(v[0], v[1], v[2]));

                // write the z-coridnate in the trajectory
                coord_types.push_back(base::geometry::SplineBase::ORDINARY_POINT);
                waypoints.push_back(v);
            }

            // use the 3d points to generate the 3d trajectory
            std::vector<double> parameters;
            base::Trajectory new_trajectory;
            try {
                new_trajectory.spline.interpolate(waypoints, parameters, coord_types);
            } catch (std::runtime_error& e) {
                LOG_ERROR_S << "Spline exception: " << e.what();
            }
            new_trajectory.speed = it->speed;
            trajectories_3d.clear();
            trajectories_3d.push_back(new_trajectory);
        }
        updated_lines->setColor(osg_lines::Color(0.0, 1.0, 0.0, 1.0));
        updated_lines->setLineWidth(4);
    }


    // send the 3d trajectory when triggered
    _trajectories_3d.write(trajectories_3d);
}