#include "WrenchModel.hpp"

WrenchModel::WrenchModel(double resolution)
    : show_seperate_axes_force(false)
    , show_seperate_axes_torque(false)
    , resolution(resolution)
{
    primitivesfactory = osgviz::OsgViz::getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
    if (!primitivesfactory) {
        std::cerr << "Could not load PrimitivesFactory" << std::endl;
        return;
    }
    buildGeometry();
}

osg::ref_ptr<osg::PositionAttitudeTransform> WrenchModel::createCircularArrow(const osg::Vec4f& color) {
    // osg::ref_ptr<CircularArrow> arr = new CircularArrow(0.2, 0.02, 12, 32, 1.5*M_PI);
    // arr->setColor(color);
    // arr->rebuildGeometry();

    osg::ref_ptr<osg::PositionAttitudeTransform> arr = primitivesfactory->createCircularArrow(0.1, 0.02, 12, 32, 1.5*M_PI, 2.0, color);

    return arr;
}


void WrenchModel::buildGeometry() {

    

    // force
    force_node = primitivesfactory->createArrow(osg::Vec4f(1, 0, 0, 1.0), true);
    addChild(force_node, true); // child 0

    // torque
    torque_group = new osg::Group;    
    
    torque_group->addChild(primitivesfactory->createArrow(osg::Vec4f(0, 1, 0, 1.0), true));
    torque_group->addChild(createCircularArrow(osg::Vec4f(0, 1, 0, 1.0)));
    addChild(torque_group, true); // child 1

    // seperated along axes
    force_axes_group = new osg::Switch;
    torque_axes_group = new osg::Switch;

    for (unsigned int i=0; i<3; ++i) {
        //add force axis
        force_axes_group->addChild(primitivesfactory->createArrow(osg::Vec4f(1, 0, 0, 1.0), true), true);

        //add torque axis
        osg::ref_ptr<osg::Group> torque_axis_group = new osg::Group;
        torque_axis_group->addChild(primitivesfactory->createArrow(osg::Vec4f(0, 1, 0, 1.0), true));
        torque_axis_group->addChild(createCircularArrow(osg::Vec4f(0, 1, 0, 1.0)));

        torque_axes_group->addChild(torque_axis_group, true);
    }
    
    addChild(force_axes_group, false); // child 2
    addChild(torque_axes_group, false); // child 3
}

void WrenchModel::update(const base::Wrench& wrench) {
    osg::Matrixd R, S;
    osg::Quat Q;
        
    if (show_seperate_axes_force) {
        // for all axes
        for (unsigned int i=0; i<3; ++i) {
            if (wrench.force[i] == 0) {
                // if component is zero, disable
                force_axes_group->setValue(i, false);
            } 
            else {
                //update transform
                osg::PositionAttitudeTransform* force_transform = dynamic_cast<osg::PositionAttitudeTransform*>(force_axes_group->getChild(i));
                osg::Vec3d force(0,0,0);
                force[i] = wrench.force[i];
                Q.makeRotate(osg::Vec3d(0,0,1), force);
                force_transform->setScale(osg::Vec3f(resolution, resolution, resolution*force.length()));
                force_transform->setAttitude(Q);
                force_axes_group->setChildValue(force_transform, true);
            }
        }
    } 
    else {
        if (wrench.force.isZero()) {
            // if force is zero vector, disable
            setChildValue(force_node, false);
        } 
        else {
            // update transform
            osg::Vec3d force(wrench.force.x(), wrench.force.y(), wrench.force.z());
            Q.makeRotate(osg::Vec3d(0,0,1), force);
            //force_node->setScale(osg::Vec3f(resolution, resolution, resolution*force.length()));

            osgviz::ArrowNode* f_node = dynamic_cast<osgviz::ArrowNode*>(force_node.get());
            f_node->setLength(resolution*force.length());
            force_node->setAttitude(Q);

            setChildValue(force_node, true);
        }
    }

    if (show_seperate_axes_torque) {
        for (unsigned int i=0; i<3; ++i) {
            if (wrench.torque[i] == 0) {
                torque_axes_group->setValue(i, false);
            } 
            else {
                osg::Group* group = dynamic_cast<osg::Group*>(torque_axes_group->getChild(i));
                osg::PositionAttitudeTransform* torque_vec_transform = dynamic_cast<osg::PositionAttitudeTransform*>(group->getChild(0));
                osg::Vec3d torque(0,0,0);
                torque[i] = wrench.torque[i];
                Q.makeRotate(osg::Vec3d(0,0,1), torque);
                torque_vec_transform->setScale(osg::Vec3f(resolution, resolution, resolution * torque.length()));
                torque_vec_transform->setAttitude(Q);

                osg::PositionAttitudeTransform* torque_transform = dynamic_cast<osg::PositionAttitudeTransform*>(group->getChild(1));
                torque_transform->setAttitude(Q);
                torque_transform->setPosition(torque * 0.4 * resolution);
                //torque_transform->setScale(osg::Vec3f(resolution,resolution,resolution));
                torque_axes_group->setValue(i, true);
            }
        }
    } 
    else {
        if (wrench.torque.isZero()) {
            // if torque is zero vector, disable
            setChildValue(torque_group, false);
        }
        else {
            // update transform
            osgviz::ArrowNode* torque_vec_transform = dynamic_cast<osgviz::ArrowNode*>(torque_group->getChild(0));
            osg::Vec3d torque(wrench.torque.x(), wrench.torque.y(), wrench.torque.z());
            Q.makeRotate(osg::Vec3d(0,0,1), torque);
            //torque_vec_transform->setScale(osg::Vec3d(resolution, resolution, resolution*torque.length()));
            torque_vec_transform->setLength(resolution*torque.length());
            torque_vec_transform->setAttitude(Q);

            osg::PositionAttitudeTransform* torque_transform = dynamic_cast<osg::PositionAttitudeTransform*>(torque_group->getChild(1));
            torque_transform->setAttitude(Q);
            torque_transform->setPosition(torque * 0.4 * resolution);
            //torque_transform->setScale(osg::Vec3f(resolution,resolution,resolution));
            setChildValue(torque_group, true);
        }        
    }
}