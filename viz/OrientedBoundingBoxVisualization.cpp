#include "OrientedBoundingBoxVisualization.hpp"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/ShapeDrawable>

#include <vizkit3d/Vizkit3DHelper.hpp>

using namespace vizkit3d;

struct OrientedBoundingBoxVisualization::Data {
    // Copy of the value given to updateDataIntern.
    // Making a copy is required because of how OSG works
    std::vector<base::samples::OrientedBoundingBox> data;
};


OrientedBoundingBoxVisualization::OrientedBoundingBoxVisualization()
    : p(new Data), color(1.0f, 1.0f, 1.0f, 1.0f) {
}

OrientedBoundingBoxVisualization::~OrientedBoundingBoxVisualization() {
    delete p;
}

void OrientedBoundingBoxVisualization::setColor(QColor q_color) {
    color = osg::Vec4(q_color.redF(), q_color.greenF(),
            q_color.blueF(), q_color.alphaF());
    setDirty();
    emit propertyChanged("Color");
}

QColor OrientedBoundingBoxVisualization::getColor() const {
    QColor q_color;
    q_color.setRgbF(color[0], color[1], color[2], color[3]);
    return q_color;
}

osg::ref_ptr<osg::Node> OrientedBoundingBoxVisualization::createMainNode() {
    osg::ref_ptr<osg::Group> boxes = new osg::Group();

    addBoxes(boxes);

    return boxes;
}

void OrientedBoundingBoxVisualization::updateMainNode(osg::Node* node) {
    node->asGroup()->removeChildren(0, node->asGroup()->getNumChildren());
    addBoxes(node->asGroup());
}

void OrientedBoundingBoxVisualization::updateDataIntern(std::vector<base::samples::OrientedBoundingBox> const& data) {
    p->data = data;
}

void  OrientedBoundingBoxVisualization::updateDataIntern(base::samples::OrientedBoundingBox const& data) {
    p->data.clear();
    p->data.push_back(data);
}

void OrientedBoundingBoxVisualization::addBoxes(osg::Group* group) {
    for (const auto & box : p->data) {
        osg::ref_ptr<osgviz::Object> wireframe = primitives.createWireframeBox(box.dimension.x(), box.dimension.y(), box.dimension.z(), color);
        wireframe->setPosition(box.position.x(), box.position.y(), box.position.z());
        wireframe->setOrientation(box.orientation.x(), box.orientation.y(), box.orientation.z(), box.orientation.w());
        group->addChild(wireframe);
    }
}
