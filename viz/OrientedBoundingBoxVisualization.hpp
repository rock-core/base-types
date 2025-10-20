#pragma once

#include <vector>
#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/samples/OrientedBoundingBox.hpp>
#include <osgViz/modules/viz/Primitives/PrimitivesFactory.h>

namespace osg {
class Group;
}

namespace vizkit3d {


class OrientedBoundingBoxVisualization
    : public vizkit3d::Vizkit3DPlugin< std::vector<base::samples::OrientedBoundingBox> >
    , public vizkit3d::VizPluginAddType<base::samples::OrientedBoundingBox>
    , boost::noncopyable
{
Q_OBJECT
Q_PROPERTY(QColor Color READ getColor WRITE setColor)

 public:
    OrientedBoundingBoxVisualization();
    ~OrientedBoundingBoxVisualization();

    /**
     * Thread-safe call of 
     * 'updateDataIntern ( std::vector<base::samples::OrientedBoundingBox> const& data )'.
     */
    Q_INVOKABLE void updateData(std::vector<base::samples::OrientedBoundingBox> const &sample) {
        vizkit3d::Vizkit3DPlugin< std::vector<base::samples::OrientedBoundingBox> >::updateData(sample);
    }

    /**
     * Thread-safe call of 'updateDataIntern ( const base::samples::OrientedBoundingBox& data )'.
     */
    Q_INVOKABLE void updateData(base::samples::OrientedBoundingBox const &sample) {
        vizkit3d::Vizkit3DPlugin< std::vector<base::samples::OrientedBoundingBox> >::updateData(sample);
    }

 public slots:
    /**
     * Sets the color of all boxes.
     */
    void setColor(QColor q_color);

    /**
     * Returns the current color of the boxes.
     */
    QColor getColor() const;

 protected:
    /**
     * OSG tree: Group <- Transformation <- Geode <- Sphere 
     *                                            <- Triangle
     */
    osg::ref_ptr<osg::Node> createMainNode();

    /**
     * Clears the group and redraws all boxes.
     */
    void updateMainNode(osg::Node* node);

    /**
     * Replaces the current list of boxes with the passed one.
     */
    void updateDataIntern(std::vector<base::samples::OrientedBoundingBox> const& data);

    /**
     * Clears the current list of boxes and adds the new waypoint.
     */
    void updateDataIntern(base::samples::OrientedBoundingBox const& data);

 private:
    osg::ref_ptr<osg::Group> group;
    struct Data;
    Data* p;
    osg::Vec4 color;
    osgviz::PrimitivesFactory primitives;
    /**
     * Inserts all boxes into the tree using the tree structure shown 
     * in \a createMainNode() and the currently set color.
     */
    void addBoxes(osg::Group* group);
};
}  // namespace vizkit3d
