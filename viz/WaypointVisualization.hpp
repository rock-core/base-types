#ifndef WAYPOINTVISUALIZATION_H
#define WAYPOINTVISUALIZATION_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/Waypoint.hpp>

namespace osg {
    class Group;
}

namespace vizkit3d
{
    class WaypointVisualization
        : public vizkit3d::Vizkit3DPlugin< std::vector<base::Waypoint> >
        , public vizkit3d::VizPluginAddType<base::Waypoint>
        , boost::noncopyable
    {
    Q_OBJECT
    Q_PROPERTY(QColor Color READ getColor WRITE setColor)
    
    public:
        WaypointVisualization();
        ~WaypointVisualization();

        /**
         * Thread-safe call of 
         * 'updateDataIntern ( std::vector<base::Waypoint> const& data )'.
         */
        Q_INVOKABLE void updateData(std::vector<base::Waypoint> const &sample) {
            vizkit3d::Vizkit3DPlugin< std::vector<base::Waypoint> >::updateData(sample);
        }
        
        /**
         * Thread-safe call of 'updateDataIntern ( const base::Waypoint& data )'.
         */
        Q_INVOKABLE void updateData(base::Waypoint const &sample) {
            vizkit3d::Vizkit3DPlugin< std::vector<base::Waypoint> >::updateData(sample);
        }
        
    public slots:
        /**
         * Sets the color of all waypoints.
         */
        void setColor(QColor q_color);
        
        /**
         * Returns the current color of the waypoints.
         */
        QColor getColor() const;
        
    protected:
        /**
         * OSG tree: Group <- Transformation <- Geode <- Sphere 
         *                                            <- Triangle
         */
        osg::ref_ptr<osg::Node> createMainNode();
        
        /**
         * Clears the group and redraws all waypoints.
         */
        void updateMainNode(osg::Node* node);
        
        /**
         * Replaces the current list of waypoints with the passed one.
         */
        void updateDataIntern(std::vector<base::Waypoint> const& data);
        
        /**
         * Clears the current list of waypoints and adds the new waypoint.
         */
        void updateDataIntern ( base::Waypoint const& data );
        
    private:
        osg::ref_ptr<osg::Group> group;
        struct Data;
        Data* p;
        osg::Vec4 color;
        
        /**
         * Inserts all waypoints into the tree using the tree structure shown 
         * in \a createMainNode() and the currently set color.
         */
        void addWaypoints(osg::Group* group);
    };
}
#endif
