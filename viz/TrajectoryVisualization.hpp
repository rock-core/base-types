#ifndef TRAJECTORYVISUALISATION_H
#define TRAJECTORYVISUALISATION_H
#include <Eigen/Geometry>
#include <osg/Geometry>
#include <deque>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <base/geometry/Spline.hpp>
#include <base/Trajectory.hpp>

namespace vizkit3d 
{

class TrajectoryVisualization: public Vizkit3DPlugin<base::Vector3d>
                , public VizPluginAddType<base::geometry::Spline3>
                , public VizPluginAddType<std::vector<base::Trajectory> >
{
    Q_OBJECT
    //unsigned int is not supported by the property browser so far
    Q_PROPERTY(int MaxPoints READ getMaxNumberOfPoints WRITE setMaxNumberOfPoints)
    Q_PROPERTY(double LineWidth READ getLineWidth WRITE setLineWidth)
    Q_PROPERTY(QColor Color READ getColor WRITE setColor)
    Q_PROPERTY(QColor BackwardColor READ getBackwardColor WRITE setBackwardColor)

    public:
        TrajectoryVisualization();
        ~TrajectoryVisualization();
        void setColor(const base::Vector3d& color); 
        Q_INVOKABLE void clear();

        Q_INVOKABLE void updateTr(const std::vector<base::Trajectory>& data)
        {
            Vizkit3DPlugin<base::Vector3d>::updateData(data);
        }

        Q_INVOKABLE void updateData(const base::geometry::Spline<3>& data)
        { Vizkit3DPlugin<base::Vector3d>::updateData(data); }
        Q_INVOKABLE void updateSpline(const base::geometry::Spline<3>& data)
        { updateData(data); }
        Q_INVOKABLE void updateData(const base::Vector3d& data)
        { Vizkit3DPlugin<base::Vector3d>::updateData(data); }
        Q_INVOKABLE void updateTrajectory(const base::Vector3d& data)
        { updateData(data); }

    public slots:
        int getMaxNumberOfPoints(){return max_number_of_points;};
        void setMaxNumberOfPoints(int points){max_number_of_points = points; emit propertyChanged("MaxPoints");};
        double getLineWidth();
        void setLineWidth(double line_width);
        void setColor(QColor color);
        QColor getColor() const;
        void setBackwardColor(QColor color);
        QColor getBackwardColor() const;

    protected:
        void addSpline(const base::geometry::Spline3& data, const osg::Vec4& color);
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode( osg::Node* node );
        virtual void updateDataIntern( const  base::Vector3d& data );
        virtual void updateDataIntern(const base::geometry::Spline3& data);
        virtual void updateDataIntern(const std::vector<base::Trajectory>& data);

        private:
        bool doClear;
            size_t max_number_of_points;
        double line_width;

        osg::Vec4 color;
        osg::Vec4 backwardColor;
        
        struct Point
        {
            osg::Vec3 point;
            osg::Vec4 color;
        };
        
        std::deque<Point> points;
        osg::ref_ptr<osg::Vec4Array> colorArray; 
        osg::ref_ptr<osg::Vec3Array> pointsOSG;
        osg::ref_ptr<osg::DrawArrays> drawArrays;
        osg::ref_ptr<osg::Geometry> geom;
        osg::ref_ptr<osg::Geode> geode;
};

}
#endif // TRAJECTORYVISUALISATION_H
