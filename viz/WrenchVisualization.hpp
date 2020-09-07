#ifndef WrenchVisualization_H
#define WrenchVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit3d/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/Wrench.hpp>

namespace vizkit3d
{
    class WrenchVisualization
        : public vizkit3d::Vizkit3DPlugin<base::Wrench>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        WrenchVisualization();
        ~WrenchVisualization();

    Q_INVOKABLE void updateData(base::Wrench const &sample)
    {vizkit3d::Vizkit3DPlugin<base::Wrench>::updateData(sample);}

    void setForce(const base::Vector3d& f) {
        state.force = f;
    }

    void setTorque(const base::Vector3d& t) {
        state.torque = t;
    }

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::Wrench const& plan);
  
    private:
        base::Wrench state;
    };
}
#endif
