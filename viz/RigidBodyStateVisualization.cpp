#include "RigidBodyStateVisualization.hpp"
#include "Uncertainty.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Material>

using namespace osg;

namespace vizkit3d 
{

RigidBodyStateVisualization::RigidBodyStateVisualization(QObject* parent)
    : Vizkit3DPlugin<base::samples::RigidBodyState>(parent)
    , covariance(false)
    , covariance_with_samples(false)
    , color(1, 1, 1)
    , total_size(1)
    , main_size(0.1)
    , body_type(BODY_NONE)
    , forcePositionDisplay(false)
    , forceOrientationDisplay(false)
{
}

RigidBodyStateVisualization::~RigidBodyStateVisualization()
{
}

void RigidBodyStateVisualization::setColor(const Vec4d& color, Geode* geode)
{
    Material *material = new Material();
    material->setDiffuse(Material::FRONT,  Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(Material::FRONT, Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(Material::FRONT,  Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(Material::FRONT, color);
    material->setShininess(Material::FRONT, 10.0);

    geode->getOrCreateStateSet()->setAttribute(material);    
}

bool RigidBodyStateVisualization::isPositionDisplayForced() const
{ return forcePositionDisplay; }
void RigidBodyStateVisualization::setPositionDisplayForceFlag(bool flag)
{ forcePositionDisplay = flag; }
bool RigidBodyStateVisualization::isOrientationDisplayForced() const
{ return forceOrientationDisplay; }
void RigidBodyStateVisualization::setOrientationDisplayForceFlag(bool flag)
{ forceOrientationDisplay = flag; }

ref_ptr<osg::Group> RigidBodyStateVisualization::createSimpleSphere(double size)
{   
    ref_ptr<Group> group = new Group();
    
    ref_ptr<Geode> geode = new Geode();
    ref_ptr<Sphere> sp = new Sphere(Vec3f(0,0,0), main_size * size);
    ref_ptr<ShapeDrawable> spd = new ShapeDrawable(sp);
    spd->setColor(Vec4f(color.x(), color.y(), color.z(), 1.0));
    geode->addDrawable(spd);
    group->addChild(geode);
    
    return group;
}
  
ref_ptr<Group> RigidBodyStateVisualization::createSimpleBody(double size)
{   
    ref_ptr<Group> group = new Group();
    
    ref_ptr<Geode> geode = new Geode();
    ref_ptr<Sphere> sp = new Sphere(Vec3f(0,0,0), main_size * size);
    ref_ptr<ShapeDrawable> spd = new ShapeDrawable(sp);
    spd->setColor(Vec4f(color.x(), color.y(), color.z(), 1.0));
    geode->addDrawable(spd);
    group->addChild(geode);
    
    //up
    ref_ptr<Geode> c1g = new Geode();
    ref_ptr<Cylinder> c1 = new Cylinder(Vec3f(0, 0, size / 2), size / 40, size);
    ref_ptr<ShapeDrawable> c1d = new ShapeDrawable(c1);
    c1g->addDrawable(c1d);
    setColor(Vec4f(0, 0, 1.0, 1.0), c1g);
    group->addChild(c1g);
    
    //north direction
    ref_ptr<Geode> c2g = new Geode();
    ref_ptr<Cylinder> c2 = new Cylinder(Vec3f(0, size / 2, 0), size / 40, size);
    c2->setRotation(Quat(M_PI/2.0, Vec3d(1,0,0)));
    ref_ptr<ShapeDrawable> c2d = new ShapeDrawable(c2);
    c2g->addDrawable(c2d);
    setColor(Vec4f(0.0, 1.0, 0, 1.0), c2g);
    group->addChild(c2g);

    //east
    ref_ptr<Geode> c3g = new Geode();
    ref_ptr<Cylinder> c3 = new Cylinder(Vec3f(size / 2, 0, 0), size / 40, size);
    c3->setRotation(Quat(M_PI/2.0, Vec3d(0,1,0)));
    ref_ptr<ShapeDrawable> c3d = new ShapeDrawable(c3);
    c3g->addDrawable(c3d);
    setColor(Vec4f(1.0, 0.0, 0, 1.0), c3g);
    group->addChild(c3g);

    return group;
}

double RigidBodyStateVisualization::getMainSphereSize() const
{
    return main_size;
}

void RigidBodyStateVisualization::setMainSphereSize(double size)
{
    main_size = size;
    // This triggers an update of the model if we don't have a custom model
    setSize(total_size);
}

void RigidBodyStateVisualization::setSize(double size)
{
    total_size = size;
    if (body_type == BODY_SIMPLE)
        resetModel(size);
    else if (body_type == BODY_SPHERE)
        resetModelSphere(size);
}

double RigidBodyStateVisualization::getSize() const
{
    return total_size;
}

void RigidBodyStateVisualization::resetModel(double size)
{
    body_type  = BODY_SIMPLE;
    body_model = createSimpleBody(size);
    setDirty();
}

void RigidBodyStateVisualization::resetModelSphere(double size)
{
    body_type  = BODY_SPHERE;
    body_model = createSimpleSphere(size);
    setDirty();
}

QString RigidBodyStateVisualization::getModelPath() const
{
    if (body_type == BODY_SPHERE)
        return "sphere";
    else if (body_type == BODY_SIMPLE)
        return "simple";
    else
        return model_path;
}

void RigidBodyStateVisualization::loadModel(QString const& path)
{
    return loadModel(path.toStdString());
}

void RigidBodyStateVisualization::loadModel(std::string const& path)
{
    if (path == "sphere")
    {
        resetModelSphere(total_size);
        return;
    }
    else if (path == "simple")
    {
        resetModel(total_size);
        return;
    }

    ref_ptr<Node> model = osgDB::readNodeFile(path);
    body_type  = BODY_CUSTOM_MODEL;
    body_model = model;
    model_path = QString::fromStdString(path);
    //set plugin name
    if(vizkit3d_plugin_name.isEmpty())
    {
        size_t found;
        std::string str;
        found = path.find_last_of("/\\");
        if(found == std::string::npos)
            str = path;
        else
            str = path.substr(found+1);
        found = str.find_last_of(".");
        if(found != std::string::npos)
        {
            str = str.substr(0,found);
            if(!str.empty())
                setPluginName(QString::fromStdString(str));
        }
    }

    setDirty();
    emit propertyChanged("modelPath");
}

void RigidBodyStateVisualization::displayCovariance(bool enable)
{ covariance = enable; }
bool RigidBodyStateVisualization::isCovarianceDisplayed() const
{ return covariance; }

void RigidBodyStateVisualization::setColor(base::Vector3d const& color)
{ this->color = color; }

void RigidBodyStateVisualization::displayCovarianceWithSamples(bool enable)
{ covariance_with_samples = enable; }
bool RigidBodyStateVisualization::isCovarianceDisplayedWithSamples() const
{ return covariance_with_samples; }

ref_ptr<Node> RigidBodyStateVisualization::createMainNode()
{
    Group* group = new Group;
    PositionAttitudeTransform* body_pose =
        new PositionAttitudeTransform();
    if (!body_model)
        resetModel(total_size);
    body_pose->addChild(body_model);
    group->addChild(body_pose);
    return group;
}

void RigidBodyStateVisualization::updateMainNode(Node* node)
{
    Group* group = node->asGroup();
    PositionAttitudeTransform* body_pose =
        dynamic_cast<PositionAttitudeTransform*>(group->getChild(0));

    // Check if we need an uncertainty representation node, and manage the
    // uncertainty child accordingly
    bool needs_uncertainty = covariance && state.hasValidPositionCovariance();
    Uncertainty* uncertainty = 0;
    if (group->getNumChildren() > 1)
    {
        if (needs_uncertainty)
            uncertainty = dynamic_cast<Uncertainty*>(group->getChild(1));
        else
            group->removeChild(1);
    }
    else if (needs_uncertainty)
    {
        uncertainty = new Uncertainty;
        group->addChild(uncertainty);
    }

    // Reset the body model if needed
    osg::Node* body_node = body_pose->getChild(0);
    if (body_node != this->body_model)
        body_pose->setChild(0, this->body_model);

    if (forcePositionDisplay || state.hasValidPosition())
    {
        pos.set(state.position.x(), state.position.y(), state.position.z());
        body_pose->setPosition(pos);
    }
    if (needs_uncertainty)
    {
        if (covariance_with_samples)
            uncertainty->showSamples();
        else
            uncertainty->hideSamples();

        uncertainty->setMean(static_cast<Eigen::Vector3d>(state.position));
        uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(state.cov_position));
    }
    if (forceOrientationDisplay || state.hasValidOrientation())
    {
        orientation.set(state.orientation.x(),
                state.orientation.y(),
                state.orientation.z(),
                state.orientation.w());
        body_pose->setAttitude(orientation);
    }
}

void RigidBodyStateVisualization::updateDataIntern( const base::samples::RigidBodyState& state )
{
    this->state = state;
}

}
