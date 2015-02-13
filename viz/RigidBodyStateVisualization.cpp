#include "RigidBodyStateVisualization.hpp"
#include "Uncertainty.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osgFX/BumpMapping>

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
    , texture_dirty(false)
    , bump_mapping_dirty(false)
    , forcePositionDisplay(false)
    , forceOrientationDisplay(false)
{
}

RigidBodyStateVisualization::~RigidBodyStateVisualization()
{
}

void RigidBodyStateVisualization::setColor(const osg::Vec4d& color, osg::Geode* geode)
{
    osg::Material *material = new osg::Material();
    material->setDiffuse(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(osg::Material::FRONT,  osg::Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(osg::Material::FRONT, color);
    material->setShininess(osg::Material::FRONT, 10.0);

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

void RigidBodyStateVisualization::setTexture(QString const& path)
{ return setTexture(path.toStdString()); }
void RigidBodyStateVisualization::setTexture(std::string const& path)
{
    if (path.empty())
        return clearTexture();

    image = osgDB::readImageFile(path);
    texture_dirty = true;
}

void RigidBodyStateVisualization::clearTexture()
{
    image.release();
    texture_dirty = true;
}

void RigidBodyStateVisualization::addBumpMapping(
                QString const& diffuse_color_map_path,
                QString const& normal_map_path)
{ return addBumpMapping(diffuse_color_map_path.toStdString(),
        normal_map_path.toStdString()); }
void RigidBodyStateVisualization::addBumpMapping(
                std::string const& diffuse_color_map_path,
                std::string const& normal_map_path)
{
    if (!body_model->asGeode())
    {
        std::cerr << "model is not a geometry, cannot use bump mapping" << std::endl;
        return;
    }

    // Setup the textures
    diffuse_image = osgDB::readImageFile(diffuse_color_map_path);
    normal_image  = osgDB::readImageFile(normal_map_path);

    // And add bump mapping
    osgFX::BumpMapping* bump_mapping = new osgFX::BumpMapping();
    bump_mapping->setLightNumber(0);
    bump_mapping->setNormalMapTextureUnit(0);
    bump_mapping->setDiffuseTextureUnit(1);
    this->bump_mapping = bump_mapping;
    bump_mapping_dirty = true;
}

void RigidBodyStateVisualization::updateTexture()
{
    ref_ptr<StateSet> state = body_model->getOrCreateStateSet();
    if (!image)
    {
        state->setTextureMode(0, GL_TEXTURE_2D, StateAttribute::OFF);
        return;
    }
    else
    {
        texture->setImage(image.get());
        state->setTextureAttributeAndModes(0, texture, StateAttribute::ON);
    }
}

void RigidBodyStateVisualization::updateBumpMapping()
{
    ref_ptr<StateSet> state = body_model->getOrCreateStateSet();

    if (!bump_mapping)
    {
        diffuse_image.release();
        normal_image.release();
        state->setTextureMode(0, GL_TEXTURE_2D, StateAttribute::OFF);
        state->setTextureMode(1, GL_TEXTURE_2D, StateAttribute::OFF);
        return;
    }
    else
    {

        ref_ptr<Geometry> geometry = body_model->asGeode()->getDrawable(0)->asGeometry();
        ref_ptr<Array> tex_coord = geometry->getTexCoordArray(0);
        geometry->setTexCoordArray(1, tex_coord);
        geometry->setTexCoordArray(2, tex_coord);

        diffuse_texture->setImage(diffuse_image.get());
        normal_texture->setImage(normal_image.get());
        state->setTextureAttributeAndModes(0, normal_texture, StateAttribute::ON);
        state->setTextureAttributeAndModes(1, diffuse_texture, StateAttribute::ON);
        bump_mapping->prepareChildren();
    }
}

void RigidBodyStateVisualization::removeBumpMapping()
{
    bump_mapping.release();
    bump_mapping_dirty = true;
}

osg::ref_ptr<osg::Group> RigidBodyStateVisualization::createSimpleSphere(double size)
{   
    osg::ref_ptr<osg::Group> group = new osg::Group();
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3f(0,0,0), main_size * size);
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp);
    spd->setColor(osg::Vec4f(color.x(), color.y(), color.z(), 1.0));
    geode->addDrawable(spd);
    group->addChild(geode);
    
    return group;
}
  
osg::ref_ptr<osg::Group> RigidBodyStateVisualization::createSimpleBody(double size)
{   
    osg::ref_ptr<osg::Group> group = new osg::Group();
    
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    osg::ref_ptr<osg::Sphere> sp = new osg::Sphere(osg::Vec3f(0,0,0), main_size * size);
    osg::ref_ptr<osg::ShapeDrawable> spd = new osg::ShapeDrawable(sp);
    spd->setColor(osg::Vec4f(color.x(), color.y(), color.z(), 1.0));
    geode->addDrawable(spd);
    group->addChild(geode);
    
    //up
    osg::ref_ptr<osg::Geode> c1g = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> c1 = new osg::Cylinder(osg::Vec3f(0, 0, size / 2), size / 40, size);
    osg::ref_ptr<osg::ShapeDrawable> c1d = new osg::ShapeDrawable(c1);
    c1g->addDrawable(c1d);
    setColor(osg::Vec4f(0, 0, 1.0, 1.0), c1g);
    group->addChild(c1g);
    
    //north direction
    osg::ref_ptr<osg::Geode> c2g = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> c2 = new osg::Cylinder(osg::Vec3f(0, size / 2, 0), size / 40, size);
    c2->setRotation(osg::Quat(M_PI/2.0, osg::Vec3d(1,0,0)));
    osg::ref_ptr<osg::ShapeDrawable> c2d = new osg::ShapeDrawable(c2);
    c2g->addDrawable(c2d);
    setColor(osg::Vec4f(0.0, 1.0, 0, 1.0), c2g);
    group->addChild(c2g);

    //east
    osg::ref_ptr<osg::Geode> c3g = new osg::Geode();
    osg::ref_ptr<osg::Cylinder> c3 = new osg::Cylinder(osg::Vec3f(size / 2, 0, 0), size / 40, size);
    c3->setRotation(osg::Quat(M_PI/2.0, osg::Vec3d(0,1,0)));
    osg::ref_ptr<osg::ShapeDrawable> c3d = new osg::ShapeDrawable(c3);
    c3g->addDrawable(c3d);
    setColor(osg::Vec4f(1.0, 0.0, 0, 1.0), c3g);
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

    osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(path);
    body_type  = BODY_CUSTOM_MODEL;
    body_model = model;
    if (!body_model->asGeode())
        std::cerr << "model is not a geode, using bump mapping will not be possible" << std::endl;
    else if (!body_model->asGeode()->getDrawable(0))
        std::cerr << "model does not contain a mesh, using bump mapping will not be possible" << std::endl;
    else if (!body_model->asGeode()->getDrawable(0)->asGeometry())
        std::cerr << "model does not contain a mesh, using bump mapping will not be possible" << std::endl;

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

osg::ref_ptr<osg::Node> RigidBodyStateVisualization::createMainNode()
{
    osg::Group* group = new osg::Group;
    osg::PositionAttitudeTransform* body_pose =
        new osg::PositionAttitudeTransform();
    if (!body_model)
        resetModel(total_size);
    body_pose->addChild(body_model);
    group->addChild(body_pose);

    texture = new osg::Texture2D;
    texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
    texture->setWrap(Texture::WRAP_T, Texture::REPEAT);
    texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    diffuse_texture = new osg::Texture2D;
    diffuse_texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
    diffuse_texture->setWrap(Texture::WRAP_T, Texture::REPEAT);
    diffuse_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    diffuse_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);
    normal_texture = new osg::Texture2D;
    normal_texture->setWrap(Texture::WRAP_S, Texture::REPEAT);
    normal_texture->setWrap(Texture::WRAP_T, Texture::REPEAT);
    normal_texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
    normal_texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

    return group;
}

void RigidBodyStateVisualization::updateMainNode(osg::Node* node)
{
    osg::Group* group = node->asGroup();
    osg::PositionAttitudeTransform* body_pose =
        dynamic_cast<osg::PositionAttitudeTransform*>(group->getChild(0));

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
    Node* body_node = body_pose->getChild(0);
    if (bump_mapping && body_node != bump_mapping)
    {
        bump_mapping->addChild(body_model);
        body_pose->setChild(0, bump_mapping);
    }
    else if (body_node != body_model)
        body_pose->setChild(0, body_model);

    if (texture_dirty)
        updateTexture();
    if (bump_mapping_dirty)
        updateBumpMapping();

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
