#include "RigidBodyStateVisualization.hpp"
#include "Uncertainty.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osgFX/BumpMapping>
#include <osgText/Text>
#include <boost/concept_check.hpp>

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
    , text_size(0.0)
    , translation(0, 0, 0)
    , rotation(0, 0, 0, 1)
    , body_type(BODY_NONE)
    , texture_dirty(false)
    , bump_mapping_dirty(false)
    , forcePositionDisplay(false)
    , forceOrientationDisplay(false)
{
    /*
    state = base::samples::RigidBodyState::invalid();
    state.position = base::Vector3d::Zero();
    state.orientation = base::Quaterniond::Identity();
    */
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
{ forcePositionDisplay = flag; emit propertyChanged("forcePositionDisplay"); }
bool RigidBodyStateVisualization::isOrientationDisplayForced() const
{ return forceOrientationDisplay; }
void RigidBodyStateVisualization::setOrientationDisplayForceFlag(bool flag)
{ forceOrientationDisplay = flag; emit propertyChanged("forceOrientationDisplay"); }

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

void RigidBodyStateVisualization::addBumpMapping(QString const& diffuse_color_map_path,
        QString const& normal_map_path) { 
    return addBumpMapping(diffuse_color_map_path.toStdString(), 
            normal_map_path.toStdString()); 
}
        
void RigidBodyStateVisualization::addBumpMapping(
                std::string const& diffuse_color_map_path,
                std::string const& normal_map_path) {
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
    bump_mapping->setNormalMapTextureUnit(1);
    bump_mapping->setDiffuseTextureUnit(2);
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
        state->setTextureMode(1, GL_TEXTURE_2D, StateAttribute::OFF);
        state->setTextureMode(2, GL_TEXTURE_2D, StateAttribute::OFF);
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
        state->setTextureAttributeAndModes(1, normal_texture, StateAttribute::ON);
        state->setTextureAttributeAndModes(2, diffuse_texture, StateAttribute::ON);
        bump_mapping->prepareChildren();
    }
}

void RigidBodyStateVisualization::removeBumpMapping()
{
    bump_mapping.release();
    bump_mapping_dirty = true;
}

ref_ptr<Group> RigidBodyStateVisualization::createSimpleSphere(double size)
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
    if(text_size>0.0)
    {
        double actual_size = text_size * size;
        ref_ptr<osgText::Text> text= new osgText::Text;
        if(states.size() == 1)
        {
            text->setText(states[0].sourceFrame);
        }
        text->setCharacterSize(actual_size);
        text->setPosition(osg::Vec3d(actual_size/2,actual_size/2,0));
        geode->addDrawable(text);
    }

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
    emit propertyChanged("sphereSize");
    // This triggers an update of the model if we don't have a custom model
    setSize(total_size);
}

double RigidBodyStateVisualization::getTextSize() const
{
    return text_size;
}

void RigidBodyStateVisualization::setTextSize(double size)
{
    text_size = size;
    emit propertyChanged("textSize");
    // This triggers an update of the model if we don't have a custom model
    setSize(total_size);
}

void RigidBodyStateVisualization::setSize(double size)
{
    total_size = size;
    emit propertyChanged("size");
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
    emit propertyChanged("modelPath");
}
QVector3D RigidBodyStateVisualization::getTranslation() const
{
    return QVector3D(translation.x(), translation.y(), translation.z());
}
void RigidBodyStateVisualization::setTranslation(QVector3D const& v)
{
    translation = osg::Vec3(v.x(), v.y(), v.z());
    setDirty();
}

void RigidBodyStateVisualization::setRotation(QQuaternion const& q)
{
    rotation = osg::Quat(q.x(), q.y(), q.z(), q.scalar());
    setDirty();
}

void RigidBodyStateVisualization::displayCovariance(bool enable)
{ covariance = enable; emit propertyChanged("displayCovariance"); }
bool RigidBodyStateVisualization::isCovarianceDisplayed() const
{ return covariance; }

void RigidBodyStateVisualization::setColor(base::Vector3d const& color)
{ this->color = color; }

void RigidBodyStateVisualization::displayCovarianceWithSamples(bool enable)
{ covariance_with_samples = enable; emit propertyChanged("displayCovarianceWithSamples"); }
bool RigidBodyStateVisualization::isCovarianceDisplayedWithSamples() const
{ return covariance_with_samples; }

ref_ptr<Node> RigidBodyStateVisualization::createMainNode()
{  
    Group* group = new Group;

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

void RigidBodyStateVisualization::updateMainNode(Node* node)
{    
    Group* group = node->asGroup();
    
    group->removeChildren(0, group->getNumChildren());
    
    if (texture_dirty)
        updateTexture();
    // Bump mapping not added yet, seems not to work anyway.
    //if (bump_mapping_dirty)
    //    updateBumpMapping();
    
    std::vector<base::samples::RigidBodyState>::iterator it;
    for(it = states.begin(); it != states.end(); it++) {
        PositionAttitudeTransform* body_pose = new PositionAttitudeTransform();
        if (!body_model) {
            resetModel(total_size);
        }

        body_pose->addChild(body_model);
        group->addChild(body_pose);
        
        if (forcePositionDisplay || it->hasValidPosition()) {
            osg::Vec3d pos(it->position.x(), it->position.y(), it->position.z());
            body_pose->setPosition(pos + translation);
        }
        
        if (forceOrientationDisplay || it->hasValidOrientation()) {
            osg::Quat orientation(it->orientation.x(),
                    it->orientation.y(),
                    it->orientation.z(),
                    it->orientation.w());
            body_pose->setAttitude(rotation * orientation);
        }  
        
        // Add uncertainties if required.
        bool needs_uncertainty = covariance && it->hasValidPositionCovariance();
        if(needs_uncertainty) {
            Uncertainty* uncertainty = new Uncertainty;
            uncertainty->setMean(static_cast<Eigen::Vector3d>(it->position));
            uncertainty->setCovariance(static_cast<Eigen::Matrix3d>(it->cov_position));
            group->addChild(uncertainty);
            
            if (covariance_with_samples) {
                uncertainty->showSamples();
            } else {
                uncertainty->hideSamples();
            }
        }
    }
    
    /*
        // Reset the body model if needed
        Node* body_node = body_pose->getChild(i);
        if (bump_mapping && body_node != bump_mapping)
        {
            bump_mapping->addChild(body_model);
            body_pose->setChild(i, bump_mapping);
        }
        else if (body_node != body_model)
            body_pose->setChild(i, body_model);
    */
}

void RigidBodyStateVisualization::updateDataIntern( const base::samples::RigidBodyState& state )
{
    states.clear();
    states.resize(1);
    states[0] = state;
}

void RigidBodyStateVisualization::updateDataIntern( const std::vector<base::samples::RigidBodyState>& states )
{
    this->states = states; 
}

}
