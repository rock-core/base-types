#include "RigidBodyStateSE3Visualization.hpp"
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osg/Material>
#include <osgFX/BumpMapping>
#include <osgText/Text>

using namespace osg;
namespace vizkit3d 
{

RigidBodyStateSE3Visualization::RigidBodyStateSE3Visualization(QObject* parent)
    : Vizkit3DPlugin<base::samples::RigidBodyStateSE3>(parent)
    , color(1, 1, 1)
    , total_size(1)
    , main_size(0.1)
    , text_size(0.1)
    , translation(0, 0, 0)
    , rotation(0, 0, 0, 1)
    , body_type(BODY_NONE)
    , texture_dirty(false)
    , bump_mapping_dirty(false)
    , forcePositionDisplay(false)
    , forceOrientationDisplay(false)
    , showPose(true)
    , showVelocity(false)
    , showAcceleration(false)
    , showWrench(false)
    , show_seperate_axes(false)
    , resolution(0.1)
{
    state.pose.position = base::Vector3d::Zero();
    state.pose.orientation = base::Quaterniond::Identity();
    primitivesfactory = osgviz::OsgViz::getModuleInstance<osgviz::PrimitivesFactory>("PrimitivesFactory");
}

RigidBodyStateSE3Visualization::~RigidBodyStateSE3Visualization()
{
}

void RigidBodyStateSE3Visualization::setColor(const Vec4d& color, Geode* geode)
{
    Material *material = new Material();
    material->setDiffuse(Material::FRONT,  Vec4(0.1, 0.1, 0.1, 1.0));
    material->setSpecular(Material::FRONT, Vec4(0.6, 0.6, 0.6, 1.0));
    material->setAmbient(Material::FRONT,  Vec4(0.1, 0.1, 0.1, 1.0));
    material->setEmission(Material::FRONT, color);
    material->setShininess(Material::FRONT, 10.0);

    geode->getOrCreateStateSet()->setAttribute(material);    
}

bool RigidBodyStateSE3Visualization::isPositionDisplayForced() const
{ return forcePositionDisplay; }
void RigidBodyStateSE3Visualization::setPositionDisplayForceFlag(bool flag)
{ forcePositionDisplay = flag; emit propertyChanged("forcePositionDisplay"); }

bool RigidBodyStateSE3Visualization::isOrientationDisplayForced() const
{ return forceOrientationDisplay; }
void RigidBodyStateSE3Visualization::setOrientationDisplayForceFlag(bool flag)
{ forceOrientationDisplay = flag; emit propertyChanged("forceOrientationDisplay"); }

bool RigidBodyStateSE3Visualization::isPoseDisplayed() const {
    return showPose;
}
void RigidBodyStateSE3Visualization::setPoseDisplayed(bool flag) {
    showPose = flag;
    emit propertyChanged("showPose");
    updateModel(total_size);
}

bool RigidBodyStateSE3Visualization::isVelocityDisplayed() const {
    return showVelocity;
}
void RigidBodyStateSE3Visualization::setVelocityDisplayed(bool flag) {
    showVelocity = flag;
    emit propertyChanged("showVelocity");
    updateModel(total_size);
}

bool RigidBodyStateSE3Visualization::isAccelerationDisplayed() const {
    return showAcceleration;
}
void RigidBodyStateSE3Visualization::setAccelerationDisplayed(bool flag) {
    showAcceleration = flag;
    emit propertyChanged("showAcceleration");
    updateModel(total_size);
}

bool RigidBodyStateSE3Visualization::isWrenchDisplayed() const {
    return showWrench;
}
void RigidBodyStateSE3Visualization::setWrenchDisplayed(bool flag) {
    showWrench = flag;
    emit propertyChanged("showWrench");
    updateModel(total_size);
}

void RigidBodyStateSE3Visualization::setTexture(QString const& path)
{ return setTexture(path.toStdString()); }

void RigidBodyStateSE3Visualization::setTexture(std::string const& path)
{
    if (path.empty())
        return clearTexture();

    image = osgDB::readImageFile(path);
    texture_dirty = true;
}

void RigidBodyStateSE3Visualization::clearTexture()
{
    image.release();
    texture_dirty = true;
}

void RigidBodyStateSE3Visualization::addBumpMapping(
                QString const& diffuse_color_map_path,
                QString const& normal_map_path)
{ return addBumpMapping(diffuse_color_map_path.toStdString(),
        normal_map_path.toStdString()); }

void RigidBodyStateSE3Visualization::addBumpMapping(
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
    bump_mapping->setNormalMapTextureUnit(1);
    bump_mapping->setDiffuseTextureUnit(2);
    this->bump_mapping = bump_mapping;
    bump_mapping_dirty = true;
}

void RigidBodyStateSE3Visualization::updateTexture()
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

void RigidBodyStateSE3Visualization::updateBumpMapping()
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

void RigidBodyStateSE3Visualization::removeBumpMapping()
{
    bump_mapping.release();
    bump_mapping_dirty = true;
}

ref_ptr<Group> RigidBodyStateSE3Visualization::createSimpleSphere(double size)
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
  
ref_ptr<Group> RigidBodyStateSE3Visualization::createSimpleBody(double size)
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
        text->setText(state.frame_id);
        text->setCharacterSize(actual_size);
        text->setPosition(osg::Vec3d(actual_size/2,actual_size/2,0));
        geode->addDrawable(text);
    }

    group->addChild(geode);
    
    if (showPose) {
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
    }
    
    if (showVelocity) {
        //linear velocity
        //linear_vel_transform = new Arrow(osg::Vec4f(1,0,0,1));
        linear_vel_transform = primitivesfactory->createArrow(osg::Vec4f(1, 0, 0, 1.0), true);
        group->addChild(linear_vel_transform);
        
        //angular velocity
        //angular_vel_transform = new Arrow(osg::Vec4f(0,1,0,1));
        angular_vel_transform = primitivesfactory->createArrow(osg::Vec4f(0, 1, 0, 1.0), true);
        group->addChild(angular_vel_transform);
    }

    if (showAcceleration) {
        //linear acceleration
        //linear_acc_transform = new Arrow(osg::Vec4f(1,0,0,1));
        linear_acc_transform = primitivesfactory->createArrow(osg::Vec4f(1, 0, 0, 1.0), true);
        group->addChild(linear_acc_transform);
        
        //angular acceleration
        //angular_acc_transform = new Arrow(osg::Vec4f(0,1,0,1));
        angular_acc_transform = primitivesfactory->createArrow(osg::Vec4f(0, 1, 0, 1.0), true);
        group->addChild(angular_acc_transform);
    }

    if (showWrench) {
        wrench_model = new WrenchModel();
        wrench_model->setResolution(resolution);
        wrench_model->seperateAxes(show_seperate_axes);
        group->addChild(wrench_model);
    }

    return group;
}

double RigidBodyStateSE3Visualization::getMainSphereSize() const
{
    return main_size;
}

void RigidBodyStateSE3Visualization::setMainSphereSize(double size)
{
    main_size = size;
    emit propertyChanged("sphereSize");
    // This triggers an update of the model if we don't have a custom model
    setSize(total_size);
}

double RigidBodyStateSE3Visualization::getTextSize() const
{
    return text_size;
}

void RigidBodyStateSE3Visualization::setTextSize(double size)
{
    text_size = size;
    emit propertyChanged("textSize");
    // This triggers an update of the model if we don't have a custom model
    setSize(total_size);
}

void RigidBodyStateSE3Visualization::setSize(double size)
{
    total_size = size;
    emit propertyChanged("size");
    updateModel(total_size);
}

double RigidBodyStateSE3Visualization::getSize() const
{
    return total_size;
}

void RigidBodyStateSE3Visualization::updateModel(double size) {
    if (body_type == BODY_SIMPLE)
        resetModel(size);
    else if (body_type == BODY_SPHERE)
        resetModelSphere(size);
}

void RigidBodyStateSE3Visualization::resetModel(double size)
{
    body_type  = BODY_SIMPLE;
    body_model = createSimpleBody(size);
    setDirty();
}

void RigidBodyStateSE3Visualization::resetModelSphere(double size)
{
    body_type  = BODY_SPHERE;
    body_model = createSimpleSphere(size);
    setDirty();
}

QString RigidBodyStateSE3Visualization::getModelPath() const
{
    if (body_type == BODY_SPHERE)
        return "sphere";
    else if (body_type == BODY_SIMPLE)
        return "simple";
    else
        return model_path;
}

void RigidBodyStateSE3Visualization::loadModel(QString const& path)
{
    return loadModel(path.toStdString());
}

void RigidBodyStateSE3Visualization::loadModel(std::string const& path)
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

QVector3D RigidBodyStateSE3Visualization::getTranslation() const
{
    return QVector3D(translation.x(), translation.y(), translation.z());
}
void RigidBodyStateSE3Visualization::setTranslation(QVector3D const& v)
{
    translation = osg::Vec3(v.x(), v.y(), v.z());
    setDirty();
}

void RigidBodyStateSE3Visualization::setRotation(QQuaternion const& q)
{
    rotation = osg::Quat(q.x(), q.y(), q.z(), q.scalar());
    setDirty();
}

void RigidBodyStateSE3Visualization::setColor(base::Vector3d const& color)
{ this->color = color; }

ref_ptr<Node> RigidBodyStateSE3Visualization::createMainNode()
{
    Group* group = new Group;
    PositionAttitudeTransform* body_pose =
        new PositionAttitudeTransform();
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

void RigidBodyStateSE3Visualization::updateMainNode(Node* node)
{
    Group* group = node->asGroup();
    PositionAttitudeTransform* body_pose =
        dynamic_cast<PositionAttitudeTransform*>(group->getChild(0));

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

    bool hasValidPose = state.hasValidPose();
    if (forcePositionDisplay || hasValidPose) {
	    osg::Vec3d pos(state.pose.position.x(), state.pose.position.y(), state.pose.position.z());
        body_pose->setPosition(pos + translation);
    }
    if (forceOrientationDisplay || hasValidPose) {
	    osg::Quat orientation(state.pose.orientation.x(),
                state.pose.orientation.y(),
                state.pose.orientation.z(),
                state.pose.orientation.w());
        body_pose->setAttitude(rotation * orientation);
    }
    
    if (showVelocity && state.hasValidTwist()) {
        osg::Quat Q;
        const base::Twist& t = state.twist;
        osg::Vec3d lin_vel(t.linear.x(), t.linear.y(), t.linear.z());
        Q.makeRotate(osg::Vec3d(0,0,1), lin_vel);
        linear_vel_transform->setScale(osg::Vec3f(resolution, resolution, resolution*lin_vel.length()));
        linear_vel_transform->setAttitude(Q);

        osg::Vec3d ang_vel(t.angular.x(), t.angular.y(), t.angular.z());
        Q.makeRotate(osg::Vec3d(0,0,1), ang_vel);
        angular_vel_transform->setScale(osg::Vec3f(resolution, resolution, resolution*ang_vel.length()));
        angular_vel_transform->setAttitude(Q);
    }

    if (showAcceleration && state.hasValidAcceleration()) {
        osg::Quat Q;
        const base::Acceleration& acc = state.acceleration;
        osg::Vec3d lin_acc(acc.linear.x(), acc.linear.y(), acc.linear.z());
        Q.makeRotate(osg::Vec3d(0,0,1), lin_acc);
        linear_acc_transform->setScale(osg::Vec3f(resolution, resolution, resolution*lin_acc.length()));
        linear_acc_transform->setAttitude(Q);

        osg::Vec3d ang_acc(acc.angular.x(), acc.angular.y(), acc.angular.z());
        Q.makeRotate(osg::Vec3d(0,0,1), ang_acc);
        angular_acc_transform->setScale(osg::Vec3f(resolution, resolution, resolution*ang_acc.length()));
        angular_acc_transform->setAttitude(Q);
    }

    if (showWrench && state.hasValidWrench()) {

        wrench_model->setResolution(resolution);
        wrench_model->update(state.wrench);
    }
}

void RigidBodyStateSE3Visualization::updateDataIntern( const base::samples::RigidBodyStateSE3& state )
{
    this->state = state;
}

}

//Macro that makes this plugin loadable in ruby, this is optional.
//VizkitQtPlugin(WrenchVisualization)