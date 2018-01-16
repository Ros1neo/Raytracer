#include "renderingWidget.h"

#include "glPrimitives.h"
#include "mesh.h"
#include "parser.h"

#include <filesystem/resolver.h>
#include <nanogui/slider.h>
#include <nanogui/checkbox.h>
#include <nanogui/layout.h>
#include <thread>

RenderingWidget::RenderingWidget() :
    nanogui::Screen(Vector2i(512,512+36), "SIRE raytracer")
{
    m_drawCamera = false;
    m_drawRay = false;
    m_drawRays = false;
    m_renderingDone = true;

    /* Add some UI elements to adjust the exposure value */
    using namespace nanogui;
    m_panel = new Widget(this);
    m_panel->setLayout(new BoxLayout(BoxLayout::Horizontal, BoxLayout::Middle, 10, 10));
    m_slider = new Slider(m_panel);
    m_slider->setValue(0.5f);
    m_slider->setFixedWidth(150);
    m_slider->setCallback(
                [&](float value) {
        m_scale = std::pow(2.f, (value - 0.5f) * 20);
    }
    );
    m_checkbox = new CheckBox(m_panel,"srgb");
    m_checkbox->setChecked(true);
    m_checkbox->setCallback(
                [&](bool value) {
        m_srgb = value ? 1 : 0;
    }
    );

    m_panel->setSize(Eigen::Vector2i(512,36));
    performLayout(mNVGContext);
    m_panel->setPosition(Eigen::Vector2i((512 - m_panel->size().x()) / 2, 512));

    initializeGL();

    drawAll();
    setVisible(true);
}

RenderingWidget::~RenderingWidget()
{
    m_program.free();
    m_flatProgram.free();
}

void RenderingWidget::initializeGL()
{
    std::cout << "Using OpenGL version: \"" << glGetString(GL_VERSION) << "\"" << std::endl;

    // load the default shaders
    m_program.initFromFiles("simple", SIRE_DIR"/shaders/simple.vert", SIRE_DIR"/shaders/simple.frag");
    m_flatProgram.initFromFiles("flat", SIRE_DIR"/shaders/flat.vert", SIRE_DIR"/shaders/flat.frag");
    m_tonemapProgram.initFromFiles("flat", SIRE_DIR"/shaders/tonemap.vert", SIRE_DIR"/shaders/tonemap.frag");

    MatrixXu indices(3, 2); /* Draw 2 triangles */
    indices.col(0) << 0, 1, 2;
    indices.col(1) << 2, 3, 0;

    MatrixXf positions(2, 4);
    positions.col(0) << 0, 0;
    positions.col(1) << 1, 0;
    positions.col(2) << 1, 1;
    positions.col(3) << 0, 1;

    m_tonemapProgram.bind();
    m_tonemapProgram.uploadIndices(indices);
    m_tonemapProgram.uploadAttrib("position", positions);

    /* Allocate texture memory for the rendered image */
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // Assign camera to trackball
    m_trackball.setCamera(&m_GLCamera);
}

void RenderingWidget::drawContents()
{

    if(m_resultImage) // raytracing in progress
    {
        /* Reload the partially rendered image onto the GPU */
        m_resultImage->lock();
        int borderSize = m_resultImage->getBorderSize();
        const Vector2i &size = m_resultImage->getSize();
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, m_texture);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, m_resultImage->cols());
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, size.x(), size.y(),
                     0, GL_RGBA, GL_FLOAT, (uint8_t *) m_resultImage->data() +
                     (borderSize * m_resultImage->cols() + borderSize) * sizeof(Color4f));
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        m_resultImage->unlock();

        glViewport(0, 36, mPixelRatio*size[0], mPixelRatio*size[1]);
        m_tonemapProgram.bind();
        m_tonemapProgram.setUniform("scale", m_scale);
        m_tonemapProgram.setUniform("srgb", m_srgb);
        m_tonemapProgram.setUniform("source", 0);
        m_tonemapProgram.drawIndexed(GL_TRIANGLES, 0, 2);
    }
    else if(m_scene) // scene loaded, OpenGL rendering
    {
        glEnable(GL_DEPTH_TEST);
        glViewport(0, 36, m_GLCamera.vpWidth(), m_GLCamera.vpHeight());
        glClearColor(m_scene->backgroundColor()[0],m_scene->backgroundColor()[1],m_scene->backgroundColor()[2],1);
        glScissor(0, 36, m_GLCamera.vpWidth(), m_GLCamera.vpHeight());
        glEnable(GL_SCISSOR_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glDisable(GL_SCISSOR_TEST);

        m_program.bind();
        glUniformMatrix4fv(m_program.uniform("mat_view"), 1, GL_FALSE, m_GLCamera.viewMatrix().data());
        glUniformMatrix4fv(m_program.uniform("mat_proj"), 1, GL_FALSE, m_GLCamera.projectionMatrix().data());
        Vector3f lightDir = Vector3f(1,1,1).normalized();
        glUniform3fv(m_program.uniform("light_dir"),  1, lightDir.data());

        m_scene->draw();

        m_flatProgram.bind();
        glUniformMatrix4fv(m_flatProgram.uniform("mat_view"), 1, GL_FALSE, m_GLCamera.viewMatrix().data());
        glUniformMatrix4fv(m_flatProgram.uniform("mat_proj"), 1, GL_FALSE, m_GLCamera.projectionMatrix().data());

        if(m_drawCamera)
        {
            glUniform3f(m_flatProgram.uniform("color"),0.75f,0.75f,0.75f);
            m_scene->camera()->draw(&m_flatProgram);
        }

        glUniformMatrix4fv(m_flatProgram.uniform("mat_obj"), 1, GL_FALSE, Eigen::Affine3f::Identity().data());

        if(m_drawRay){
            glUniform3f(m_flatProgram.uniform("color"),0.9f,0.9f,0.9f);

            if(m_hit.foundIntersection()){
                Line::draw(&m_flatProgram,m_ray.origin,m_ray.at(m_hit.t())); //tracer le rayon jusqu'à l'intersection
                Point::draw(&m_flatProgram,m_hit.intersection());}           //tracer l'intersection
            else
                Line::draw(&m_flatProgram,m_ray.origin,m_ray.at(1000));      // tracer le rayon très long

        }
        if(m_drawRays){ //Si on appuie sur la touche J
            int h = m_scene->camera()->vpHeight();
            int w = m_scene->camera()->vpWidth();
            Ray ray_temp;
            for (int i = 0; i<h; i++){
                for(int j=0;j<w;j++){
                    Point2i point(i,j);//Point à faire parcourir
                    m_scene->camera()->convertClickToLine(point,ray_temp.origin,ray_temp.direction); // Comment stocker la direction des centres plutôt?
                    Line::draw(&m_flatProgram,ray_temp.origin,ray_temp.at(1));
                }}
        }

    }

    glViewport(0, 0, mFBSize[0], mFBSize[1]);
}

void RenderingWidget::framebufferSizeChanged()
{
    m_GLCamera.setViewport(mFBSize[0],mFBSize[1]-36);
    m_panel->setSize(Eigen::Vector2i(m_GLCamera.vpWidth(),36));
    performLayout(mNVGContext);
    m_panel->setPosition(Eigen::Vector2i((m_GLCamera.vpWidth() - m_panel->size().x()) / 2, m_GLCamera.vpHeight()));
}

void RenderingWidget::loadScene(const std::string& filename)
{
    if(filename.size()>0) {
        filesystem::path path(filename);

        if (path.extension() != "scn")
            return;
        m_renderingDone = true;
        if(m_resultImage) {
            delete m_resultImage;
            m_resultImage = nullptr;
        }

        getFileResolver()->prepend(path.parent_path());

        Object* root = loadFromXML(filename);
        if (root->getClassType() == Object::EScene){
            if (m_scene)
                delete m_scene;
            m_scene = static_cast<Scene*>(root);
            m_scene->attachShaderToShapes(&m_program);
            m_scene->attachShaderToLights(&m_flatProgram);
            std::cout << m_scene->toString() << std::endl;
            m_curentFilename = filename;

            // Update GUI
            m_GLCamera = Camera(*m_scene->camera());
            setSize(m_GLCamera.outputSize() + Eigen::Vector2i(0,36));
            glfwSetWindowSize(glfwWindow(),m_GLCamera.vpWidth(),m_GLCamera.vpHeight()+36);
            m_panel->setSize(Eigen::Vector2i(m_GLCamera.vpWidth(),36));
            performLayout(mNVGContext);
            m_panel->setPosition(Eigen::Vector2i((m_GLCamera.vpWidth() - m_panel->size().x()) / 2, m_GLCamera.vpHeight()));
        }
        drawAll();
    }
}

void RenderingWidget::loadImage(const std::string &filename)
{
    Bitmap bitmap(filename);
    m_resultImage = new ImageBlock(Eigen::Vector2i(bitmap.cols(), bitmap.rows()));
    m_resultImage->fromBitmap(bitmap);
    m_renderingDone = false;
    // Update GUI
    setSize(Eigen::Vector2i(m_resultImage->cols(), m_resultImage->rows()+36));
    glfwSetWindowSize(glfwWindow(),m_resultImage->cols(), m_resultImage->rows()+36);
    m_panel->setSize(Eigen::Vector2i(mFBSize[0],36));
    performLayout(mNVGContext);
    m_panel->setPosition(Eigen::Vector2i((mFBSize[0] - m_panel->size().x()) / 2, m_resultImage->rows()));
}

void render(Scene* scene, ImageBlock* result, std::string outputName, bool* done)
{
    if(!scene)
        return;

    int t = clock();

    const Camera *camera = scene->camera();
    const Integrator* integrator = scene->integrator();
    integrator->preprocess(scene);

    float tanfovy2 = tan(camera->fovY()*0.5);
    Vector3f camX = camera->right() * tanfovy2 * camera->nearDist() * float(camera->vpWidth())/float(camera->vpHeight());
    Vector3f camY = camera->up() * tanfovy2 * camera->nearDist();
    Vector3f camF = camera->direction() * camera->nearDist();

    /// TODO:
    ///  1. iterate over the image pixels
    ///  2. generate a primary ray
    ///  3. call the integartor to compute the radiance along this ray
    ///  4. write this radiance in the result image

    t = clock() - t;
    std::cout << "Raytracing time : " << float(t)/CLOCKS_PER_SEC << "s"<<std::endl;

    Bitmap* img = result->toBitmap();
    img->save(outputName);
    delete img;
    *done = true;
}

bool RenderingWidget::keyboardEvent(int key, int scancode, bool press, int modifiers)
{
    if(press) {
        switch(key)
        {
        case GLFW_KEY_L:
        {
            std::string filename = nanogui::file_dialog( { {"scn", "Scene file"}, {"exr", "Image file"} }, false);
            filesystem::path path(filename);
            if (path.extension() == "scn")
                loadScene(filename);
            else if(path.extension() == "exr")
                loadImage(filename);
            return true;
        }
        case GLFW_KEY_C:
        {
            if(m_scene) {
                m_scene->setCamera(new Camera(m_GLCamera));
                drawAll();
            }
            return true;
        }
        case GLFW_KEY_H:
        {
            m_drawCamera = !m_drawCamera;
            drawAll();
            return true;
        }
        case GLFW_KEY_J:
        {
            m_drawRays = !m_drawRays;
            drawAll();
            return true;
        }
        case GLFW_KEY_R:
        {
            if(m_scene && m_renderingDone) {
                m_renderingDone = false;
                /* Determine the filename of the output bitmap */
                std::string outputName = m_curentFilename;
                size_t lastdot = outputName.find_last_of(".");
                if (lastdot != std::string::npos)
                    outputName.erase(lastdot, std::string::npos);
                outputName += ".exr";

                /* Allocate memory for the entire output image */
                if(m_resultImage)
                    delete m_resultImage;
                m_resultImage = new ImageBlock(m_scene->camera()->outputSize());

                std::thread render_thread(render,m_scene,m_resultImage,outputName,&m_renderingDone);
                render_thread.detach();
            }
            return true;
        }
        default:
            break;
        }
    }
    return Screen::keyboardEvent(key,scancode,press,modifiers);
}

void RenderingWidget::dropEvent(const std::vector<std::string> &filenames)
{
    // only tries to load the first file
    filesystem::path path(filenames.front());
    if (path.extension() == "scn")
        loadScene(filenames.front());
    else if(path.extension() == "exr")
        loadImage(filenames.front());

    drawAll();
}

void RenderingWidget::select(const Point2i &point)
{
    ///TODO : trace a ray trough the pixel \param point
    m_scene->camera()->convertClickToLine(point, m_ray.origin, m_ray.direction);
    m_scene->intersect(m_ray,m_hit);
    //throw SireException("RenderingWidget::select not implemented yet.");

    m_drawRay = true;
    drawAll();
}

bool RenderingWidget::mouseButtonEvent(const Eigen::Vector2i &p, int button, bool down, int modifiers)
{
    if(Widget::mouseButtonEvent(p,button,down,modifiers))
        return true;

    if(down) {
        m_mouseCoords = p;
        bool fly = (modifiers == GLFW_MOD_ALT);
        bool shift = (modifiers == GLFW_MOD_SHIFT);
        switch(button)
        {
        case GLFW_MOUSE_BUTTON_LEFT:
            if(shift){
                select(p);
            }
            else if(fly)
            {
                m_currentTrackingMode = TM_LOCAL_ROTATE;
                m_trackball.start(Trackball::Local);
                m_trackball.track(m_mouseCoords);
            }
            else
            {
                m_currentTrackingMode = TM_ROTATE_AROUND;
                m_trackball.start(Trackball::Around);
                m_trackball.track(m_mouseCoords);
            }
            break;
        case GLFW_MOUSE_BUTTON_MIDDLE:
            if(fly)
                m_currentTrackingMode = TM_FLY_Z;
            else
                m_currentTrackingMode = TM_ZOOM;
            break;
        case GLFW_MOUSE_BUTTON_RIGHT:
            m_currentTrackingMode = TM_FLY_PAN;
            break;
        default:
            break;
        }
    }else{
        m_currentTrackingMode = TM_NO_TRACK;
    }
    drawAll();
    return true;
}

bool RenderingWidget::scrollEvent(const Eigen::Vector2i &p, const Eigen::Vector2f &rel)
{
    if(Widget::scrollEvent(p,rel))
        return true;

    m_GLCamera.zoom(rel[1]*0.01);

    drawAll();
    return true;
}

bool RenderingWidget::mouseMotionEvent(const Eigen::Vector2i &p, const Eigen::Vector2i &rel, int button, int modifiers)
{
    if(Widget::mouseMotionEvent(p,rel,button,modifiers))
        return true;

    // tracking
    if(m_currentTrackingMode != TM_NO_TRACK)
    {
        if(m_renderingDone && m_resultImage) {
            delete m_resultImage;
            m_resultImage = nullptr;
        }

        float dx =   float(p[0] - m_mouseCoords[0]) / float(m_GLCamera.vpWidth());
        float dy = - float(p[1] - m_mouseCoords[1]) / float(m_GLCamera.vpHeight());

        // speedup the transformations
        if(modifiers == GLFW_MOD_SHIFT)
        {
            dx *= 10.;
            dy *= 10.;
        }

        switch(m_currentTrackingMode)
        {
        case TM_ROTATE_AROUND:
        case TM_LOCAL_ROTATE:
            m_trackball.track(p);
            break;
        case TM_ZOOM :
            m_GLCamera.zoom(dy*100);
            break;
        case TM_FLY_Z :
            m_GLCamera.localTranslate(Vector3f(0, 0, -dy*200));
            break;
        case TM_FLY_PAN :
            m_GLCamera.localTranslate(Vector3f(dx*200, dy*200, 0));
            break;
        default:
            break;
        }

        drawAll();
    }

    m_mouseCoords = p;
    return true;
}
