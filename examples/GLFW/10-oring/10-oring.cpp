//==============================================================================
/*
    Software License Agreement (BSD License)
    Copyright (c) 2003-2024, CHAI3D
    (www.chai3d.org)

    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.

    * Neither the name of CHAI3D nor the names of its contributors may
    be used to endorse or promote products derived from this software
    without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE. 

    \author    <http://www.chai3d.org>
    \author    Francois Conti
    \version   3.3.0
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//------------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled 
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

// a world that contains all objects of the virtual environment
cWorld* world;

// a camera to render the world in the window display
cCamera* camera;

// a viewport to display the scene viewed by the camera
cViewport* viewport = nullptr;

// a light source to illuminate the objects in the world
cSpotLight *light;

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;

// a virtual tool representing the haptic device in the scene
cToolCursor* tool;

// a virtual torus like object
cShapeTorus* object;

// angular velocity of object
cVector3d angVel(0.0, 0.0, 0.1);

// a font for rendering text
cFontPtr font;

// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;

// level widget to display interaction forces
cLevel* level;

// dial widget to display velocity of torus
cDial* dial;

// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;

// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = nullptr;

// current size of GLFW window
int windowW = 0;
int windowH = 0;

// current size of GLFW framebuffer
int framebufferW = 0;
int framebufferH = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window is resized
void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window is resized
void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when the window framebuffer is resized
void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void onErrorCallback(int a_error, const char* a_description);

// callback when a key is pressed
void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// callback when window content scaling is modified
void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale);

// this function renders the scene
void renderGraphics(void);

// this function contains the main haptics simulation loop
void renderHaptics(void);

// this function closes the application
void close(void);


//==============================================================================
/*
    DEMO:   10-oring.cpp

    This demonstration illustrates the use of a simple 3D shape primitive
    to create a virtual world. A virtual tool is used to represent the
    haptic device. Haptic interaction between the tool and the object
    is computed through the force models defined in the CHAI3D framework.
    stiffness properties can be modified by adjusting the
    parameters of the haptic effect classes.

    In the main haptics loop function  "renderHaptics()" , the position
    of the haptic device is retrieved at each simulation iteration.
    The interaction forces are then computed and sent to the device.
    Finally, a simple dynamics model is used to simulate the behavior
    of the o-ring like object.
*/
//==============================================================================

int main(int argc, char* argv[])
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: 10-oring" << endl;
    cout << "Copyright 2003-2024" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Texture   (ON/OFF)" << endl;
    cout << "[2] - Wireframe (ON/OFF)" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;


    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set GLFW error callback
    glfwSetErrorCallback(onErrorCallback);

    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    windowW = 0.8 * mode->height;
    windowH = 0.5 * mode->height;
    int x = 0.5 * (mode->width - windowW);
    int y = 0.5 * (mode->height - windowH);

    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    // enable double buffering
    glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);

    // set the desired number of samples to use for multisampling
    glfwWindowHint(GLFW_SAMPLES, 4);

    // specify that window should be resized based on monitor content scale
    glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);

    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(windowW, windowH, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // set GLFW key callback
    glfwSetKeyCallback(window, onKeyCallback);

    // set GLFW window size callback
    glfwSetWindowSizeCallback(window, onWindowSizeCallback);

    // set GLFW framebuffer size callback
    glfwSetFramebufferSizeCallback(window, onFrameBufferSizeCallback);

    // set GLFW window content scaling callback
    glfwSetWindowContentScaleCallback(window, onWindowContentScaleCallback);

    // get width and height of window
    glfwGetFramebufferSize(window, &framebufferW, &framebufferH);

    // set position of window
    glfwSetWindowPos(window, x, y);

    // set window size
    glfwSetWindowSize(window, windowW, windowH);

    // set GLFW current display context
    glfwMakeContextCurrent(window);

    // set GLFW swap interval for the current display context
    glfwSwapInterval(swapInterval);


#ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
#endif


    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();

    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);
    world->addChild(camera);

    // position and orient the camera
    camera->set(cVector3d(3.0, 0.0, 0.0),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(1.0, 10.0);

    // set stereo mode
    camera->setStereoMode(stereoMode);

    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);

    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cSpotLight(world);

    // attach light to camera
    camera->addChild(light);

    // enable light source
    light->setEnabled(true);

    // position the light source
    light->setLocalPos( 0.0, 0.5, 0.0);

    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);

    // enable this light source to generate shadows
    light->setShadowMapEnabled(true);

    // set the resolution of the shadow map
    light->m_shadowMap->setQualityLow();
    //light->m_shadowMap->setQualityMedium();

    // set light cone half angle
    light->setCutOffAngleDeg(20);


    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------

    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    // define a radius for the virtual tool (sphere)
    tool->setRadius(0.03);

    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);

    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when 
    // the tool is located inside an object for instance. 
    tool->setWaitForSmallForce(true);

    // start the haptic tool
    tool->start();


    //--------------------------------------------------------------------------
    // CREATE OBJECTS
    //--------------------------------------------------------------------------

    // get current path
    bool fileload;
    string currentpath = cGetCurrentPath();

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // stiffness properties
    double maxStiffness	= hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;

    // create a torus object (o-ring)
    object = new cShapeTorus(0.24, 0.50);

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

    // rotate object of 90 degrees around its y-axis so that it
    // stands vertically
    object->rotateAboutGlobalAxisDeg( cVector3d(0, 1, 0), 90);

    // set the material stiffness to 100% of the maximum permitted stiffness
    // of the haptic device
    object->m_material->setStiffness(1.0 * maxStiffness);

    // set some environmental texture and material properties
    object->m_texture = cTexture2d::create();

    // load environmenta; texture file
    fileload = object->m_texture->loadFromFile(currentpath + "../resources/images/spheremap-6.jpg");
    if (!fileload)
    {
        cout << "Error - Texture image failed to load correctly." << endl;
        close();
        return (-1);
    }

    // setup texture and material properties
    object->setUseTexture(true);
    object->m_texture->setSphericalMappingEnabled(true);
    object->m_material->m_ambient.set(0.9f, 0.9f, 0.9f);
    object->m_material->m_diffuse.set(0.9f, 0.9f, 0.9f);
    object->m_material->m_specular.set(1.0f, 1.0f, 1.0f);

    // create a haptic effect for this object so that the operator can
    // feel its surface
    cEffectSurface* newEffect = new cEffectSurface(object);
    object->addEffect(newEffect);


    //--------------------------------------------------------------------------
    // WIDGETS
    //--------------------------------------------------------------------------

    // create a font
    font = NEW_CFONT_CALIBRI_20();

    // create a label to display the haptic and graphic rate of the simulation
    labelRates = new cLabel(font);
    camera->m_frontLayer->addChild(labelRates);

    // create a dial to display the magnitude of the haptic interaction forces
    level = new cLevel();
    camera->m_frontLayer->addChild(level);
    level->setLocalPos(100,100,0);
    level->setRange(0.0, 25.0);

    // create a dial to display the angular velocity of the object
    dial = new cDial();
    camera->m_frontLayer->addChild(dial);
    dial->setSingleIncrementDisplay(false);
    dial->setLocalPos(600,100,0);
    dial->setRange(0.0, 5.0);
    dial->setSize(60);
    dial->setValue0(0.0);


    //--------------------------------------------------------------------------
    // VIEWPORT DISPLAY
    //--------------------------------------------------------------------------

    // get content scale factor
    float contentScaleW, contentScaleH;
    glfwGetWindowContentScale(window, &contentScaleW, &contentScaleH);

    // create a viewport to display the scene.
    viewport = new cViewport(camera, contentScaleW, contentScaleH);


    //--------------------------------------------------------------------------
    // START HAPTIC SIMULATION THREAD
    //--------------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(renderHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // render graphics
        renderGraphics();

        // process events
        glfwPollEvents();
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return (0);
}

//------------------------------------------------------------------------------

void onWindowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    windowW = a_width;
    windowH = a_height;

    // render scene
    renderGraphics();
}

//------------------------------------------------------------------------------

void onFrameBufferSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update frame buffer size
    framebufferW = a_width;
    framebufferH = a_height;
}

//------------------------------------------------------------------------------

void onWindowContentScaleCallback(GLFWwindow* a_window, float a_xscale, float a_yscale)
{
    // update window content scale factor
    viewport->setContentScale(a_xscale, a_yscale);
}

//------------------------------------------------------------------------------

void onErrorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//------------------------------------------------------------------------------

void onKeyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - enable/disable texture rendering
    else if (a_key == GLFW_KEY_1)
    {
        bool useTexture = !object->getUseTexture();
        object->setUseTexture(useTexture);
        if (useTexture)
            cout << "> Enable texture     \r";
        else
            cout << "> Disable texture    \r";
    }

    // option - enable/disable wireframe mode
    else if (a_key == GLFW_KEY_2)
    {
        bool useWireMode = !object->getWireMode();
        object->setWireMode(useWireMode);
        if (useWireMode)
            cout << "> Wire mode enabled  \r";
        else
            cout << "> Wire mode disabled \r";
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
        }

        // set the desired swap interval and number of samples to use for multisampling
        glfwSwapInterval(swapInterval);
        glfwWindowHint(GLFW_SAMPLES, 4);
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void renderGraphics(void)
{
    // sanity check
    if (viewport == nullptr) { return; }

    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // get width and height of CHAI3D internal rendering buffer
    int displayW = viewport->getDisplayWidth();
    int displayH = viewport->getDisplayHeight();

    // update haptic and graphic rate data
    labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
                        cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    labelRates->setLocalPos((int)(0.5 * (displayW - labelRates->getWidth())), 15);

    // update value of level
    level->setValue(tool->getDeviceGlobalForce().length());

    // update position of level
    level->setLocalPos(50, (int)(0.5 * (displayH - level->getHeight())));

    // update value of dial
    dial->setValue1(angVel.length());

    // update position of dial
    dial->setLocalPos((int)(displayW - 80), (int)(0.5 * displayH));


    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render world
    viewport->renderView(framebufferW, framebufferH);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) cout << "Error: " << gluErrorString(error) << endl;

    // swap buffers
    glfwSwapBuffers(window);

    // signal frequency counter
    freqCounterGraphics.signal(1);
}

//------------------------------------------------------------------------------

void renderHaptics(void)
{
    // reset clock
    cPrecisionClock clock;
    clock.reset();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////
        // SIMULATION TIME
        /////////////////////////////////////////////////////////////////////

        // stop the simulation clock
        clock.stop();

        // read the time increment in seconds
        double timeInterval = clock.getCurrentTimeSeconds();

        // restart the simulation clock
        clock.reset();
        clock.start();

        // signal frequency counter
        freqCounterHaptics.signal(1);


        /////////////////////////////////////////////////////////////////////
        // HAPTIC FORCE COMPUTATION
        /////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////
        // DYNAMIC SIMULATION
        /////////////////////////////////////////////////////////////////////

        // some constants
        const double INERTIA = 0.4;
        const double MAX_ANG_VEL = 10.0;
        const double DAMPING = 0.1;

        // get position of cursor in global coordinates
        cVector3d toolPos = tool->getDeviceGlobalPos();

        // get position of object in global coordinates
        cVector3d objectPos = object->getGlobalPos();

        // compute a vector from the center of mass of the object (point of rotation) to the tool
        cVector3d v = cSub(toolPos, objectPos);

        // compute angular acceleration based on the interaction forces between the tool and the object
        cVector3d angAcc(0,0,0);
        if (v.length() > 0.0)
        {
            // get the last force applied to the cursor in global coordinates
            // we negate the result to obtain the opposite force that is applied on the
            // object
            cVector3d toolForce = cNegate(tool->getDeviceGlobalForce());

            // compute the effective force that contributes to rotating the object.
            cVector3d force = toolForce - cProject(toolForce, v);

            // compute the resulting torque
            cVector3d torque = cMul(v.length(), cCross( cNormalize(v), force));

            // update rotational acceleration
            angAcc = (1.0 / INERTIA) * torque;
        }

        // update rotational velocity
        angVel.add(timeInterval * angAcc);

        // set a threshold on the rotational velocity term
        double vel = angVel.length();
        if (vel > MAX_ANG_VEL)
        {
            angAcc.mul(MAX_ANG_VEL / vel);
        }

        // add some damping too
        angVel.mul(1.0 - DAMPING * timeInterval);

        // if user switch is pressed, set velocity to zero
        if (tool->getUserSwitch(0) == 1)
        {
            angVel.zero();
        }

        // compute the next rotation configuration of the object
        if (angVel.length() > C_SMALL)
        {
            object->rotateAboutGlobalAxisRad(cNormalize(angVel), timeInterval * angVel.length());
        }
    }
    
    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
