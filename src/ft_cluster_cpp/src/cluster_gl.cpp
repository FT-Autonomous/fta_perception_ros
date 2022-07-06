#include <stdio.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>

#include <vector>
#include <fstream>
#include <iostream>
#include <cassert>

#include <glad/egl.h>
#include <glad/gl.h>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "cluster_gl.hpp"

const int LOCAL_SIZE_X = 1;

void shaderOkOrDie(unsigned int shader) {
    int shaderCompiled = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &shaderCompiled);
    if (not shaderCompiled) {
        char log[512];
        glGetShaderInfoLog(shader, 512, nullptr, log);
        assert(strlen(log));
        cout << log << endl;
    }
    assert(shaderCompiled);
}

int GLCluster::numberOfInputPoints() const {
    return this->input.size() / 4;
}

void GLCluster::regenerateBatchSizeDependentBuffers() {
    this->distances.resize(this->maximumPoints * this->batchSize);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->distancesBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, this->batchSize * this->maximumPoints * sizeof(float), nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, this->distancesBuffer);
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->batchTargetsBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 4 * this->batchSize * sizeof(float), nullptr, GL_DYNAMIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, this->batchTargetsBuffer);
    
    this->sums.resize(this->batchSize);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->sumBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, this->sumBuffer);

    this->neighbors.resize(this->maximumPoints * this->batchSize);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->neighborBuffer);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, this->neighborBuffer);
}

void GLCluster::loadShader() {
    unsigned int shader = glCreateShader(GL_COMPUTE_SHADER);
    
    auto source = ament_index_cpp::get_package_prefix("ft_cluster_cpp") + "/lib/ft_cluster_cpp/shaders/compute.glsl";
    ifstream shaderSource(source);
    string shaderText(istreambuf_iterator<char>{shaderSource},
                      istreambuf_iterator<char>{});
    
    shaderSource.close();
    auto shaderTextPtr = shaderText.c_str();
    glShaderSource(shader, 1, &shaderTextPtr, nullptr);
    glCompileShader(shader);
    shaderOkOrDie(shader);

    this->program = glCreateProgram();
    glAttachShader(program, shader);
    glLinkProgram(program);
    glUseProgram(program);

    this->batchSizeLocation = glGetUniformLocation(program, "batchSize");
    this->epsSquaredLocation = glGetUniformLocation(program, "epsSquared");
    this->numberInBatchLocation = glGetUniformLocation(program, "numberInBatch");
    this->numberOfPointsLocation = glGetUniformLocation(program, "numberOfPoints");
}

void GLCluster::generateBuffers() {
    glGenBuffers(1, &this->inputBuffer);
    glGenBuffers(1, &this->batchTargetsBuffer);
    glGenBuffers(1, &this->distancesBuffer);
    glGenBuffers(1, &this->sumBuffer);
    glGenBuffers(1, &this->neighborBuffer);
}

GLCluster::GLCluster() {
    loadShader();
    generateBuffers();
}

void GLCluster::setBatchSize(int batchSize)  {
    assert(batchSize > 0);
    this->batchSize = batchSize;
    regenerateBatchSizeDependentBuffers();
}

void GLCluster::setInputData(float *data, int numPoints, bool aligned) {
    assert(this->canAccomodatePoints(numPoints));

    glUniform1i(this->numberOfPointsLocation, numPoints);
    
    input.resize(numPoints * 4);

    if (aligned) {
        memcpy(input.data(), data, sizeof(float) * input.size());
    } else {
        for (int i = 0; i < numPoints; i++) {
            input[i*4] = data[i*3];
            input[i*4+1] = data[i*3+1];
            input[i*4+2] = data[i*3+2];
        }
    }
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->inputBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 4 * numPoints * sizeof(float), input.data(), GL_STATIC_COPY);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, this->inputBuffer);
    
    regenerateBatchSizeDependentBuffers();
}

int GLCluster::getBatchSize() const {
    return this->batchSize;
}

void GLCluster::setEps(float eps) {
    this->eps = eps;
    glUniform1f(this->epsSquaredLocation, eps * eps);
}

bool GLCluster::canAccomodatePoints(int points) {
    return this->maximumPoints >= points && points >= 0;
}

template <typename MapType> void GLCluster::generateOutput(int targetIndex, MapType & neighbors)
    {
        regenerateBatchSizeDependentBuffers();
        
    assert(this->batchSize && this->numberOfInputPoints() && this->eps);
    assert(targetIndex < this->numberOfInputPoints() && targetIndex >= 0);
    int numberInBatch = std::min(this->numberOfInputPoints() - targetIndex, batchSize);
    glUniform1i(this->numberInBatchLocation, numberInBatch);
        
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->batchTargetsBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, 4 * sizeof(float) * numberInBatch, this->input.data() + targetIndex * 4, GL_STATIC_COPY);

    std::fill(this->sums.begin(), this->sums.end(), 0);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->sumBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int) * batchSize, this->sums.data(), GL_STATIC_COPY);

    std::fill(this->neighbors.begin(), this->neighbors.end(), 0);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->neighborBuffer);
    glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(int) * batchSize * maximumPoints, this->neighbors.data(), GL_STATIC_COPY);
    
    // Don't do any extra space handling for now
    glDispatchCompute(this->maximumPoints / LOCAL_SIZE_X, numberInBatch, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->sumBuffer);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numberInBatch * sizeof(int), this->sums.data());

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->distancesBuffer);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, numberInBatch * this->maximumPoints * sizeof(float), this->distances.data());
    
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, this->neighborBuffer);
    glGetBufferSubData(GL_SHADER_STORAGE_BUFFER, 0, this->maximumPoints * numberInBatch * sizeof(int), this->neighbors.data());
    #define DISPLAY
    for (int i = 0; i < numberInBatch; i++) {
        #ifdef DISPLAY
        //cout << "Point " << targetIndex + i << " has " << sums[i] << " neighbors" << endl;
        //cout << " Distances" << endl;
        for (int j = 0; j < this->numberOfInputPoints(); j++) {
            //cout << " - " << this->distances[this->maximumPoints*i + j] << " from " << j << endl;
        }
        //cout << " Neighbors" << endl;
        #endif
        for (int j = 0; j < this->sums[i]; j++) {
            int neighbor = this->neighbors[this->maximumPoints*i + j];
            #ifdef DISPLAY
            //cout << " - " << neighbor << endl;
            #endif
            assert(neighbor < this->maximumPoints);
            neighbors[targetIndex+i].push_back(neighbor);
        }
    }
}


void GLCluster::destroy() {
    glDeleteProgram(this->program);
    glDeleteBuffers(1, &this->inputBuffer);
    glDeleteBuffers(1, &this->batchTargetsBuffer);
    glDeleteBuffers(1, &this->distancesBuffer);
    glDeleteBuffers(1, &this->sumBuffer);
    glDeleteBuffers(1, &this->neighborBuffer);
}

GLCluster::~GLCluster() { this->destroy(); }

void GLCluster::setMaximumPoints(int maximumPoints) {
    if (maximumPoints != this->maximumPoints) {
        this->maximumPoints = maximumPoints + (LOCAL_SIZE_X - maximumPoints % LOCAL_SIZE_X); // Pad
        glUniform1i(this->batchSizeLocation, this->maximumPoints);
    }
}

void testGLCluster() {
    cout << "hello" << endl;
    int g = glCreateShader(GL_COMPUTE_SHADER);
    cout << g << endl;
    
    const int maximumPoints = 65535;
    std::vector<float> data;
    for (int i = 0; i < maximumPoints; i++) {
        data.push_back(i);
        data.push_back(i);
        data.push_back(i);
        data.push_back(0.0);
    }

    GLCluster cluster;
    cluster.setEps(2.0);
    cluster.setBatchSize(100);
    cluster.setMaximumPoints(maximumPoints);

    std::vector<std::vector<int>> neighbors(maximumPoints);
  
    cluster.setInputData(data.data(), maximumPoints);
    for (int i = 0; i < maximumPoints; i+=cluster.getBatchSize()) {
        cluster.generateOutput(i, neighbors);
    }
}

void initializeEGL(){
    int m_windowWidth;
    int m_windowHeight;
    int m_renderDevice;
    
    EGLBoolean success;
    EGLint num_configs;
    EGLConfig egl_config;
    EGLSurface egl_surface;
    EGLContext egl_context;
    EGLDisplay egl_display;
    
    m_windowWidth = 256;
    m_windowHeight = 256;
    m_renderDevice = -1;

    EGLint egl_config_attribs[] = {
        EGL_RED_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE, 8,
        EGL_DEPTH_SIZE, 8,
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
        EGL_NONE
    };
    
    EGLint egl_pbuffer_attribs[] = {
        EGL_WIDTH, m_windowWidth, EGL_HEIGHT, m_windowHeight,
        EGL_NONE,
    };
    
    EGLInternalData2* m_data = new EGLInternalData2();

    // Load EGL functions
    int egl_version = gladLoaderLoadEGL(NULL);
    if(!egl_version) {
        fprintf(stderr, "failed to EGL with glad.\n");
        exit(EXIT_FAILURE);

    };

    // Query EGL Devices
    const int max_devices = 32;
    EGLDeviceEXT egl_devices[max_devices];
    EGLint num_devices = 0;
    EGLint egl_error = eglGetError();
    if (!eglQueryDevicesEXT(max_devices, egl_devices, &num_devices) ||
        egl_error != EGL_SUCCESS) {
        printf("eglQueryDevicesEXT Failed.\n");
        m_data->egl_display = EGL_NO_DISPLAY;
    }

    // Query EGL Screens
    if(m_data->m_renderDevice == -1) {
        // Chose default screen, by trying all
        for (EGLint i = 0; i < num_devices; ++i) {
            // Set display
            EGLDisplay display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
                                                          egl_devices[i], NULL);
            if (eglGetError() == EGL_SUCCESS && display != EGL_NO_DISPLAY) {
                int major, minor;
                EGLBoolean initialized = eglInitialize(display, &major, &minor);
                if (eglGetError() == EGL_SUCCESS && initialized == EGL_TRUE) {
                    m_data->egl_display = display;
                }
            }
        }
    } else {
        // Chose specific screen, by using m_renderDevice
        if (m_data->m_renderDevice < 0 || m_data->m_renderDevice >= num_devices) {
            m_data->m_renderDevice = 1;
            m_renderDevice = m_data->m_renderDevice;
            //fprintf(stderr, "Invalid render_device choice: %d < %d.\n", m_data->m_renderDevice, num_devices);
            //exit(EXIT_FAILURE);
        }

        // Set display
        EGLDisplay display = eglGetPlatformDisplayEXT(EGL_PLATFORM_DEVICE_EXT,
                                                      egl_devices[m_data->m_renderDevice], NULL);
        if (eglGetError() == EGL_SUCCESS && display != EGL_NO_DISPLAY) {
            int major, minor;
            EGLBoolean initialized = eglInitialize(display, &major, &minor);
            if (eglGetError() == EGL_SUCCESS && initialized == EGL_TRUE) {
                m_data->egl_display = display;
            }
        }
    }

    if (!eglInitialize(m_data->egl_display, NULL, NULL)) {
        fprintf(stderr, "Unable to initialize EGL\n");
        exit(EXIT_FAILURE);
    }

    egl_version = gladLoaderLoadEGL(m_data->egl_display);
    if (!egl_version) {
        fprintf(stderr, "Unable to reload EGL.\n");
        exit(EXIT_FAILURE);
    }
    printf("Loaded EGL %d.%d after reload.\n", GLAD_VERSION_MAJOR(egl_version),
           GLAD_VERSION_MINOR(egl_version));


    m_data->success = eglBindAPI(EGL_OPENGL_API);
    if (!m_data->success) {
        // TODO: Properly handle this error (requires change to default window
        // API to change return on all window types to bool).
        fprintf(stderr, "Failed to bind OpenGL API.\n");
        exit(EXIT_FAILURE);
    }

    m_data->success =
        eglChooseConfig(m_data->egl_display, egl_config_attribs,
                        &m_data->egl_config, 1, &m_data->num_configs);
    if (!m_data->success) {
        // TODO: Properly handle this error (requires change to default window
        // API to change return on all window types to bool).
        fprintf(stderr, "Failed to choose config (eglError: %d)\n", eglGetError());
        exit(EXIT_FAILURE);
    }
    if (m_data->num_configs != 1) {
        fprintf(stderr, "Didn't get exactly one config, but %d\n", m_data->num_configs);
        exit(EXIT_FAILURE);
    }

    m_data->egl_surface = eglCreatePbufferSurface(
                                                  m_data->egl_display, m_data->egl_config, egl_pbuffer_attribs);
    if (m_data->egl_surface == EGL_NO_SURFACE) {
        fprintf(stderr, "Unable to create EGL surface (eglError: %d)\n", eglGetError());
        exit(EXIT_FAILURE);
    }


    m_data->egl_context = eglCreateContext(
                                           m_data->egl_display, m_data->egl_config, EGL_NO_CONTEXT, NULL);
    if (!m_data->egl_context) {
        fprintf(stderr, "Unable to create EGL context (eglError: %d)\n",eglGetError());
        exit(EXIT_FAILURE);
    }

    m_data->success =
        eglMakeCurrent(m_data->egl_display, m_data->egl_surface, m_data->egl_surface,
                       m_data->egl_context);
    if (!m_data->success) {
        fprintf(stderr, "Failed to make context current (eglError: %d)\n", eglGetError());
        exit(EXIT_FAILURE);
    }

    if (!gladLoadGL(eglGetProcAddress)) {
        fprintf(stderr, "failed to load GL with glad.\n");
        exit(EXIT_FAILURE);
    }
}
