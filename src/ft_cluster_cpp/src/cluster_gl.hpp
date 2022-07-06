#ifndef CLUSTER_GL_HPP
#define CLUSTER_GL_HPP

#include <vector>
#include <glad/egl.h>

using namespace std;

class GLCluster {
private:
    // Don't go crazy with the batch size.
    // After a certain point things actually get slower.
    // Perhaps RAM is being used rather than VRAM.
    int batchSize = 0;
    int maximumPoints = 0;
    float eps = 0;
    
    unsigned int inputBuffer = 0;
    unsigned int outputBuffer = 0;
    unsigned int batchTargetsBuffer = 0;
    unsigned int sumBuffer = 0;
    unsigned int distancesBuffer = 0;
    unsigned int neighborBuffer = 0;
    unsigned int program = 0;
    unsigned int batchSizeLocation;
    unsigned int numberInBatchLocation;
    unsigned int epsSquaredLocation;
    unsigned int numberOfPointsLocation;
    std::vector<float> input;
    std::vector<float> distances;
    std::vector<int> sums;
    std::vector<int> neighbors;

    void generateBuffers();
    void loadShader();

    int numberOfInputPoints() const;
    void regenerateInputBuffer();
    void regenerateBatchSizeDependentBuffers();
    
public:
    GLCluster();
    // Takes XYZ_ data, or XYZ data if aligned is set to false
    void setInputData(float * data, int numPoints, bool aligned = true);
    int getBatchSize() const;
    void setBatchSize(int batchSize);
    // This function does nothing the the size doesn't change
    void setMaximumPoints(int maximumPoints);
    void setEps(float eps);
    template <typename MapType> void generateOutput(int targetIndex, MapType & neighbors);
    bool canAccomodatePoints(int points);
    void destroy();
    ~GLCluster();
};

// I copied the EGL context generation code from here:
// https://github.com/erwincoumans/egl_example
struct EGLInternalData2 {
    bool m_isInitialized;

    int m_windowWidth;
    int m_windowHeight;
    int m_renderDevice;

    EGLBoolean success;
    EGLint num_configs;
    EGLConfig egl_config;
    EGLSurface egl_surface;
    EGLContext egl_context;
    EGLDisplay egl_display;

    EGLInternalData2()
    : m_isInitialized(false),
    m_windowWidth(0),
    m_windowHeight(0) {}
};

void initializeEGL();

#endif // CLUSTER_GL_HPP
