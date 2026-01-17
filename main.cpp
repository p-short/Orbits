#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <RtAudio.h>

#include <stdio.h>
#include <stdlib.h>
#include <cstdint>
#include <cmath>

#include <vector>
#include <iostream>

#define WIDTH 640
#define HEIGHT 480
#define CENTER_X WIDTH / 2
#define CENTER_Y HEIGHT / 2
#define GLOB_CIRCLE_RAD 220
#define SMALL_CIRCLE_RAD 15

const float WIDTH_F = static_cast<float>(WIDTH);
const float HEIGHT_F = static_cast<float>(HEIGHT);
const float CENTER_X_F = static_cast<float>(CENTER_X);
const float CENTER_Y_F = static_cast<float>(CENTER_Y);
const float GLOB_CIRCLE_RAD_F = static_cast<float>(GLOB_CIRCLE_RAD);
const float SMALL_CIRCLE_RAD_F = static_cast<float>(SMALL_CIRCLE_RAD);

struct Point {
    float x;
    float y;
};

Point CreateVector(Point p0, Point p1) {
    Point point;
    point.x = p1.x - p0.x;
    point.y = p1.y - p0.y;

    return point;
}

float CalculateDotProduct(Point v0, Point v1) {
    return v0.x * v1.x + v0.y * v1.y;
}

float CalculateMagnitude(Point v) {
    return sqrt(v.x * v.x + v.y *v.y);
}

Point NormaliseVector(Point v) {
    float m = CalculateMagnitude(v);

    Point point;
    point.x = v.x / m;
    point.y = v.y / m;

    return point;
}

void drawCirclePoints(SDL_Renderer* renderer, int xc, int yc, int x, int y) {
    SDL_RenderPoint(renderer, xc+x, yc+y);
    SDL_RenderPoint(renderer, xc+x, yc+y);
    SDL_RenderPoint(renderer, xc-x, yc+y);
    SDL_RenderPoint(renderer, xc+x, yc-y);
    SDL_RenderPoint(renderer, xc-x, yc-y);
    SDL_RenderPoint(renderer, xc+y, yc+x);
    SDL_RenderPoint(renderer, xc-y, yc+x);
    SDL_RenderPoint(renderer, xc+y, yc-x);
    SDL_RenderPoint(renderer, xc-y, yc-x);
}

void drawCircle(SDL_Renderer* renderer, int xc, int yc, int r) {
    int x = 0, y = r;
    int d = 3 - 2 * r;
    drawCirclePoints(renderer, xc, yc, x, y);
    while (y >= x){
      
        // check for decision parameter
        // and correspondingly 
        // update d, y
        if (d > 0) {
            y--; 
            d = d + 4 * (x - y) + 10;
        }
        else
            d = d + 4 * x + 6;

        // Increment x after updating decision parameter
        x++;
        
        // Draw the circle using the new coordinates
        drawCirclePoints(renderer, xc, yc, x, y);
    }
}

void ClampValue(float& value, float min, float max); // forward dec

class Node {
public:
    Node(uint32_t index) {
        m_index = index;
    }

    float GetXPos() const {
        return m_main_coords.x;
    }

    void SetXPos(float x) {
        m_main_coords.x = x;
    }

    float GetYPos() const {
        return m_main_coords.y;
    }

    void SetYPos(float y) {
        m_main_coords.y = y;
    }

    float GetCXPos() const {
        return m_offset_coords.x;
    }

    void SetCXPos(float cx) {
        m_offset_coords.x = cx;
    }

    float GetCYPos() const {
        return m_offset_coords.y;
    }

    void SetCYPos(float cy) {
        m_offset_coords.y = cy;
    }

    Point GetPoint() const {
        return m_main_coords;
    }

    float GetVelocity() const {
        return m_velocity;
    }

    uint32_t GetIndex() const {
        return m_index;
    }

    bool IsOn() const {
        return m_isOn;
    }

    void UpdatePosition(Node& other) {
        float x = (pow(GetXPos(), 2) + -pow(GetYPos(), 2)) +
                  (pow(GetCXPos(), 2) + -pow(GetCYPos(), 2));
        other.SetXPos(x);

        float y = 2 * GetXPos() * GetYPos() * 2 * GetCXPos() * GetCYPos();
        other.SetYPos(y);

        other.SetCXPos(GetCXPos());
        other.SetCYPos(GetCYPos());

        other.Constrain();
    }

    void Constrain() {
        /*
        Point center;
        center.x = static_cast<float>(WIDTH / 2);
        center.y =  static_cast<float>(HEIGHT / 2);
        Point nodePos = GetPoint();
        Point vec = CreateVector(center, nodePos);
        
        float phi = atan2(nodePos.y, nodePos.x);
        float mag = CalculateMagnitude(vec); 
        ClampValue(mag, 0.f, 1.f);

        if (std::isnan(mag) || std::isinf(mag))
            mag = 1.f;

        SetXPos(mag * cos(phi));
        SetYPos(mag * sin(phi));
        */

        float mag = std::sqrt(
            m_main_coords.x * m_main_coords.x +
            m_main_coords.y * m_main_coords.y
        );

        if (mag > maxNorm && mag > 0.0f) {
            m_main_coords.x /= mag;
            m_main_coords.y /= mag;

            m_main_coords.x *= maxNorm;
            m_main_coords.y *= maxNorm;
        }
    }
    
private:
    Point m_main_coords;
    Point m_offset_coords;
    float m_velocity { 0.f };
    uint32_t m_index { 0 }; 
    bool m_isOn { false };
    const float maxNorm = (GLOB_CIRCLE_RAD_F - SMALL_CIRCLE_RAD_F) / GLOB_CIRCLE_RAD_F;
};

const size_t totalNumberOfNodes = 24;
std::vector<Node> nodes;

float x_pos = 0.f;
float y_pos = 0.f;
float cx_pos = 0.f;
float cy_pos = 0.f;
const float inc = 0.01;

void ClampValue(float& value, float min, float max) {
    if (value < min)
        value = min;
    else if (value > max)
        value = max;
}

float t = 0.f;

// Four-channel sawtooth wave generator.
int myCallback( void *outputBuffer, void *inputBuffer,
                unsigned int nBufferFrames,
                double streamTime,
                RtAudioStreamStatus status,
                void *userData )
{
  double *buffer = static_cast<double*>(outputBuffer);
  double *lastValues = static_cast<double*>(userData);

  if ( status )
    std::cout << "Stream underflow detected!" << std::endl;

  const unsigned int numChannels = 4;

  // Interleaved audio: frame0[ch0 ch1 ch2 ch3] frame1[ch0 ch1 ch2 ch3] ...
  for ( unsigned int i = 0; i < nBufferFrames; i++ ) {
    for ( unsigned int ch = 0; ch < numChannels; ch++ ) {
      *buffer++ = lastValues[ch];

      // Different slope per channel so they are audible separately
      lastValues[ch] += 0.005 * (ch + 1);
      if ( lastValues[ch] >= 1.0 )
        lastValues[ch] -= 2.0;
    }
  }

  return 0;
}

int main()
{
    RtAudio audio;
 
    // Get the list of device IDs
    std::vector< unsigned int > ids = audio.getDeviceIds();
    if ( ids.size() == 0 ) {
        std::cout << "No devices found." << std::endl;
        return 0;
    }
    
    // Scan through devices for various capabilities
    RtAudio::DeviceInfo info;
    for ( unsigned int n=0; n<ids.size(); n++ ) {
    
        info = audio.getDeviceInfo( ids[n] );
    
        // Print, for example, the name and maximum number of output channels for each device
        std::cout << "device ID: " << info.ID << std::endl;
        std::cout << "device name = " << info.name << std::endl;
        std::cout << ": maximum output channels = " << info.outputChannels << std::endl;

        std::cout << "Avaliable sample rates...\n";
        for (const auto& sr : info.sampleRates)
            std::cout << "\t" << sr << "\n";

        std::cout << "preferd sample rate: " << info.preferredSampleRate << std::endl;
        std::cout << std::endl;
    }

    RtAudio::StreamParameters parameters;
    parameters.deviceId = 131;
    parameters.nChannels = 4;
    parameters.firstChannel = 0;
    unsigned int sampleRate = 44100;
    unsigned int bufferFrames = 256;
    double data[4] = {0.0, 0.0, 0.0, 0.0};

    if (audio.openStream(&parameters, NULL, RTAUDIO_FLOAT64, sampleRate, &bufferFrames, &myCallback, (void *)&data)) {
        std::cout << audio.getErrorText() << "\n";
        exit(0);
    }

      // Stream is open ... now start it.
    if ( audio.startStream() ) {
        std::cout << audio.getErrorText() << std::endl;
        goto cleanup;
    }
    
    char input;
    std::cout << "\nPlaying ... press <enter> to quit.\n";
    std::cin.get( input );
    
    // Block released ... stop the stream
    if ( audio.isStreamRunning() )
        audio.stopStream();  // or could call dac.abortStream();
    
    cleanup:
    if ( audio.isStreamOpen() ) audio.closeStream();

    printf("Hello, World!\n");

    if (!SDL_Init(SDL_INIT_VIDEO))
    {
        printf("SDL_Init Error: %s\n", SDL_GetError());
        return EXIT_FAILURE;
    }

    SDL_Window* pWindow = SDL_CreateWindow("sdl3-test", WIDTH, HEIGHT, SDL_WINDOW_OPENGL);
    if (!pWindow)
    {
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, "SDL_CreateWindow failed: %s", SDL_GetError());
        return EXIT_FAILURE;
    }
    SDL_Renderer* pRenderer = SDL_CreateRenderer(pWindow, NULL);
    if (!pRenderer)
    {
        SDL_LogError(SDL_LOG_CATEGORY_ERROR, "SDL_CreateRenderer failed: %s", SDL_GetError());
        return EXIT_FAILURE;
    }

    // create nodes
    for (size_t i = 0; i < totalNumberOfNodes; ++i) {
        nodes.push_back(Node(i));
    }

    bool done = false;
    while (!done)
    {
        SDL_Event event;

        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_EVENT_KEY_DOWN) {
                // std::cout << "key: " <<  event.key.key << " has been pressed\n";

                if (event.key.key == 113 /* q */) {
                    x_pos += inc;
                    ClampValue(x_pos, -1, 1);
                }

                if (event.key.key == 97 /* a */) {
                    x_pos -= inc;
                    ClampValue(x_pos, -1, 1);
                }

                if (event.key.key == 119 /* w */) {
                    y_pos += inc;
                    ClampValue(y_pos, -1, 1);
                }

                if (event.key.key == 115 /* s */) {
                    y_pos -= inc;
                    ClampValue(y_pos, -1, 1);
                }


                if (event.key.key == 101 /* e */) {
                    cx_pos += inc;
                    ClampValue(cx_pos, -1, 1);
                }

                if (event.key.key == 100 /* d */) {
                    cx_pos -= inc;
                    ClampValue(cx_pos, -1, 1);
                }

                if (event.key.key == 114 /* r */) {
                    cy_pos += inc;
                    ClampValue(cy_pos, -1, 1);
                }

                if (event.key.key == 102 /* f */) {
                    cy_pos -= inc;
                    ClampValue(cy_pos, -1, 1);
                }
            }

            if (event.type == SDL_EVENT_QUIT)
                done = true;
        }

        SDL_SetRenderDrawColor(pRenderer, 0, 0, 0, SDL_ALPHA_OPAQUE);  /* black, full alpha */
        SDL_RenderClear(pRenderer);  /* start with a blank canvas. */
        SDL_SetRenderDrawColor(pRenderer, 255, 255, 255, SDL_ALPHA_OPAQUE);  /* white, full alpha */

        // draw main circle that houses nodes
        drawCircle(pRenderer, CENTER_X, CENTER_Y, GLOB_CIRCLE_RAD);

        SDL_RenderLine(pRenderer, static_cast<float>(WIDTH / 2),
                       static_cast<float>(HEIGHT / 2),
                       static_cast<float>(WIDTH / 2) + cosf(t) * GLOB_CIRCLE_RAD_F,
                       static_cast<float>(HEIGHT / 2) + sinf(t) * GLOB_CIRCLE_RAD_F);

        // Update node 0 from sliders ONCE
        nodes[0].SetXPos(x_pos);
        nodes[0].SetYPos(y_pos);
        nodes[0].SetCXPos(cx_pos);
        nodes[0].SetCYPos(cy_pos);
        nodes[0].Constrain();

        for (size_t i = 0; i < totalNumberOfNodes; ++i) {

            // draw current node
            drawCircle(
                pRenderer,
                WIDTH / 2 + nodes[i].GetXPos() * GLOB_CIRCLE_RAD,
                HEIGHT / 2 + nodes[i].GetYPos() * GLOB_CIRCLE_RAD,
                SMALL_CIRCLE_RAD
            );

            // update the NEXT node after drawing
            if (i + 1 < totalNumberOfNodes) {
                nodes[i].UpdatePosition(nodes[i + 1]);
            }
        }

        //for (size_t i = 0; i < totalNumberOfNodes; ++i) {
            // TODO: need to limit position to inside the bounds of main circle
        //    nodes[i].Constrain();


        //}

        SDL_RenderPresent(pRenderer);  /* put it all on the screen! */
        t += 0.001;
    }

    SDL_DestroyRenderer(pRenderer);
    SDL_DestroyWindow(pWindow);
    pWindow = NULL;

    SDL_Quit();

    return EXIT_SUCCESS;
}