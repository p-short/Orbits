#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <RtAudio.h>

#include <sculpt.h>

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

float CalculateDistance(float x0, float y0, float x1, float y1) {
    const float dx = x1 - x0;
    const float dy = y1 - y0;

    return sqrt(dx * dx + dy * dy);
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

class OneShot {
public:
    bool Process(bool signal) {
        bool fire = signal && !previous;
        previous = signal;
        return fire;
    }
private:
    bool previous = false;
};

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

    bool IsIntersected(const Point& rotatingArm, float rotationSpeed) {

        Point nodePoint { static_cast<float>(WIDTH / 2) + GetXPos() * GLOB_CIRCLE_RAD_F,
                          static_cast<float>(HEIGHT / 2) + GetYPos() * GLOB_CIRCLE_RAD_F };

        // vector between node and center of main circle
        Point vectorA = CreateVector(nodePoint, centerPoint);

        // vector between end point of rotating arm and center of main circle.
        Point vectorB = CreateVector(rotatingArm, centerPoint);

        const float scalarProjection = CalculateDotProduct(vectorA, NormaliseVector(vectorB));
        const float spx = static_cast<float>(WIDTH / 2) + scalarProjection * cosf(rotationSpeed);
        const float spy = static_cast<float>(HEIGHT / 2) + scalarProjection * sinf(rotationSpeed);

        //SDL_RenderLine(pRenderer, nodePoint.x, nodePoint.y, spx, spy);
        // std::cout << "nodePoint.x: " << nodePoint.x << " nodePoint.y: " << nodePoint.y << "\n";
        // std::cout << "spx: " << spx << " spy: " << spy << "\n";

        // distance between end of rotating arm and node circle.
        float dist1 = CalculateDistance(rotatingArm.x, rotatingArm.y, nodePoint.x, nodePoint.y);
        float dist2 = CalculateDistance(nodePoint.x, nodePoint.y, spx, spy);
        float dist3 = CalculateDistance(nodePoint.x, nodePoint.y, centerPoint.x, centerPoint.y);

        //std::cout << "dist1: " << dist1 << "\n";
        //std::cout << "dist2: " << dist2 << "\n";

        if (dist1 < GLOB_CIRCLE_RAD_F && dist2 < 15 /* node circle radius */ && dist3 > 15) {
            //std::cout << "Intersecting!!!\n";
            return true;
        } else {
            //std::cout << "NOT intersecting\n";
            return false;
        }
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
    Point centerPoint { CENTER_X_F, CENTER_Y_F };
};

struct NodeAudioState {
    float x;
    float y;
};

const size_t totalNumberOfNodes = 14;
std::vector<OneShot> triggers;
std::vector<Node> nodes;
std::vector<NodeAudioState> nodeAudioStates;

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

struct QuadGains { float tl, tr, br, bl; };

QuadGains ComputeTargetGains(float dx, float dy) {
    float len = std::sqrt(dx * dx + dy * dy);
    if (len > 0.f) {
        dx /= len;
        dy /= len;
    }

    const float inv = 0.70710678f;

    float tl = std::max(0.0f, (-dx - dy) * inv);
    float tr = std::max(0.0f, ( dx - dy) * inv);
    float br = std::max(0.0f, ( dx + dy) * inv);
    float bl = std::max(0.0f, (-dx + dy) * inv);

    tl = std::sqrt(tl);
    tr = std::sqrt(tr);
    br = std::sqrt(br);
    bl = std::sqrt(bl);

    float sum = tl + tr + br + bl;
    if (sum > 0.0f) {
        tl /= sum;
        tr /= sum;
        br /= sum;
        bl /= sum;
    }

    return {tr, tl, br, bl};
}

struct Voice {
    bool isActive;
    uint32_t nodeIndex;
    uint32_t duration;
    Sculpt::Oscillator::Sawwave osc;
    Sculpt::Envelope::ADR env;
    QuadGains prevGain;
};

// TODO: fix early env ending bug.
// first envelope sounds for the entire envelope
// subsequent envelopes get to 0.5 then straight to 0 (missing the release phase)
// thus killing the voice in PolySynth.Process method.
class PolySynth {
public:
    PolySynth() {
        for (auto& voice : voices) {
            voice.isActive = false;
            voice.nodeIndex = 0;
            voice.duration = 0;
            voice.env.SetParameters(0.1, 0.5, 0.2, 5);
            voice.prevGain.tl = 0.0f;
            voice.prevGain.tr = 0.0f;
            voice.prevGain.br = 0.0f;
            voice.prevGain.bl = 0.0f;
        }
    }

    void NoteOn(uint32_t nodeIndex) {
        //std::cout << "notes: " << noteOffset + midiNotes[nodeIndex] << "\n";
        for (auto& voice : voices) {
            // TODO: theres zero checks if voice is active or not.
            // think its cutting first note short over and over.
            if (!voice.isActive) {
                //std::cout << "yep\n";
                voice.isActive = true;
                voice.nodeIndex = nodeIndex;


                float x = WIDTH_F / 2.f + nodes[voice.nodeIndex].GetXPos() * GLOB_CIRCLE_RAD_F;
                float y = HEIGHT_F / 2.f + nodes[voice.nodeIndex].GetYPos() * GLOB_CIRCLE_RAD_F;

                float cx = CENTER_X_F;
                float cy = CENTER_Y_F;

                float dx = x - cx;
                float dy = y - cy;

                voice.prevGain = ComputeTargetGains(dx, dy);

                voice.duration = 0;
                voice.osc.SetFrequency(Sculpt::Utility::MidiNoteToHz(noteOffset + midiNotes[nodeIndex]));
                //std::cout << "notes: " << noteOffset + midiNotes[nodeIndex] << "\n";
                voice.env.NoteOn();
                return;
            }
        }

        std::cout << "Voices array is full and active.\n";

        // if code gets this far its because all voices are active
        // find the oldest voice (one with largest duration) and steal that
        uint32_t largestDuration = 0;
        uint32_t index = 0;
        for (int i = 0; i < voices.size(); ++i) {
            if (voices[i].duration > largestDuration) {
                largestDuration = voices[i].duration;
                index = i;
            }
        }

        // TODO: envelope class needs a Reset method.
        // steal voice
        voices[index].isActive = true;
        voices[index].nodeIndex = nodeIndex;
        voices[index].duration = 0;
        voices[index].osc.SetFrequency(Sculpt::Utility::MidiNoteToHz(noteOffset + midiNotes[nodeIndex]));
        voices[index].env.NoteOn();
    }

    double Process() {
        double mix = 0.0;
        double currentEnvSample = 0.0;

        for (auto& voice : voices) {
            if (voice.isActive) {
                voice.duration++;

                currentEnvSample = voice.env.Process();
                mix += voice.osc.Process() * currentEnvSample;

                if (currentEnvSample <= 0.0) {
                    voice.isActive = false;
                    voice.duration = 0;
                }
            }
        }
        return mix * 0.2; // small amount of gain to avoid clipping
    }

    std::array<Voice, 30> voices;

private:
    std::array<int, 15> midiNotes = { 0, 2, 3, 5, 7, 8, 10, 12, 14, 15, 17, 19, 20, 22, 24};
    const uint32_t noteOffset = 60;
};

PolySynth polySynth;

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
  //int midiNote = 60 + midiNotes[noteIndex];
  //saw.SetFrequency(Sculpt::Utility::MidiNoteToHz(midiNote));
  double s = 0.0;
  double currentEnvSample = 0.0;

  // Interleaved audio: frame0[ch0 ch1 ch2 ch3] frame1[ch0 ch1 ch2 ch3] ...
  for ( unsigned int i = 0; i < nBufferFrames; i++ ) {
    double topLeft = 0.0;
    double topRight = 0.0;
    double bottomRight = 0.0;
    double bottomLeft = 0.0;


    //double sample = polySynth.Process();
    //double gain = 0.2

    for (auto& voice : polySynth.voices) {
        if (voice.isActive) {
            voice.duration++;

            currentEnvSample = voice.env.Process();
            s = voice.osc.Process() * currentEnvSample;

            if (currentEnvSample <= 0.0) {
                voice.isActive = false;
                voice.duration = 0;
            }

            float x = WIDTH_F / 2.f + nodeAudioStates[voice.nodeIndex].x * GLOB_CIRCLE_RAD_F;
            float y = HEIGHT_F / 2.f + nodeAudioStates[voice.nodeIndex].y * GLOB_CIRCLE_RAD_F;

            float cx = CENTER_X_F;
            float cy = CENTER_Y_F;

            float dx = x - cx;
            float dy = y - cy;

            QuadGains target = ComputeTargetGains(dx, dy);

            const float smoothTimeInMS = 10.f;
            const float smooth = 1.0f - std::exp(-1.0 / (smoothTimeInMS * 0.001f * 44100.0));
            voice.prevGain.tl += smooth * (target.tl - voice.prevGain.tl);
            voice.prevGain.tr += smooth * (target.tr - voice.prevGain.tr);
            voice.prevGain.br += smooth * (target.br - voice.prevGain.br);
            voice.prevGain.bl += smooth * (target.bl - voice.prevGain.bl);

            topLeft += s * voice.prevGain.tl;
            topRight += s * voice.prevGain.tr;
            bottomRight += s * voice.prevGain.br;
            bottomLeft += s * voice.prevGain.bl;
        }
    }
    //s * 0.2; // small amount of gain to avoid clipping
    

    buffer[0] = bottomRight * 0.2;
    buffer[1] = bottomLeft * 0.2;
    buffer[2] = topLeft * 0.2;
    buffer[3] = topRight * 0.2;

    buffer += 4;
    //for ( unsigned int ch = 0; ch < numChannels; ch++ ) {
      ///*buffer++ = lastValues[ch];

     // Different slope per channel so they are audible separately
      


      //lastValues[ch] = mix * gain;

      //lastValues[ch] += 0.005 * (ch + 1);
      //if ( lastValues[ch] >= 1.0 )
       //lastValues[ch] -= 2.0;
    //}
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
    //if ( audio.startStream() ) {
    //    std::cout << audio.getErrorText() << std::endl;
    //    goto cleanup;
    //}

    audio.startStream();
    
    //char input;
    //std::cout << "\nPlaying ... press <enter> to quit.\n";
    //std::cin.get( input );
    
    //cleanup:
    //if ( audio.isStreamOpen() ) audio.closeStream();

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
        triggers.push_back(OneShot());
        nodeAudioStates.push_back(NodeAudioState());
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

            if (event.type == SDL_EVENT_QUIT) {
                // Block released ... stop the stream
                if ( audio.isStreamRunning() )
                    audio.stopStream();  // or could call dac.abortStream();

                if ( audio.isStreamOpen() ) audio.closeStream();

                done = true;
            }
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

        Point RotatingArmPoint { static_cast<float>(WIDTH / 2) + cosf(t) * GLOB_CIRCLE_RAD_F,
                                 static_cast<float>(HEIGHT / 2) + sinf(t) * GLOB_CIRCLE_RAD_F };

        // Update node 0 from sliders ONCE
        nodes[0].SetXPos(x_pos);
        nodes[0].SetYPos(y_pos);
        nodes[0].SetCXPos(cx_pos);
        nodes[0].SetCYPos(cy_pos);
        nodes[0].Constrain();

        for (size_t i = 0; i < totalNumberOfNodes; ++i) {

            // update the NEXT node after drawing
            if (i + 1 < totalNumberOfNodes) {
                nodes[i].UpdatePosition(nodes[i + 1]);
            }

            nodeAudioStates[i].x = nodes[i].GetXPos();
            nodeAudioStates[i].y = nodes[i].GetYPos();

            if (triggers[i].Process(nodes[i].IsIntersected(RotatingArmPoint, t)))
                polySynth.NoteOn(nodes[i].GetIndex());

            if (nodes[i].IsIntersected(RotatingArmPoint, t)) {


                SDL_SetRenderDrawColor(pRenderer, 255, 0, 0, SDL_ALPHA_OPAQUE);  // red, full alpha
            } else {
                SDL_SetRenderDrawColor(pRenderer, 255, 255, 255, SDL_ALPHA_OPAQUE);  // white, full alpha 
            } 

            // draw current node
            drawCircle(
                pRenderer,
                WIDTH / 2 + nodes[i].GetXPos() * GLOB_CIRCLE_RAD,
                HEIGHT / 2 + nodes[i].GetYPos() * GLOB_CIRCLE_RAD,
                SMALL_CIRCLE_RAD
            );

 
        }

        //for (size_t i = 0; i < totalNumberOfNodes; ++i) {
            // TODO: need to limit position to inside the bounds of main circle
        //    nodes[i].Constrain();


        //}

        // Point centerPoint { CENTER_X_F, CENTER_Y_F };

        // Point RotatingArmPoint { static_cast<float>(WIDTH / 2) + cosf(t) * GLOB_CIRCLE_RAD_F,
        //                          static_cast<float>(HEIGHT / 2) + sinf(t) * GLOB_CIRCLE_RAD_F };

        // Point nodePoint { static_cast<float>(WIDTH / 2) + nodes[0].GetXPos() * GLOB_CIRCLE_RAD_F,
        //                   static_cast<float>(HEIGHT / 2) + nodes[0].GetYPos() * GLOB_CIRCLE_RAD_F };

        // // vector between node and center of main circle
        // Point vectorA = CreateVector(nodePoint, centerPoint);

        // // vector between end point of rotating arm and center of main circle.
        // Point vectorB = CreateVector(RotatingArmPoint, centerPoint);

        // const float scalarProjection = CalculateDotProduct(vectorA, NormaliseVector(vectorB));
        // const float spx = static_cast<float>(WIDTH / 2) + scalarProjection * cosf(t);
        // const float spy = static_cast<float>(HEIGHT / 2) + scalarProjection * sinf(t);

        // SDL_RenderLine(pRenderer, nodePoint.x, nodePoint.y, spx, spy);
        // // std::cout << "nodePoint.x: " << nodePoint.x << " nodePoint.y: " << nodePoint.y << "\n";
        // // std::cout << "spx: " << spx << " spy: " << spy << "\n";

        // // distance between end of rotating arm and node circle.
        // float dist1 = CalculateDistance(RotatingArmPoint.x, RotatingArmPoint.y, nodePoint.x, nodePoint.y);
        // float dist2 = CalculateDistance(nodePoint.x, nodePoint.y, spx, spy);

        // if (dist1 < GLOB_CIRCLE_RAD_F && dist2 < 15 /* node circle radius */)
        //     std::cout << "Intersecting!!!\n";
        // else
        //     std::cout << "NOT intersecting\n";

        SDL_RenderPresent(pRenderer);  /* put it all on the screen! */
        t += 0.001;
    }

    SDL_DestroyRenderer(pRenderer);
    SDL_DestroyWindow(pWindow);
    pWindow = NULL;

    SDL_Quit();

    return EXIT_SUCCESS;
}