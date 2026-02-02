#ifndef UNIT_TWO_CIRCLE_H
#define UNIT_TWO_CIRCLE_H

#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

#include <iostream>
#include <cstdint>

#define WINDOWSIZE 400
#define NINE 9
#define SIXTEEN 16
#define WIDTH 700
#define HEIGHT WINDOWSIZE

SDL_Window* pWindow = nullptr;
SDL_Renderer* pRenderer = nullptr;
SDL_Texture* pTexture = nullptr;

uint32_t frameBuffer [HEIGHT][WIDTH];

SDL_Color red {255, 0, 0, 255};
SDL_Color grey {224, 216, 216, 255};

class Circle {
public:
    void Draw(int xc, int yc, int r, SDL_Color c, bool bFill = true) {
        int x = 0, y = r;
        int d = 3 - 2 * r;
        DrawPoints(xc, yc, x, y, m_white);
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
            DrawPoints(xc, yc, x, y, m_white);
        }

        FloodFill(xc, yc, c, m_black);
        UpdateTexture();
    }

private:
    void DrawPoints(int xc, int yc, int x, int y, SDL_Color c) {
        SetPixel(xc+x, yc+y, c);
        SetPixel(xc+x, yc+y, c);
        SetPixel(xc-x, yc+y, c);
        SetPixel(xc+x, yc-y, c);
        SetPixel(xc-x, yc-y, c);
        SetPixel(xc+y, yc+x, c);
        SetPixel(xc-y, yc+x, c);
        SetPixel(xc+y, yc-x, c);
        SetPixel(xc-y, yc-x, c);
    }

    bool IsColorTheSame(SDL_Color c1, SDL_Color c2) {
        return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b;
    }

    void FloodFill(int x, int y, SDL_Color fill, SDL_Color old) {
        SDL_Color currentColor = GetPixel(x, y);
 
        if (!IsColorTheSame(old, currentColor)) 
            return;
        
        SetPixel(x, y, fill);
        FloodFill(x + 1, y, fill, old);
        FloodFill(x - 1, y, fill, old);
        FloodFill(x, y + 1, fill, old);
        FloodFill(x, y - 1, fill, old);
    }

    SDL_Color GetPixel(int x, int y) {
        const uint32_t pixel = frameBuffer[y][x];

        return {
            static_cast<uint8_t>((pixel >> 16) & 0xFF),
            static_cast<uint8_t>((pixel >> 8) & 0xFF),
            static_cast<uint8_t>(pixel & 0xFF),
            static_cast<uint8_t>((pixel >> 24) & 0xFF)
        };
    }

    void SetPixel(int x, int y, SDL_Color color) {
        frameBuffer[y][x] = (color.a << 24) | (color.r << 16) | (color.g << 8) | color.b; 
    }

    void UpdateTexture() {
        SDL_UpdateTexture(pTexture, nullptr, frameBuffer, WIDTH * sizeof(uint32_t));
    }

    // white
    SDL_Color m_white{250, 245, 241, 255};
    // background: black
    SDL_Color m_black{0, 0, 0, 255};
};

#endif // !UNIT_TWO_CIRCLE_H