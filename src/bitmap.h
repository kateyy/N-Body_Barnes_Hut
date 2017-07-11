#pragma once

#include <cstdint>
#include <memory>
#include <string>

/* A coloured pixel. */
struct Pixel
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

/* A picture. */
class Bitmap
{
public:
    Bitmap(uint32_t width, uint32_t height);
    ~Bitmap();
    uint32_t width() const {
        return m_width;
    }
    uint32_t height() const {
        return m_height;
    }
    Pixel & at(uint32_t x, uint32_t y);
    const Pixel& at(uint32_t x, uint32_t y) const;
    uint8_t* rgb8_data();
    const uint8_t* rgb8_data() const;

    bool toPngFile(const std::string &filePath) const;
private:
    uint32_t m_width;
    uint32_t m_height;
    std::unique_ptr<Pixel[]> m_pixels;
};
