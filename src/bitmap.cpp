#include "bitmap.h"

#include <cassert>

#include <png.h>

Bitmap::Bitmap(uint32_t width, uint32_t height)
    : m_width{ width }
    , m_height{ height }
    , m_pixels{ std::make_unique<Pixel[]>(width * height)}
{
}

Bitmap::~Bitmap() = default;

Pixel& Bitmap::at(const uint32_t x, const uint32_t y)
{
    assert(m_pixels && x < m_width && y < m_height);
    return m_pixels[y * m_width + x];
}

const Pixel& Bitmap::at(const uint32_t x, const uint32_t y) const
{
    assert(m_pixels && x < m_width && y < m_height);
    return m_pixels[y * m_width + x];
}

uint8_t* Bitmap::rgb8_data()
{
    return &m_pixels[0].red;
}

const uint8_t* Bitmap::rgb8_data() const
{
    return &m_pixels[0].red;
}

bool Bitmap::toPngFile(const std::string & filePath) const
{
    FILE *fp;
    png_structp png_ptr = NULL;
    png_infop info_ptr = NULL;
    png_byte **row_pointers = NULL;
    bool success = false;
    /* The following number is set by trial and error only. I cannot
       see where it is documented in the libpng manual.
     */
    int pixel_size = 3;
    int depth = 8;

    fp = fopen(filePath.c_str(), "wb");
    if (!fp) {
        return false;
    }

    png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (png_ptr == NULL) {
        goto png_create_write_struct_failed;
    }

    info_ptr = png_create_info_struct(png_ptr);
    if (info_ptr == NULL) {
        goto png_create_info_struct_failed;
    }

    /* Set up error handling. */

    if (setjmp(png_jmpbuf(png_ptr))) {
        goto png_failure;
    }

    /* Set image attributes. */

    png_set_IHDR(png_ptr,
        info_ptr,
        m_width,
        m_height,
        depth,
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

      /* Initialize rows of PNG. */

    row_pointers = static_cast<png_byte**>(png_malloc(png_ptr, m_height * sizeof(png_byte *)));
    for (uint32_t y = 0; y < m_height; ++y) {
        png_byte *row = static_cast<png_byte*>(png_malloc(png_ptr,
            sizeof(png_byte) * m_width *
            pixel_size));
        row_pointers[y] = row;
        for (uint32_t x = 0; x < m_width; ++x) {
            const Pixel &pixel = at(x, y);
            *row++ = pixel.red;
            *row++ = pixel.green;
            *row++ = pixel.blue;
        }
    }

    /* Write the image data to "fp". */

    png_init_io(png_ptr, fp);
    png_set_rows(png_ptr, info_ptr, row_pointers);
    png_write_png(png_ptr, info_ptr, PNG_TRANSFORM_IDENTITY, NULL);

    /* The routine has successfully written the file */
    success = true;

    for (size_t y = 0; y < m_height; y++) {
        png_free(png_ptr, row_pointers[y]);
    }
    png_free(png_ptr, row_pointers);

png_failure:
png_create_info_struct_failed:
    png_destroy_write_struct(&png_ptr, &info_ptr);
png_create_write_struct_failed:
    fclose(fp);
    return success;
}
