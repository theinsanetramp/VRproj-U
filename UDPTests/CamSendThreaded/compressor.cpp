#include "compressor.h"

Compressor::Compressor()
{
	handle = liq_attr_create();
	liq_set_dithering_level(quantization_result, 1.0);
	liq_set_speed(handle, 6);

	lowThreshold = 30;
    thresh_ratio = 3.5;
    kernel_size = 3;
    dilation_type = MORPH_RECT;
    dilation_size = 1;
    element = getStructuringElement( dilation_type,
                                   Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                   Point( dilation_size, dilation_size ) );
    blur_kernel = 5;
}

void Compressor::CannyThreshold(Mat image)
{
    // Reduce noise with kernel defined by blur_kernel
    blur( image, image, Size(blur_kernel,blur_kernel) );

    /// Canny detector
    Canny( image, image, lowThreshold, lowThreshold*thresh_ratio, kernel_size );

    /// Apply the dilation operation
    dilate( image, image, element );
}

vector<unsigned char> Compressor::CompressImage(Mat image)
{
	raw_rgba_pixels = new uchar[image.total()*4];
	continuousRGBA = Mat(image.size(), CV_8UC4, raw_rgba_pixels);
	cvtColor(image, continuousRGBA, CV_GRAY2RGBA, 4);

	input_image = liq_image_create_rgba(handle, raw_rgba_pixels, image.cols, image.rows, 0);
	// You could set more options here, like liq_set_quality
	if (liq_image_quantize(input_image, handle, &quantization_result) != LIQ_OK) {
	  fprintf(stderr, "Quantization failed\n");
	}

	pixels_size = image.cols * image.rows;
	raw_8bit_pixels = (unsigned char*)malloc(pixels_size);

	liq_write_remapped_image(quantization_result, input_image, raw_8bit_pixels, pixels_size);
	palette = liq_get_palette(quantization_result);

	lodepng_state_init(&state);
	state.info_raw.colortype = LCT_PALETTE;
	state.info_raw.bitdepth = 8;
	state.info_png.color.colortype = LCT_PALETTE;
	state.info_png.color.bitdepth = 8;

	for(int i=0; i < palette->count; i++) {
	 lodepng_palette_add(&state.info_png.color, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
	 lodepng_palette_add(&state.info_raw, palette->entries[i].r, palette->entries[i].g, palette->entries[i].b, palette->entries[i].a);
	}

	unsigned int out_status = lodepng_encode(&output_file_data, &output_file_size, raw_8bit_pixels, image.cols, image.rows, &state);
	if (out_status) {
	  fprintf(stderr, "Can't encode image: %s\n", lodepng_error_text(out_status));
	}

	liq_image_destroy(input_image);
	delete raw_rgba_pixels;
	free(raw_8bit_pixels);
	lodepng_state_cleanup(&state);

  	return vector<unsigned char>(output_file_data, output_file_data + output_file_size);
}

Compressor::~Compressor()
{
	liq_result_destroy(quantization_result);
	liq_attr_destroy(handle);
}

int Compressor::GetLowThreshold()
{
	return lowThreshold;
}

void Compressor::SetLowThreshold(int newLowThreshold)
{
	lowThreshold = newLowThreshold;
}