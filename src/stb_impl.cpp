// stb_impl.cpp - stb_image implementation file
// SPDX-License-Identifier: Apache-2.0
//
// stb_image is a single-header library that uses #define STB_IMAGE_IMPLEMENTATION
// to compile. we put the impl here so all other TUs can just #include the header.

#define STB_IMAGE_IMPLEMENTATION
#include "stb/stb_image.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"
