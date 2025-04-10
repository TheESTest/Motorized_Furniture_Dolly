# -*- coding: utf-8 -*-
r"""
Batch Convert PNG to BMP

This script is used for converting all .PNG files in a given folder into .BMP,
specifically used in the process of taking an app/set of screens laid out in
Figma or Inkscape and displaying them on an ESP32 + connected display.

27 Mar 2025 - Extended: Now supports user-specified 8, 24, or 32-bit output.
6 Apr 2025 - Adding support for 4-bit image output using ImageMagick, download,
install, add to path and install legacy tools during installation:
https://imagemagick.org/script/download.php#windows


"""

import os
from PIL import Image
import subprocess

# Toggle to convert PNG to C header files
convert_to_c_headers_bool = True
# Toggle to apply color quantization
apply_color_quantization = False
# Toggle to adjust endianess - has to do with the color output
adjust_endianess = True
# Choose desired output BMP bit depth (8, 24, or 32)
output_bit_depth = 4  # Options: 4, 8, 24, 32

#You only need to install ImageMagick if you want to output 4-bit BMP images.
#If so, specify the executable file for your ImageMagick installation here.
#For example:
MAGICK_PATH = 'C:\Program Files\ImageMagick-7.1.1-Q16-HDRI\magick.exe'

def convert_to_4bit_bmp(input_path, output_path):
    try:
        result = subprocess.run(
            f'"{MAGICK_PATH}" "{input_path}" -colors 16 -type palette -define bmp:format=bmp3 "{output_path}"',
            shell=True,
            check=True,
            capture_output=True,
            text=True
        )
        print(result.stdout)
    except subprocess.CalledProcessError as e:
        print("ImageMagick conversion failed:")
        print(e.stderr)

def quantize_color(r, g, b):
    """
    Simplify colors to make the pixels crisper.
    Map colors to the nearest of black, white, red.
    """
    colors = {
        'black': (0, 0, 0),
        'white': (255, 255, 255),
        'red': (255, 0, 0),
    }
    
    closest_color = min(colors, key=lambda k: (colors[k][0]-r)**2 + (colors[k][1]-g)**2 + (colors[k][2]-b)**2)
    return colors[closest_color]

def convert_png_to_bmp(source_folder):
    """
    Convert all PNG images in the specified folder to BMP format and optionally quantize colors.

    :param source_folder: Path to the folder containing PNG images.
    """
    for file_name in os.listdir(source_folder):
        if file_name.endswith('.png'):
            sanitized_file_name = file_name.replace(' ', '_').replace('-', '_')
            if sanitized_file_name != file_name:
                os.rename(os.path.join(source_folder, file_name), os.path.join(source_folder, sanitized_file_name))
                file_name = sanitized_file_name

            file_path = os.path.join(source_folder, file_name)
            with Image.open(file_path) as img:
                # Convert to appropriate bit depth
                if output_bit_depth == 4:
                    # Convert to RGB first, then quantize down to 16 colors
                    img = img.convert("RGB").quantize(colors=16, method=Image.MEDIANCUT)
                elif output_bit_depth == 8:
                    img = img.convert('P')  # 8-bit palettized
                elif output_bit_depth == 24:
                    img = img.convert('RGB')  # 24-bit
                elif output_bit_depth == 32:
                    img = img.convert('RGBA')  # 32-bit with alpha
                else:
                    raise ValueError("Unsupported output_bit_depth. Choose 4, 8, 24, or 32.")

                # Optional color quantization
                if apply_color_quantization:
                    pixels = img.load()
                    for y in range(img.height):
                        for x in range(img.width):
                            original_color = pixels[x, y]
                            quantized_color = quantize_color(*original_color[:3])
                            pixels[x, y] = quantized_color + (original_color[3],) if img.mode == 'RGBA' else quantized_color

                bmp_filename = os.path.splitext(file_name)[0] + '.bmp'
                bmp_filename = bmp_filename.replace(' ', '_').replace(':', '_')
                bmp_path = os.path.join(source_folder, bmp_filename)
                bmp_path = os.path.join(source_folder, bmp_filename)
                # Save temporary BMP
                img.save(bmp_path, 'BMP')
                
                # If user selected 4-bit, run ImageMagick to downconvert
                if output_bit_depth == 4:
                    bmp_4bit_path = bmp_path.replace(".bmp", "_4bit.bmp")
                    convert_to_4bit_bmp(bmp_path, bmp_4bit_path)
                    os.remove(bmp_path)  # remove the larger 8-bit version
                    os.rename(bmp_4bit_path, bmp_path)
                print(f"Converted {file_name} to {bmp_filename}")

                if convert_to_c_headers_bool:
                    with Image.open(bmp_path) as img:
                        img = img.convert('RGB')  # Force RGB so pixel[:3] always works
                        width, height = img.size
                        data = []
                        for y in range(height):
                            row = []
                            for x in range(width):
                                pixel = img.getpixel((x, y))
                                r, g, b = pixel[:3]

                                # Swapping B and R values for correct RGB565 format
                                color = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

                                # Apply endianess adjustment if needed
                                if adjust_endianess:
                                    color = ((color & 0xFF) << 8) | ((color >> 8) & 0xFF)

                                row.append(f"0x{color:04X}")
                            data.append(", ".join(row))

                        # Construct header file content
                        variable_name = os.path.splitext(os.path.basename(file_path))[0]
                        array_name = f"{variable_name}_data"
                        header_content = f"const uint16_t {array_name}[] = {{\n"
                        for line in data:
                            header_content += f"    {line},\n"
                        header_content = header_content.rstrip(',\n') + '\n};\n'

                        # Write to header file
                        header_filename = f"{variable_name}.h"
                        with open(os.path.join(source_folder, header_filename), 'w') as file:
                            file.write(header_content)
                        print(f"Header file created: {header_filename}")

# Specify the folder where the PNG files are, to batch convert them to BMP files:
source_folder = r'C:\(Your folder path here)'

print()
print('Converting all .png files in the following folder...')
print(source_folder)
convert_png_to_bmp(source_folder)
print()
print('Script completed.')











