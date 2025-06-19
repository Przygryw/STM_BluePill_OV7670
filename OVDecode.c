#include <stdio.h>
#include <stdlib.h>

#define WIDTH 640
#define HEIGHT 480

void main() {
    FILE *bmp_header = fopen("red_zdjecie11.bmp", "rb");  // Read BMP header
    FILE *raw_data = fopen("OV.bin", "rb");  // Read OV7670 data
    FILE *output_bmp = fopen("decoded.bmp", "wb");  // Output BMP file
    int i =0;
    unsigned char byte1;
    unsigned char byte2;
    //byte1 = fgetc(raw_data);
    for(int i =0; i < 54; i++) fputc(fgetc(bmp_header), output_bmp);

    for(int rows = 0; rows< 480; rows++)
    {
        for(int pixels =0; pixels < 640; pixels++)
        {
            if(pixels < 154 && rows < 144) 
            {
                byte1 = fgetc(raw_data);
                byte2 = fgetc(raw_data);
                //char col = (byte2 & (0b01111100));
                char pix[3];
                
                pix[0] = ((byte1 & 0x7C)>>2)*255/31;  //R
                pix[1] = ((((byte1 & 0x03)<<3)|(byte2 & 0xE0)>>5))*255/31;
                pix[2] = (((byte2 & 0x1F) << 0))*255/31; //B 
                  
                fputc(pix[2], output_bmp);
                fputc(pix[1], output_bmp);
                fputc(pix[0], output_bmp);
            }
            else 
            {
                fputc(0, output_bmp);
                fputc(0, output_bmp);
                fputc(0, output_bmp);  
            }
        }
    }
    fclose(bmp_header);
    fclose(raw_data);
    fclose(output_bmp);
}
