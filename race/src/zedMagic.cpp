#include <stdio.h>
#include <math.h>
#include "zedMagic.h"
#define square(x) x*x
#define THRESHOLD 70

const int newH = H_O;
const int newW = W_O;

std::vector <unsigned char> scaleAndCrop(std::vector <unsigned char> imgData) {
    std::vector <unsigned char> scaled; //to scale down image
    int W = W_I;
    for (int i=0;i<newH;i++) {
        for (int j=0;j<newW*3;j++) {
            long sum=0;
            //sum += scaled[3*newW*i + 3*j +k];
            sum=sum+imgData[(2*j)   + 3*W*(2*i)  ];
            sum=sum+imgData[(2*j+3) + 3*W*(2*i)  ];
            sum=sum+imgData[(2*j)   + 3*W*(2*i+1)];
            sum=sum+imgData[(2*j+3) + 3*W*(2*i+1)];
            scaled.push_back(sum/4);
        }
    }
    return scaled;
}

std::vector <unsigned char> toGrayscale(std::vector <unsigned char> scaled) {
    std::vector <unsigned char> bw;
    /* Consolidate color into black and white photo */
    for (int i=0;i<newH;i++) {
        for (int j=0;j<newW;j++) {
            long sum = 0;
            int k;
            for (k=-1;k<2;k++) sum += scaled[3*newW*i + 3*j +k];
            bw.push_back(sum/3);
        }
    }
    return bw;
}

std::vector <unsigned char> scaleCropGrayscale(std::vector <unsigned char> imgData) {
    std::vector <unsigned char> scCrop;
    int i,j, W=W_I;
    for (i=0;i<H_O;i++) {
        for (j=0;j<W_O;j++) {
            long sum = 0;
            for (int k = 0; k<6; k++) {
                sum += imgData[6*j + k + 6*W*i];
                sum += imgData[6*j + k + 6*W*i + 3*W];
            }
            scCrop.push_back(sum/12);
        }
    } //scaled and cropped grayscale into array

    return scCrop;

}

std::vector <unsigned char> getEdges(std::vector <unsigned char> bw) {
    std::vector <double> edges; // to hold sgm 
    /* Calculate dx, dy, sgm */
    int i,j;
    int dx,dy;
    double max = 0;
    for (i=0;i<newH;i++) {
        if (i==0 || i==(newH-1)) { // top/bottom row
            for (j=0;j<newW;j++) edges.push_back(0);
        }
        else {
            edges.push_back(0); // left column
            for (j=1;j<newW-1;j++) {
                int index = newW*i + j; 
                dx = bw[index+newW+1] + 2*bw[index+1] + bw[index-newW+1] \
                - (bw[index+newW-1] + 2*bw[index-1] + bw[index-newW-1]);
                dy = bw[index-newW-1] + 2*bw[index-newW] + bw[index-newW+1]\
                - (bw[index+newW-1] + 2*bw[index+newW] + bw[index+newW+1]);
                double sgm = sqrt(square(dx) + square(dy));
                max = (max<sgm)? sgm: max;
                edges.push_back(sgm);
            }
            edges.push_back(0); // right column
        }
    }

    std::vector <unsigned char> edge_img;
    /* Scale edge (sgm) data to 0 to 255 range */
    for (i=0;i<edges.size();i++) edge_img.push_back((edges[i]/max*255 > THRESHOLD)?255 :0);
    return edge_img;
}


