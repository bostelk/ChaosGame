#include "chaos.h"

int
main(void)
{
  srand(420);

  int numPoints = 5e7;
  int numIterations = 20;

  int imageDim = 2048;
  Image image(imageDim, imageDim);
  image.Allocate();

  float frameTime = 1 / 60.0f;
  float minute = 0.5;
  float seconds = minute * 60;
  int numFrames = std::lroundf(seconds / frameTime);

  Eigen::Affine2f t0;
  t0.matrix() << 0.562482f, 0.397861f, -0.539599f, 0.501088, -.42992, -.112404, 0, 0, 1;
  Eigen::Affine2f t1;
  t1.matrix() << 0.830039, -0.496174, 0.16248, 0.750468, 0.91022, 0.288389, 0, 0, 1;


  //RenderAnimation("Affine0", DensityColorMap, numPoints, imageDim, imageDim, numFrames);

  /*
    RenderImage("Sierpinski.png",
                Sierpinski(),
                IdentityColorMap,
                numPoints,
                imageDim,
                imageDim);

    return 0;
    */



  /*
    RenderImage("Affine0-Position.png",
                AffineTransformations({ t0, t1 }),
                PositionColorMap,
                numPoints,
                imageDim,
                imageDim);
    RenderImage("Affine0-Density.png",
                AffineTransformations({ t0, t1 }),
                DensityColorMap,
                numPoints,
                imageDim,
                imageDim);
*/
  RenderImage(image,
              CurryAll(AffineTransformations({ t0, t1 }), SphericalFunc),
              DensityColorMap,
              numPoints,
              numIterations);
  /*
    RenderImage("Affine0-Polar-Density.png",
                CurryAll(AffineTransformations({ t0, t1 }), PolarFunc),
                DensityColorMap,
                numPoints,
                imageDim,
                imageDim);
    RenderImage("Affine0-Horseshoe-Density.png",
                CurryAll(AffineTransformations({ t0, t1 }), HorseshoeFunc),
                DensityColorMap,
                numPoints,
                imageDim,
                imageDim);
    RenderImage("Affine0-Swirl-Density.png",
                CurryAll(AffineTransformations({ t0, t1 }), SwirlFunc),
                DensityColorMap,
                numPoints,
                imageDim,
                imageDim);
  RenderImage("Affine0-Spiral-Density.png",
              CurryAll(AffineTransformations({ t0, t1 }), SpiralFunc),
              DensityColorMap,
              numPoints,
              imageDim,
              imageDim);
  RenderImage("Affine0-Hyperbolic-Density.png",
              CurryAll(AffineTransformations({ t0, t1 }), HyperbolicFunc),
              DensityColorMap,
              numPoints,
              imageDim,
              imageDim);
  RenderImage("Affine0-Diamond-Density.png",
              CurryAll(AffineTransformations({ t0, t1 }), DiamondFunc),
              DensityColorMap,
              numPoints,
              imageDim,
              imageDim);
  RenderImage("Affine0-Disc-Density.png",
              CurryAll(AffineTransformations({ t0, t1 }), DiscFunc),
              DensityColorMap,
              numPoints,
              imageDim,
              imageDim);
  RenderImage("Affine0-Julia-Density.png",
              CurryAll(AffineTransformations({ t0, t1 }), JuliaFunc),
              DensityColorMap,
              numPoints,
              imageDim,
              imageDim);
*/

  image.Release();

  return 0;
}