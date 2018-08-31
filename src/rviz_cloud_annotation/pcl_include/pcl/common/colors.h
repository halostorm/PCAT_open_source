#ifndef PCL_COMMON_COLORS_H
#define PCL_COMMON_COLORS_H

#include <pcl/pcl_macros.h>

namespace pcl
{

  struct RGB;

  PCL_EXPORTS RGB
  getRandomColor (double min = 0.2, double max = 2.8);

  class PCL_EXPORTS GlasbeyLUT
  {

    public:

      /** Get a color from the lookup table with a given id.
        *
        * The id should be less than the size of the LUT (see size()). */
      static RGB at (unsigned int color_id);

      /** Get the number of colors in the lookup table.
        *
        * Note: the number of colors is different from the number of elements
        * in the lookup table (each color is defined by three bytes). */
      static size_t size ();

      /** Get a raw pointer to the lookup table. */
      static const unsigned char* data ();

  };

}

#endif /* PCL_COMMON_COLORS_H */

