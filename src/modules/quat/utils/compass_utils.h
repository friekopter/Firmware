
#ifndef _compass_utils_h
#define _compass_utils_h

#include <systemlib/visibility.h>

__BEGIN_DECLS

__EXPORT float compassNormalize(float heading);
__EXPORT float compassDifference(float a, float b);
__EXPORT float compassNormalizeRad(float heading);
__EXPORT float compassDifferenceRad(float a, float b);

__END_DECLS

#endif
