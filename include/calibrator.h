#pragma once
#ifdef HAS_ARUCO


void calibration(const unsigned int id, const float marker_size);
#else
inline void calibration(const unsigned int id, const float marker_size)
{
	
}

#endif