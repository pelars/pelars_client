#pragma once
typedef int XnStatus;
typedef int XnBool;
typedef unsigned char XnUInt8;
typedef signed char XnInt8;
typedef unsigned int XnUInt32;
typedef unsigned short XnUInt16;
typedef signed short XnInt16;

#define XN_MAX_UINT16 65536
#define XN_VALIDATE_INPUT_PTR(x)
#define xnLogError(a,b)
#define XN_STATUS_BAD_PARAM -1
#define XN_STATUS_OK 0
#define XN_STATUS_OUTPUT_BUFFER_OVERFLOW -2
#define XN_PREPARE_VAR16_IN_BUFFER(var) (var)
#define xnOSMemSet(a,b,c) memset(a,b,c)
#include <memory.h>
#define abs(x) ((x) < 0 ? -x : x)
// XN_PREPARE_VAR16_IN_BUFFER
// XN_CHECK_OUTPUT_OVERFLOW
/** Returns an input overflow error if x is beyond y */
#define XN_CHECK_OUTPUT_OVERFLOW(x, y)					\
		if (x > y)										\
		{												\
			return (XN_STATUS_OUTPUT_BUFFER_OVERFLOW);	\
		}

#ifdef __MINGW32__
#define DE __declspec(dllexport)
#else
#define DE
#endif
extern "C"
{
void DE initlibxndec();

XnStatus DE XnStreamUncompressConf4(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamUncompressImage8Z(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamUncompressDepth16Z(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt16* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamUncompressDepth16ZWithEmbTable(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt16* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamCompressDepth16Z(const XnUInt16* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamCompressDepth16ZWithEmbTable(const XnUInt16* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize, XnUInt16 nMaxValue);

XnStatus DE XnStreamCompressImage8Z(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);

XnStatus DE XnStreamCompressConf4(const XnUInt8* pInput, const XnUInt32 nInputSize, XnUInt8* pOutput, XnUInt32* pnOutputSize);
}