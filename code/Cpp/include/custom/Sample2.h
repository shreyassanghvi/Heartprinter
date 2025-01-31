//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//
// very simple classes for the devices

class CSystem
{
public:
	SYSTEM_CONFIGURATION	m_config;
}; 

class CSensor
{
public:
	SENSOR_CONFIGURATION	m_config;
};

class CXmtr
{
public:
	TRANSMITTER_CONFIGURATION	m_config;
};

class CBoard
{
public:
	BOARD_CONFIGURATION		m_config;
};


///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
//
//	MACROS for simplifying the procedure calls
//
// This macro will set a system parameter and call the error handler if there is 
// an error reported. Note These macros do not print to the standard output the
// set value

#define SET_SYSTEM_PARAMETER(type, value, l)									\
	{																			\
		type##_TYPE buffer = value;												\
		type##_TYPE *pBuffer = &buffer;											\
		errorCode = SetSystemParameter(type, pBuffer, sizeof(buffer));			\
		if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode, l);			\
	}

#define	SET_SENSOR_PARAMETER(sensor, type, value, l)							\
	{																			\
		type##_TYPE buffer = value;												\
		type##_TYPE *pBuffer = &buffer;											\
		errorCode = SetSensorParameter(sensor, type, pBuffer, sizeof(buffer));	\
		if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode, l);			\
	}

#define	SET_TRANSMITTER_PARAMETER(xmtr, type, value, l)							\
	{																			\
		type##_TYPE buf = value;												\
		type##_TYPE *pBuf = &buf;												\
		errorCode = SetTransmitterParameter(xmtr, type, pBuf, sizeof(buf));		\
		if(errorCode!=BIRD_ERROR_SUCCESS) errorHandler(errorCode, l);			\
	}

// In order for the above macros to compile without error it is necessary 
// to provide typedefs for all the XXX_TYPEs that are generated by "type##_TYPE"
typedef short					SELECT_TRANSMITTER_TYPE;
typedef double					POWER_LINE_FREQUENCY_TYPE;
// AGC_MODE_TYPE already defined as an enumerated type
typedef double					MEASUREMENT_RATE_TYPE;
typedef	double					MAXIMUM_RANGE_TYPE;
typedef BOOL					METRIC_TYPE;
// DATA_FORMAT_TYPE already defined as an enumerated type
typedef DOUBLE_ANGLES_RECORD	ANGLE_ALIGN_TYPE;
typedef DOUBLE_ANGLES_RECORD	REFERENCE_FRAME_TYPE;
typedef	BOOL					XYZ_REFERENCE_FRAME_TYPE;
// HEMISPHERE_TYPE already defined as an enumerated type
typedef	BOOL					FILTER_AC_WIDE_NOTCH_TYPE;
typedef	BOOL					FILTER_AC_NARROW_NOTCH_TYPE;
typedef	double					FILTER_DC_ADAPTIVE_TYPE;
typedef	ADAPTIVE_PARAMETERS		FILTER_ALPHA_PARAMETERS_TYPE;
typedef	BOOL					FILTER_LARGE_CHANGE_TYPE;
typedef	QUALITY_PARAMETERS		QUALITY_TYPE;







