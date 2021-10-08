// common.h
// This head defines some commonly-used macros and structs.
#ifndef COMMON_H_
#define COMMON_H_

// SharedMemoryName
// This is the name of the shared memory region that the user-space and
// real-time code will use to communcate.
#define SharedMemoryName L"WindowsRealtimeSharedMemory"


// Commands values used in commands sent from the Windows process to the real-time
// process.
#define CMD_CREATE_RTCONSUMER	100
#define	CMD_ADD_DATAOWNER		101
#define CMD_START_RTCONSUMER	102
#define CMD_STOP_RTCONSUMER		103

// Status values returned from the real-time process to the Windows process.
#define STATUS_OK				0
#define STATUS_UNKNOWN_CMD		1


// DataOwnerSharedData
// This structure defines the structure of the data region shared between the user-space
// process and the real-time process.  There is one instance of this structure for each
// instance of class DataOwner (because each instance of class DataOwner has its own
// shared memory region).
typedef struct DataOwnerSharedData
{
	// This member holds the name of the DataOwner object.
	wchar_t	name[64];

	struct
	{
		// inputValue
		// This member holds a floating point value to be read by the real-time process.
		// This member is written by the Windows process in project WinProducer.
		float inputValue;

		// outputValue
		// This member holds a floating point value that is written by the real-time
		// process.  This member is read by the Windows process in project WinProducer.
		float outputValue;
	} data;
} DOSHAREDDATA, * PDOSHAREDDATA;


// RtConsumerCSB
// This structure defines a command/status block (CSB) used to communicate between the
// Windows process and the real-time process.  When member commandIsReady transitions from
// 0 to 1, there is a valid command in this structure to be consumed by the real-time
// process.  When member commandIsReady transitions from 1 to 0, there is a valid status
// in this structure describing the result of the previous command send via this structure.
typedef struct RtConsumerCSB
{
	DWORD commandIsReady;
	DWORD Command;
	DWORD status;
	wchar_t csBuffer[64];
} RTCONSUMERCSB, * PRTCONSUMERCSB;

#endif
#pragma once
#pragma once
