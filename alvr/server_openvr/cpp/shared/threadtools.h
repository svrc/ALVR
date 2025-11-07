//===================== Copyright (c) Valve Corporation. All Rights Reserved. ======================
//
// Helper classes for working with threads.
//
//==================================================================================================
#pragma once

#include <thread>
#ifdef _WIN32
#include <windows.h>
#endif

#define THREAD_PRIORITY_MOST_URGENT 15

class CThreadPair
{
public:
	CThreadPair();
	virtual ~CThreadPair();
	virtual bool Init() { return true; }
	virtual void RunConsumer() = 0;
	virtual void RunProducer() = 0;
	void Start();
	void Join();
private:
	std::thread *m_pThreadConsumer;
	std::thread *m_pThreadProducer;
};

#ifdef _WIN32
class CThreadEvent
{
public:
	CThreadEvent( bool bManualReset = false );
	~CThreadEvent();
	bool Wait( uint32_t nTimeoutMs = INFINITE );
	bool Set();
	bool Reset();
private:
	HANDLE m_hSyncObject;
};
#endif
