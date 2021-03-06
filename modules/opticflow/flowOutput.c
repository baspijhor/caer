/*
 * uartOutput.c
 *
 *  Created on: Jul 5, 2016
 *      Author: bas
 */

#include "flowOutput.h"
#define SUBSYSTEM_UART "UART"
#define SUBSYSTEM_FILE "Event logger"
#define FILE_MAX_NUMBER_OF_LINES 5000000

static inline FlowEventPacket getPacketFromTransferBuffer(RingBuffer buffer);
static inline bool sendFlowEventPacketUart(FlowEventPacket header);
static int outputHandlerThread(void *stateArg);
static inline bool writeFlowEventPacketFile(flowOutputState state,
		FlowEventPacket packet);

bool initUartOutput(flowOutputState state, char* port, unsigned int baud, size_t bufferSize) {
	// Initialize UART communication
	int uartErrno = uart_open(port, baud);
	if (uartErrno) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,
				"Failed to identify serial communication, errno=%i.",uartErrno);
		return (false);
	}
	// Test message
	char* testMessage = "DVS128UART";
	if (uart_tx((int) strlen(testMessage), (unsigned char*) testMessage)) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,
				"Test transmission unsuccessful - connection closed.");
		uart_close();
		return(false);
	}

	// Start output handler thread
	state->thread = 0;
	if (thrd_create(&state->thread, &outputHandlerThread, state) != thrd_success) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,"Failed to start output handler thread");
		return (false);
	}
	state->buffer = ringBufferInit(bufferSize);
	if (state->buffer == NULL) {
		caerLog(CAER_LOG_ERROR, SUBSYSTEM_UART, "Failed to allocate transfer ring-buffer.");
		return (false);
	}
	atomic_store(&state->running, true);
	caerLog(CAER_LOG_NOTICE, SUBSYSTEM_UART, "Streaming flow events to port %s.",port);
	return (true);
}

bool initFileOutput(flowOutputState state, char* fileName, size_t bufferSize) {
	// Initialize file communication
	state->file = fopen(fileName,"w+");
	if (state->file == NULL) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to open file");
		return (false);
	}
	// Write header as check for file output
	if (fprintf(state->file,"#cAER event data with optic flow values\n") < 0) {
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,"Failed to write header");
		fclose(state->file);
		return (false);
	}
	// Write creation date
	time_t rawTime;
	struct tm * timeInfo;
	time (&rawTime);
	timeInfo = localtime(&rawTime);
	fprintf(state->file,"#Date created: %s\n",asctime(timeInfo));
	// Write column legend
	fprintf(state->file,"#x,y,t,p,u,v\n");

	state->fileLineNumber = 4;

	// Start output handler thread (ONLY IF NOT YET CALLED BY UART)
	if (state->mode != OF_OUT_BOTH ||
			!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		state->mode = OF_OUT_FILE; // to ensure that only file output is performed in this case
		state->thread = 0;
		if (thrd_create(&state->thread, &outputHandlerThread, state) != thrd_success) {
			caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE,
					"Failed to start output handler thread");
			return (false);
		}
		state->buffer = ringBufferInit(bufferSize);
		if (state->buffer == NULL) {
			caerLog(CAER_LOG_ERROR, SUBSYSTEM_FILE,
					"Failed to allocate transfer ring-buffer.");
			return (false);
		}
		atomic_store(&state->running, true);
	}
	caerLog(CAER_LOG_NOTICE, SUBSYSTEM_FILE, "Logging events to fileName %s",fileName);

	return (true);
}

void closeUartOutput(flowOutputState state) {
	state->running = false;
	uart_close();

	if ((errno = thrd_join(state->thread, NULL)) != thrd_success) {
		// This should never happen!
		caerLog(CAER_LOG_CRITICAL, SUBSYSTEM_UART,
				"Failed to join output handling thread. Error: %d.", errno);
	}
	ringBufferFree(state->buffer);
}


void closeFileOutput(flowOutputState state) {
	if (state->mode != OF_OUT_BOTH) {
		state->running = false;
		if ((errno = thrd_join(state->thread, NULL)) != thrd_success) {
			// This should never happen!
			caerLog(CAER_LOG_CRITICAL, SUBSYSTEM_UART,
					"Failed to join output handling thread. Error: %d.", errno);
		}
		ringBufferFree(state->buffer);
	}
	fclose(state->file);
}

void addPacketToTransferBuffer(flowOutputState state, FlowEventPacket packet,
		int32_t flowNumber) {
	if (packet == NULL) {
		// There was nothing in this mainloop run: return
		return;
	}

	caerEventPacketHeader header = &packet->packetHeader;

	// If no valid events are present, there is nothing to add
	if (caerEventPacketHeaderGetEventValid(header) == 0) {
		return;
	}
	if (flowNumber == 0) {
		return;
	}

	// Allocate memory for new flow event packet
	FlowEventPacket copy = malloc(sizeof(struct flow_event_packet));
	if (copy == NULL) {
			caerLog(CAER_LOG_ERROR, SUBSYSTEM_UART,"Failed to copy event packet.");
		return;
	}

	// Copy header information
	copy->packetHeader = packet->packetHeader;
	copy->packetHeader.eventNumber = flowNumber;
	copy->packetHeader.eventValid = flowNumber;
	copy->events = malloc(sizeof(struct flow_event) * (size_t) flowNumber);

	// Copy events
	int j = 0;
	for (int i = 0; i < header->eventNumber; i++) {
		FlowEvent e = &packet->events[i];
		if (e->hasFlow) {
			copy->events[j] = *e;
			j++;
		}
	}

	// Verify number of events copied
	if (j != flowNumber) {
		caerLog(CAER_LOG_ERROR, SUBSYSTEM_FILE,
				"Copied %d events while number of flow events is %d.",
				j, flowNumber);
		flowEventPacketFree(copy);
		return;
	}

	if (!ringBufferPut(state->buffer, copy)) {
		flowEventPacketFree(copy);
		caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART,"Failed to add event packet to ring buffer.");
		return;
	}
}

static inline FlowEventPacket getPacketFromTransferBuffer(RingBuffer buffer) {
	FlowEventPacket packet = ringBufferGet(buffer);
	repeat: if (packet != NULL) {
		// Are there others? Only render last one, to avoid getting backed up!
		FlowEventPacket packet2 = ringBufferGet(buffer);

		if (packet2 != NULL) {
			flowEventPacketFree(packet);
			packet = packet2;
			goto repeat;
		}
	}

	return (packet);
}

static inline bool sendFlowEventPacketUart(FlowEventPacket flow) {
	caerEventPacketHeader header = (caerEventPacketHeader) flow;
	uint32_t packetSize = (uint32_t) caerEventPacketHeaderGetEventNumber(header);
	if (packetSize == 0) {
		// No events to send - return
		return (false);
	}
	// Events are separated by unsigned ints of value 255. This value should
	// never occur as pixel location
	unsigned char eventSeparator = 255;

	// Iterate through packet and send events
	for (uint32_t i = 0; i < packetSize; i++) {
		FlowEvent e = &(flow->events[i]);
		if (e == NULL) {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Event null pointer found.");
			return (false);
		}
		if (!e->hasFlow) {continue;}
		uint8_t x = (uint8_t) caerPolarityEventGetX((caerPolarityEvent) e);
		uint8_t y = (uint8_t) caerPolarityEventGetY((caerPolarityEvent) e);
		int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
		int16_t u = (int16_t) (e->u*100);
		int16_t v = (int16_t) (e->v*100);

		// Send data over UART
		if (uart_tx(sizeof(x),(unsigned char*) &x)
				|| uart_tx(sizeof(y),(unsigned char*) &y)
				|| uart_tx(sizeof(t),(unsigned char*) &t)
				|| uart_tx(sizeof(u),(unsigned char*) &u)
				|| uart_tx(sizeof(v),(unsigned char*) &v)
				|| uart_tx(sizeof(eventSeparator), &eventSeparator))  {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_UART,"Event info not fully sent.");
			return (false);
		}
	}
	return (true);
}

static inline bool writeFlowEventPacketFile(flowOutputState state,
		FlowEventPacket flow) {
	caerEventPacketHeader header = (caerEventPacketHeader) flow;

	if (caerEventPacketHeaderGetEventValid(header) == 0) {
		return (false);
	}

	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber(header); i++) {
		FlowEvent e = &(flow->events[i]);
		if (e == NULL) {
			caerLog(CAER_LOG_ERROR,SUBSYSTEM_FILE,"Event null pointer found.");
			return (false);
		}
		if (!e->hasFlow) {continue;}

		// Get event data
		uint8_t x = (uint8_t) caerPolarityEventGetX((caerPolarityEvent) e);
		uint8_t y = (uint8_t) caerPolarityEventGetY((caerPolarityEvent) e);
		int32_t t = caerPolarityEventGetTimestamp((caerPolarityEvent) e);
		bool p = caerPolarityEventGetPolarity((caerPolarityEvent) e);
		double u = e->u;
		double v = e->v;

		// Write to file
		if (state->fileLineNumber < FILE_MAX_NUMBER_OF_LINES)  {
			fprintf(state->file,"%3i,%3i,%10i,%d,%4.3f,%4.3f\n",x,y,t,p,u,v);
			state->fileLineNumber++;
		}
		else {
			// If too many lines, stop logging to prevent overrun
			if (state->fileLineNumber == FILE_MAX_NUMBER_OF_LINES) {
				caerLog(CAER_LOG_NOTICE, SUBSYSTEM_FILE,
						"File log reached limit of %d lines - "
						"no more lines will be added.", FILE_MAX_NUMBER_OF_LINES);
				state->fileLineNumber++;
			}
		}
	}
	return (true);
}

static int outputHandlerThread(void *stateArg) {
	if (stateArg == NULL) {
		return (thrd_error);
	}
	flowOutputState state = stateArg;
	switch (state->mode) {
		case OF_OUT_UART:
			thrd_set_name(SUBSYSTEM_UART);
			break;
		case OF_OUT_FILE:
			thrd_set_name(SUBSYSTEM_FILE);
			break;
		case OF_OUT_BOTH:
			thrd_set_name("FLOW_OUTPUT");
			break;
		case OF_OUT_NONE:
			break;
	}

	struct timespec sleepTime = { .tv_sec = 0, .tv_nsec = 500000 };

	// Wait until the buffer is initialized
	while (!atomic_load_explicit(&state->running, memory_order_relaxed)) {
		thrd_sleep(&sleepTime, NULL);
	}

	// Main thread loop
	while (atomic_load_explicit(&state->running, memory_order_relaxed)) {
		FlowEventPacket packet = getPacketFromTransferBuffer(state->buffer);
		if (packet == NULL) { // no data: sleep for a while
			thrd_sleep(&sleepTime, NULL);
		}
		else {
			if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
				if (!sendFlowEventPacketUart(packet)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow packet was not sent.");
				}
			}
			if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
				if (!writeFlowEventPacketFile(state,packet)) {
					caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow packet was not written.");
				}
			}
			flowEventPacketFree(packet);
		}
	}

	// If shutdown: empty buffer before closing thread
	FlowEventPacket packet;
	while ((packet = getPacketFromTransferBuffer(state->buffer)) != NULL) {
		if (state->mode == OF_OUT_UART || state->mode == OF_OUT_BOTH) {
			if (!sendFlowEventPacketUart(packet)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_UART, "A flow packet was not sent.");
			}
		}
		if (state->mode == OF_OUT_FILE || state->mode == OF_OUT_BOTH) {
			if (!writeFlowEventPacketFile(state,packet)) {
				caerLog(CAER_LOG_ALERT, SUBSYSTEM_FILE, "A flow packet was not written.");
			}
		}
		flowEventPacketFree(packet);
	}

	return (thrd_success);
}
