#ifndef FLOWEVENT_H_
#define FLOWEVENT_H_

#include <math.h>
#include <stdbool.h>
#include "main.h"
#include <libcaer/events/polarity.h>

#define FLOW_EVENT_TYPE 101

// Flow event struct is only for in-between processing - not actually maintained.
struct flow_event {
	uint16_t x,y;
	int32_t timestamp;
	bool p;
	double u,v;
	bool hasFlow;
}__attribute__((__packed__));

typedef struct flow_event *FlowEvent;

// Extends caerPolarityEventPacket by adding flow data
struct flow_event_packet {
	struct caer_event_packet_header packetHeader;
	struct caer_polarity_event *events;
	double * u;
	double * v;
	bool * hasFlow;
//	size_t testSize;
}__attribute__((__packed__));

typedef struct flow_event_packet *FlowEventPacket;

struct flow_event_buffer {
	struct flow_event*** buffer;
	size_t sizeX;
	size_t sizeY;
	size_t size;
}__attribute__((__packed__));

typedef struct flow_event_buffer *FlowEventBuffer;

static inline struct flow_event flowEventInit(uint16_t x, uint16_t y, int32_t t, bool p) {
	struct flow_event e;
	e.x = x;
	e.y = y;
	e.timestamp = t;
	e.p = p;
	e.u = 0;
	e.v = 0;
	e.hasFlow = false;
	return (e);
}

static inline struct flow_event flowEventInitFromPolarity(caerPolarityEvent polarity) {
	uint16_t x = caerPolarityEventGetX(polarity);
	uint16_t y = caerPolarityEventGetY(polarity);
	int32_t  t = caerPolarityEventGetTimestamp(polarity);
	bool 	 p = caerPolarityEventGetPolarity(polarity);
	return (flowEventInit(x,y,t,p));
}

static inline FlowEventPacket flowEventPacketInitFromPolarity(caerPolarityEventPacket polarity) {
	if (polarity == NULL) {
			return NULL;
		}
	if (polarity->packetHeader.eventNumber == 0) {
		return NULL;
	}
	FlowEventPacket flow = malloc(sizeof(struct flow_event_packet));
	flow->packetHeader = polarity->packetHeader;
	flow->events = polarity->events;

	size_t length = (size_t) polarity->packetHeader.eventNumber;
	flow->u = calloc(length,sizeof(float));
	flow->v = calloc(length,sizeof(float));
	flow->hasFlow = calloc(length,sizeof(bool));
//	flow->testSize = length;

	// Assign new properties to packet header
	caerEventPacketHeaderSetEventType(&(flow->packetHeader), FLOW_EVENT_TYPE);
//	caerEventPacketHeaderSetEventSize(&(flow->packetHeader),
//			caerEventPacketHeaderGetEventSize(&(flow->packetHeader))
//			+ 2*sizeof(float) + sizeof(bool));

	return (flow);
}
/**
 * Make a copy of a flow event packet, sized down to only include the
 * currently present events (eventNumber, valid+invalid), and not
 * including the possible extra unused events (up to eventCapacity).
 *
 * @param eventPacket an event packet to copy.
 *
 * @return a sized down copy of an event packet.
 */
static inline void *copyFlowEventPacketOnlyEvents(FlowEventPacket flow) {
	// Handle empty event packets.
	if (flow == NULL) {
		return (NULL);
	}

	// Calculate needed memory for new event packet.
	caerEventPacketHeader header = &(flow->packetHeader);
	int32_t eventNumber = caerEventPacketHeaderGetEventNumber(header);
	int32_t eventSize = caerEventPacketHeaderGetEventSize(header);
	if (eventNumber == 0) {
		// No copy possible if result is empty (capacity=0).
		return (NULL);
	}

	size_t packetMem = CAER_EVENT_PACKET_HEADER_SIZE + (size_t) (eventSize * eventNumber)
			+ (size_t)((2*sizeof(double) + sizeof(bool)) * eventNumber);

	// Allocate memory for new event packet.
	FlowEvent flowCopy = malloc(packetMem);
	if (flowCopy == NULL) {
		// Failed to allocate memory.
		return (NULL);
	}

	// Copy the data over.
	memcpy(flowCopy, flow, packetMem);

	// Set the event capacity to the event number, since we only allocated
	// memory for that many events.
	caerEventPacketHeaderSetEventCapacity(flowCopy, eventNumber);

	return (flowCopy);
}

// Update FlowEventPacket, adding flow vector values if found
static inline void flowEventPacketUpdate(FlowEventPacket flow, FlowEvent e, int i) {
	if (e->hasFlow) {
		flow->u[i] = e->u;
		flow->v[i] = e->v;
		flow->hasFlow[i] = true;
	}
	else {
		flow->hasFlow[i] = false;
	}
}

static inline void flowEventPacketFree(FlowEventPacket flow) {
	if (flow != NULL) {
		if (flow->u != NULL) {
			free(flow->u);
		}
		if (flow->v != NULL) {
			free(flow->v);
		}
		if (flow->hasFlow != NULL) {
			free(flow->hasFlow);
		}
		free(flow);
	}
}

static inline FlowEventBuffer flowEventBufferInit(size_t width, size_t height, size_t size) {
	FlowEventBuffer buffer = malloc(sizeof(*buffer) + (width * sizeof(struct flow_event *)));
	buffer->sizeX = width;
	buffer->sizeY = height;
	buffer->size = size;

	buffer->buffer = malloc(width * sizeof(double*));
	uint16_t i,j,k;
	for (i = 0; i < width; i++) {
		buffer->buffer [i] = malloc(height * sizeof(double*));
		for (j = 0; j < height; j++) {
			buffer->buffer [i][j] = malloc(size * sizeof(struct flow_event));
			for (k = 0; k < size; k++) {
				buffer->buffer [i][j][k] = flowEventInit(i,j,0,false);
			}
		}
	}
	return (buffer);
}

static inline void flowEventBufferAdd(struct flow_event e, FlowEventBuffer buffer) {
	// TODO make buffer update more efficient for larger buffer size
	size_t i;
	for (i = buffer->size-1; i > 0 ; i--) {
		buffer->buffer[e.x][e.y][i] = buffer->buffer[e.x][e.y][i-1];
	}
	buffer->buffer[e.x][e.y][0] = e;
}

static inline void flowEventBufferFree(FlowEventBuffer buffer) {
	if (buffer != NULL) {
		uint16_t i,j;
		for (i = 0; i < buffer->sizeX; i++) {
			for (j = 0; j < buffer->sizeY; j++) {
				free(buffer->buffer[i][j]);
			}
			free(buffer->buffer[i]);
		}
		free(buffer->buffer);
		free(buffer);
	}
}

#endif
