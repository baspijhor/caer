/*
 * opticflow.c
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

// NOTE: currently this module should do exactly what the BA filter does

#include "opticflow.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "flowBenosman2014.h"

#define FLOW_BUFFER_SIZE 3

struct OpticFlowFilter_state {
	FlowEventBuffer buffer;
	FlowBenosman2014Params params;
	int8_t subSampleBy;
};

typedef struct OpticFlowFilter_state *OpticFlowFilterState;

static bool caerOpticFlowFilterInit(caerModuleData moduleData);
static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpticFlowFilterConfig(caerModuleData moduleData);
static void caerOpticFlowFilterExit(caerModuleData moduleData);
static bool allocateBuffer(OpticFlowFilterState state, int16_t sourceID);

static struct caer_module_functions caerOpticFlowFilterFunctions = { .moduleInit =
	&caerOpticFlowFilterInit, .moduleRun = &caerOpticFlowFilterRun, .moduleConfig =
	&caerOpticFlowFilterConfig, .moduleExit = &caerOpticFlowFilterExit };

void caerOpticFlowFilter(uint16_t moduleID, FlowEventPacket flow) {
	caerModuleData moduleData = caerMainloopFindModule(moduleID, "OpticFlow");
	if (moduleData == NULL) {
		return;
	}

	caerModuleSM(&caerOpticFlowFilterFunctions, moduleData, sizeof(struct OpticFlowFilter_state), 1, flow);
}

static bool caerOpticFlowFilterInit(caerModuleData moduleData) {
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dtMin", 3000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dtMax", 100000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode, "dx", 3);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "thr1", 1E5);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "thr2", 5e10);
	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	OpticFlowFilterState state = moduleData->moduleState;

	state->params.dtMin = sshsNodeGetInt(moduleData->moduleNode, "dtMin");
	state->params.dtMax = sshsNodeGetInt(moduleData->moduleNode, "dtMax");
	state->params.dx 	= (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "dx");
	state->params.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "thr1");
	state->params.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "thr2");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Nothing that can fail here.
	return (true);
}

static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args) {
	UNUSED_ARGUMENT(argsNumber);

	// Interpret variable arguments (same as above in main function).
	FlowEventPacket flow = va_arg(args, FlowEventPacket);

	// Only process packets with content.
	if (flow == NULL) {
		return;
	}

	// Use the polarity event packet as the main workhorse
	// The FlowEventPacket only encapsulates this packet with flow data
	OpticFlowFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->buffer == NULL) {
		if (!allocateBuffer(state, caerEventPacketHeaderGetEventSource(&(flow->packetHeader)))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for timestampMap.");
			return;
		}
	}

	int counter = 0;

	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber(&(flow->packetHeader)); i++) {
		caerPolarityEvent p = flow->events;
		if (!caerPolarityEventIsValid(p)) { continue; } // Skip invalid polarity events.
		struct flow_event e = flowEventInitFromPolarity(p);

		// Compute optic flow using events in buffer
		flowBenosman2014(&e, state->buffer, state->params);

		// Add event to buffer
		flowEventBufferAdd(e,state->buffer);

		// Update flow event packet
		flowEventPacketUpdate(flow, &e, i);

		// For now, count events in packet and output how many have flow
		if (e.hasFlow) {
			counter++;
		}
	}
	printf("Number of flow events: %i\n",counter);
}

static void caerOpticFlowFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowFilterState state = moduleData->moduleState;

	state->params.dtMin = sshsNodeGetInt(moduleData->moduleNode, "dtMin");
	state->params.dtMax = sshsNodeGetInt(moduleData->moduleNode, "dtMax");
	state->params.dx 	= (uint16_t)sshsNodeGetInt(moduleData->moduleNode, "dx");
	state->params.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "thr1");
	state->params.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "thr2");
	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");
}

static void caerOpticFlowFilterExit(caerModuleData moduleData) {
	// Remove listener, which can reference invalid memory in userData.
	sshsNodeRemoveAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	OpticFlowFilterState state = moduleData->moduleState;

	// Ensure map is freed.
	flowEventBufferFree(state->buffer);
}

static bool allocateBuffer(OpticFlowFilterState state, int16_t sourceID) {
	// Get size information from source.
	sshsNode sourceInfoNode = caerMainloopGetSourceInfo(U16T(sourceID));
	if (sourceInfoNode == NULL) {
		// This should never happen, but we handle it gracefully.
		caerLog(CAER_LOG_ERROR, __func__, "Failed to get source info to allocate flow event buffer.");
		return (false);
	}

	int16_t sizeX = sshsNodeGetShort(sourceInfoNode, "dvsSizeX");
	int16_t sizeY = sshsNodeGetShort(sourceInfoNode, "dvsSizeY");

	state->buffer = flowEventBufferInit((size_t) sizeX, (size_t) sizeY, FLOW_BUFFER_SIZE);
	if (state->buffer == NULL) {
		return (false);
	}

	// TODO: size the map differently if subSampleBy is set!
	return (true);
}