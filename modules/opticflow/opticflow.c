/*
 * opticflow.c
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

#include "opticflow.h"
#include "base/mainloop.h"
#include "base/module.h"
#include "ext/buffers.h"
#include "ext/portable_time.h"
#include "flowEvent.h"
#include "flowBenosman2014.h"
#include "flowRegularizationFilter.h"

#include <string.h>

#define FLOW_BUFFER_SIZE 3
#define DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW 1e6/115.0
#define DIFF_ON_MAX 4000000
#define DIFF_ON_MIN 209996
#define DIFF_OFF_MAX 132
#define DIFF_OFF_MIN 2

struct timespec timeInit;
int64_t timeInitEvent = 0;
bool timeSet = 0;

struct OpticFlowFilter_state {
	FlowEventBuffer buffer;
	FlowBenosman2014Params flowParams;
	FlowRegularizationFilterParams filterParams;
	bool enableFlowRegularization;
	int64_t refractoryPeriod;
	int8_t subSampleBy;
	sshsNode DVS128Node;
	double wx, wy;
};

typedef struct OpticFlowFilter_state *OpticFlowFilterState;

static bool caerOpticFlowFilterInit(caerModuleData moduleData);
static void caerOpticFlowFilterRun(caerModuleData moduleData, size_t argsNumber, va_list args);
static void caerOpticFlowFilterConfig(caerModuleData moduleData);
static void caerOpticFlowFilterExit(caerModuleData moduleData);
static bool allocateBuffer(OpticFlowFilterState state, int16_t sourceID);
static sshsNode obtainDVS128BiasNode(sshsNode thisNode);

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
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "refractoryPeriod", 10000);

	sshsNodePutLongIfAbsent(moduleData->moduleNode, "flow_dtMin", 3000);
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "flow_dtMax", 300000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,  "flow_dx", 3);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "flow_thr1", 1E5);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "flow_thr2", 5E3);

	sshsNodePutBoolIfAbsent(moduleData->moduleNode, "filter_enable",true);
	sshsNodePutLongIfAbsent(moduleData->moduleNode, "filter_dtMax", 300000);
	sshsNodePutIntIfAbsent(moduleData->moduleNode,  "filter_dx", 3);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "filter_dMag", 1.0);
	sshsNodePutDoubleIfAbsent(moduleData->moduleNode, "filter_dAngle", 20);

	sshsNodePutByteIfAbsent(moduleData->moduleNode, "subSampleBy", 0);

	OpticFlowFilterState state = moduleData->moduleState;

	state->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode, "refractoryPeriod");

	state->flowParams.dtMin = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMin");
	state->flowParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowParams.dx 	= (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "flow_dx");
	state->flowParams.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr1");
	state->flowParams.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr2");

	state->enableFlowRegularization = sshsNodeGetBool(moduleData->moduleNode, "filter_enable");
	state->filterParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "filter_dtMax");
	state->filterParams.dx = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "filter_dx");
	state->filterParams.maxSpeedFactor = sshsNodeGetDouble(moduleData->moduleNode, "filter_dMag");
	state->filterParams.maxAngle = sshsNodeGetDouble(moduleData->moduleNode, "filter_dAngle");

	state->subSampleBy = sshsNodeGetByte(moduleData->moduleNode, "subSampleBy");

	state->wx = 0;
	state->wy = 0;

	// Add config listeners last, to avoid having them dangling if Init doesn't succeed.
	sshsNodeAddAttributeListener(moduleData->moduleNode, moduleData, &caerModuleConfigDefaultListener);

	// Identify link to DVS128 hardware for threshold regulation
	state->DVS128Node = obtainDVS128BiasNode(moduleData->moduleNode);

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

	OpticFlowFilterState state = moduleData->moduleState;

	// If the map is not allocated yet, do it.
	if (state->buffer == NULL) {
		if (!allocateBuffer(state, caerEventPacketHeaderGetEventSource((caerEventPacketHeader) flow))) {
			// Failed to allocate memory, nothing to do.
			caerLog(CAER_LOG_ERROR, moduleData->moduleSubSystemString, "Failed to allocate memory for timestampMap.");
			return;
		}
	}
	FlowEvent e;
	// Iterate over events and filter out ones that are not supported by other
	// events within a certain region in the specified timeframe.
	for (int32_t i = 0; i < caerEventPacketHeaderGetEventNumber((caerEventPacketHeader) flow); i++) {
		e = flowEventPacketGetEvent(flow,i);
		uint16_t x = caerPolarityEventGetX((caerPolarityEvent) e);
		uint16_t y = caerPolarityEventGetY((caerPolarityEvent) e);
		// Refractory period
		if (e->timestamp - flowEventBufferRead(state->buffer,x,y,0)->timestamp
				< state->refractoryPeriod) {
			caerPolarityEventInvalidate((caerPolarityEvent) e,
					(caerPolarityEventPacket) flow);
		}

		if (!caerPolarityEventIsValid((caerPolarityEvent) e)) { continue; } // Skip invalid polarity events.

		// Compute optic flow using events in buffer
		flowBenosman2014(e,state->buffer,state->flowParams);

		// Add event to buffer
		flowEventBufferAdd(e,state->buffer);

		// Apply flow regularization filter
		if (state->enableFlowRegularization) {
			flowRegularizationFilter(e,state->buffer,state->filterParams);
		}

		// For now, count events in packet and output how many have flow
		if (e->hasFlow) {
			double wxNew = e->u*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			double wyNew = e->v*DVS128_LOCAL_FLOW_TO_VENTRAL_FLOW;
			state->wx += (wxNew-state->wx)/50;
			state->wy += (wyNew-state->wy)/50;
		}
	}
	struct timespec currentTime;
	portable_clock_gettime_monotonic(&currentTime);
	int64_t timeEvent = e->timestamp;
	int64_t delay = 0;
	if (!timeSet || timeEvent < timeInitEvent) {
		timeInit = currentTime;
		timeInitEvent = timeEvent;
		timeSet = true;
	}
	else {
		int64_t timeS = currentTime.tv_sec-timeInit.tv_sec;
		int64_t timeUs = (currentTime.tv_nsec-timeInit.tv_nsec)/1000;
		int64_t timeSystem = timeS*1000000 + timeUs;
		delay = timeSystem - (timeEvent-timeInitEvent);
		if (delay < 0) {
			timeInitEvent -= delay;
		}
	}

	// DELAY CONTROL THROUGH DVS THRESHOLD SETTINGS
	if (state->DVS128Node != NULL) {
		int32_t diffOn = sshsNodeGetInt(state->DVS128Node,"diffOn");
		int32_t diffOff = sshsNodeGetInt(state->DVS128Node,"diffOff");
		if (delay > 10) {
			if (diffOn < DIFF_ON_MAX)
				diffOn *= 2;
			else
				diffOn = DIFF_ON_MAX;
			if (diffOff > DIFF_OFF_MIN)
				diffOff /= 2;
			else
				diffOff = DIFF_OFF_MIN;
		}
		else {
			if (diffOn > DIFF_ON_MIN)
				diffOn /= 2;
			else
				diffOn = DIFF_ON_MIN;
			if (diffOff < DIFF_OFF_MAX)
				diffOff *= 2;
			else
				diffOff = DIFF_OFF_MAX;
		}
		sshsNodePutInt(state->DVS128Node,"diffOn",diffOn);
		sshsNodePutInt(state->DVS128Node,"diffOff",diffOff);
	}

	fprintf(stdout, "\rwx: %1.3f. wy: %1.3f   . timeDelay: %ld ",
			state->wx, state->wy, delay/1000);
	fflush(stdout);
}

static void caerOpticFlowFilterConfig(caerModuleData moduleData) {
	caerModuleConfigUpdateReset(moduleData);

	OpticFlowFilterState state = moduleData->moduleState;

	state->refractoryPeriod = sshsNodeGetLong(moduleData->moduleNode, "refractoryPeriod");

	state->flowParams.dtMin = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMin");
	state->flowParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "flow_dtMax");
	state->flowParams.dx 	= (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "flow_dx");
	state->flowParams.thr1  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr1");
	state->flowParams.thr2  = sshsNodeGetDouble(moduleData->moduleNode, "flow_thr2");

	state->enableFlowRegularization = sshsNodeGetBool(moduleData->moduleNode, "filter_enable");
	state->filterParams.dtMax = sshsNodeGetLong(moduleData->moduleNode, "filter_dtMax");
	state->filterParams.dx = (uint16_t) sshsNodeGetInt(moduleData->moduleNode, "filter_dx");
	state->filterParams.maxSpeedFactor = sshsNodeGetDouble(moduleData->moduleNode, "filter_dMag");
	state->filterParams.maxAngle = sshsNodeGetDouble(moduleData->moduleNode, "filter_dAngle");

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

static sshsNode obtainDVS128BiasNode(sshsNode thisNode) {
	sshsNode parent = sshsNodeGetParent(thisNode);
	size_t numChildren = 1;
	sshsNode* children = sshsNodeGetChildren(parent,&numChildren);
	sshsNode DVS128Node = children[0];
	if (DVS128Node == NULL
			|| strstr(sshsNodeGetName(DVS128Node),"DVS128") == NULL) {
		caerLog(CAER_LOG_ERROR,__func__,
				"Node with name 'DVS128' not found.");
		return NULL;
	}
	children = sshsNodeGetChildren(DVS128Node,&numChildren);
	sshsNode biasNode = children[0];
	if (biasNode == NULL
			|| strstr(sshsNodeGetName(biasNode),"bias") == NULL) {
		caerLog(CAER_LOG_ERROR,__func__,
				"Node with name 'bias' not found.");
		return NULL;
	}
	return (biasNode);
}
