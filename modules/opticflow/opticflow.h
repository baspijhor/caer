/*
 * opticflow.h
 *
 *  Created on: Jun 13, 2016
 *      Author: bas
 */

#ifndef OPTICFLOW_H_
#define OPTICFLOW_H_

#include "main.h"

#include <libcaer/events/polarity.h>
#include "flowEvent.h"

void caerOpticFlowFilter(uint16_t moduleID, FlowEventPacket polarity);

#endif /* OPTICFLOW_H_ */