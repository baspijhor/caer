IF (NOT ENABLE_OPTICFLOW)
	SET(ENABLE_OPTICFLOW 0 CACHE BOOL "Enable optic flow estimation module")
ENDIF()

IF (ENABLE_OPTICFLOW)
	SET(CAER_COMPILE_DEFINITIONS ${CAER_COMPILE_DEFINITIONS} -DENABLE_OPTICFLOW=1 PARENT_SCOPE)

	SET(CAER_OPTICFLOW_FILES 
		modules/opticflow/opticflow.c
		modules/opticflow/flowBenosman2014.c
		modules/opticflow/flowRegularizationFilter.c
		modules/opticflow/uart.c
		modules/opticflow/flowOutput.c)

	SET(CAER_C_SRC_FILES ${CAER_C_SRC_FILES} ${CAER_OPTICFLOW_FILES} PARENT_SCOPE)
ENDIF()
