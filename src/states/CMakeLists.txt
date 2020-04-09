set(MC_STATES_INSTALL_PREFIX "${MC_PLUGINS_INSTALL_PREFIX}/${PROJECT_NAME}/states")

add_fsm_state_simple(ApproachVisualServoing)
target_link_libraries(ApproachVisualServoing PUBLIC ${PLUGIN_NAME})

add_fsm_state_simple(MoveUntilTouch)

set(FSM_STATES_INSTALL_DIR "${MC_STATES_INSTALL_PREFIX}")
install(DIRECTORY data   
        DESTINATION ${FSM_STATES_INSTALL_DIR}   
        FILES_MATCHING PATTERN "*.yaml") 
