
set (models cambada.model goal_blue.model goal_yellow.model obstacle.model)

# Configuration file
install (FILES sim.conf.yaml DESTINATION ${BASE_DIR}/config)

# Models
install (FILES ${models} DESTINATION ${BASE_DIR}/config/worlds )
